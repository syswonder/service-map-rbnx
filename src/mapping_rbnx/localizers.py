# SPDX-License-Identifier: MulanPSL-2.0
"""Localization on a saved map, with automatic global relocalization.

The SLAM engines can localize in their own map, but only from a pose someone
supplies: RTAB-Map's localization needs a seed close enough for scan matching to
converge, which is the "click 2D Pose Estimate, then click again" loop. A
particle filter over the saved occupancy grid does not: its
`reinitialize_global_localization` service spreads particles across the free
space and the robot converges by driving. That is what this module runs.

`load_map` calls `activate()` with the saved map directory. With an initial pose
it seeds `/initialpose`; without one it asks for global localization. Both paths
end with the localizer owning the `map → odom` transform, so the service's
`pose` / `odom` contracts keep publishing exactly as before (`tf_to_pose.py`
reads the same tf chain).

Cost: nav2_amcl with 500-2000 particles is a few tens of MB of RAM and a few
percent of one core — no GPU, no database.
"""
from __future__ import annotations

import logging
import math
import os
import shlex
import signal
import subprocess
import time
from typing import Optional

log = logging.getLogger("mapping.localizers")

LOCALIZERS = ("none", "amcl", "beluga")
DEFAULT_LOCALIZER = "none"

# Set by atlas_bridge from the deployment config at init.
_CONFIG: dict[str, object] = {}
_PROC: Optional[subprocess.Popen] = None
_ACTIVE_MAP = ""


def configure(cfg: dict) -> str:
    """Record the deployment's localization settings; returns the chosen name.

    Raises ValueError for an unknown localizer so a manifest typo fails at boot
    rather than at the first `load_map`.
    """
    name = str(cfg.get("localizer") or DEFAULT_LOCALIZER).strip().lower()
    if name not in LOCALIZERS:
        raise ValueError(
            f"localizer={name!r} invalid; expected one of {', '.join(LOCALIZERS)}"
        )
    _CONFIG.clear()
    _CONFIG.update({
        "localizer": name,
        "scan_topic": cfg.get("scan_topic") or os.environ.get("MAPPING_SCAN_TOPIC", "/scan"),
        "base_frame": cfg.get("base_frame") or "base_link",
        "odom_frame": cfg.get("odom_frame") or "odom",
        "global_frame": cfg.get("global_frame") or "map",
        "use_sim_time": bool(cfg.get("use_sim_time", False)),
        "min_particles": int(cfg.get("min_particles") or 500),
        "max_particles": int(cfg.get("max_particles") or 2000),
    })
    return name


def name() -> str:
    """Configured localizer, or `none` when the engine localizes on its own."""
    return str(_CONFIG.get("localizer") or DEFAULT_LOCALIZER)


def enabled() -> bool:
    return name() != "none"


def active_map() -> str:
    """map_id the running localizer was started on ("" when stopped)."""
    return _ACTIVE_MAP


def _launch_file() -> str:
    """Path to localization_2d.launch.py, in-container or in a source checkout."""
    for cand in (
        "/mapping/launch/localization_2d.launch.py",
        os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "launch",
                                      "localization_2d.launch.py")),
    ):
        if os.path.isfile(cand):
            return cand
    return ""


def stop() -> None:
    """Stop the localization stack if it is running. Safe to call when it is not."""
    global _PROC, _ACTIVE_MAP
    proc, _PROC, _ACTIVE_MAP = _PROC, None, ""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        for _ in range(50):
            if proc.poll() is not None:
                break
            time.sleep(0.1)
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception as e:  # noqa: BLE001
        log.warning("stopping localizer failed: %s", e)


def start(map_dir: str, map_id: str, log_path: str = "",
          owns_tf: bool = False, initial_pose=None,
          map_topic: str = "") -> tuple[bool, str]:
    """Launch map_server + the localizer on the saved map in `map_dir`.

    Replaces any running instance (loading a second map must not leave the first
    one publishing `map → odom`). Returns (ok, detail); the caller decides
    whether to seed a pose or ask for global localization.

    `owns_tf` says whether this instance publishes `map → odom`. It does so only
    when the SLAM engine has been paused for it — the two must never publish at
    once. `initial_pose` hands back a pose the filter already recovered so a
    restart into tf-owning mode does not search the map again.
    """
    global _PROC, _ACTIVE_MAP
    if not enabled():
        return False, "no localizer configured (localizer: none)"
    launch_file = _launch_file()
    if not launch_file:
        return False, "localization_2d.launch.py not found"
    map_yaml = os.path.join(map_dir, "occupancy.yaml")
    if not os.path.isfile(map_yaml):
        return False, f"missing {map_yaml} (saved map has no occupancy grid)"
    stop()
    cmd = [
        "ros2", "launch", launch_file,
        f"localizer:={name()}", f"map_yaml:={map_yaml}",
        f"scan_topic:={_CONFIG['scan_topic']}", f"base_frame:={_CONFIG['base_frame']}",
        f"odom_frame:={_CONFIG['odom_frame']}", f"global_frame:={_CONFIG['global_frame']}",
        f"use_sim_time:={'true' if _CONFIG['use_sim_time'] else 'false'}",
        f"min_particles:={_CONFIG['min_particles']}", f"max_particles:={_CONFIG['max_particles']}",
        f"tf_broadcast:={'true' if owns_tf else 'false'}",
    ]
    if map_topic:
        cmd.append(f"map_topic:={map_topic}")
    if initial_pose is not None:
        cmd += [f"initial_x:={float(initial_pose[0])}",
                f"initial_y:={float(initial_pose[1])}",
                f"initial_yaw:={float(initial_pose[2])}"]
    out = open(log_path or os.path.join("/tmp", f"localizer_{map_id or 'map'}.log"), "ab", buffering=0)
    try:
        _PROC = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    except Exception as e:  # noqa: BLE001
        return False, f"failed to launch localizer: {e}"
    _ACTIVE_MAP = map_id
    log.info("[localizer] %s on %s (%s)", name(), map_yaml, " ".join(shlex.quote(c) for c in cmd[3:]))
    return True, (f"{name()} started on {map_yaml}"
                  f"{' owning map -> odom' if owns_tf else ''}")


def wait_ready(node, timeout_s: float = 30.0) -> tuple[bool, str]:
    """Wait until the localizer's global-localization service is up, which is the
    first moment the filter is able to take a pose or a scatter request."""
    try:
        from std_srvs.srv import Empty
    except Exception as e:  # noqa: BLE001
        return False, f"std_srvs unavailable: {e}"
    client = node.create_client(Empty, "/reinitialize_global_localization")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=1.0):
            return True, "localizer ready"
        if _PROC is not None and _PROC.poll() is not None:
            return False, f"localizer exited with code {_PROC.returncode} before becoming ready"
    return False, f"localizer did not expose /reinitialize_global_localization within {timeout_s:.0f}s"


def global_localize(node, timeout_s: float = 15.0) -> tuple[bool, str]:
    """Scatter particles over the whole map — relocalize with no prior pose.

    This is the call that removes the manual "2D Pose Estimate" step: after it
    the robot converges by driving, because the filter is now representing every
    hypothesis the map allows instead of one wrong one.
    """
    try:
        from std_srvs.srv import Empty
    except Exception as e:  # noqa: BLE001
        return False, f"std_srvs unavailable: {e}"
    client = node.create_client(Empty, "/reinitialize_global_localization")
    if not client.wait_for_service(timeout_sec=min(timeout_s, 10.0)):
        return False, "/reinitialize_global_localization unavailable (localizer not running?)"
    future = client.call_async(Empty.Request())
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done():
        return False, f"global localization request timed out after {timeout_s:.0f}s"
    return True, "global localization requested (particles scattered over the map)"


# ── convergence, as the operator sees it ─────────────────────────────────────
# A particle filter that has just been told to relocalize globally is spread
# over the whole map, and it narrows down as the robot moves and scans. The
# spread is the covariance AMCL publishes on its pose, so the UI can say
# "still relocalizing" instead of showing a confident-looking arrow that is a
# guess. Thresholds are the point where the estimate is good enough to plan
# with; they are deliberately loose, because the alternative to a converging
# filter is no localization at all.
# Relocalization runs behind an already-loaded map, so its outcome is not the
# return value of anything the operator called; it is recorded here for the
# status page to report.
_RELOC: dict = {"state": "idle", "detail": ""}


def set_relocalization(state: str, detail: str = "") -> None:
    """Record how the running (or last) relocalization is doing."""
    _RELOC["state"] = state
    _RELOC["detail"] = detail


def relocalization() -> dict:
    """The recorded relocalization outcome: idle / running / done / failed."""
    return dict(_RELOC)


POSE_TOPIC = os.environ.get("MAPPING_LOCALIZER_POSE_TOPIC", "/amcl_pose")
CONVERGED_POSITION_M = 0.25
CONVERGED_YAW_RAD = 0.15


def spread(msg) -> tuple[float, float]:
    """(position stddev in metres, yaw stddev in radians) from a
    PoseWithCovarianceStamped. The 6x6 row-major covariance holds x at 0,
    y at 7 and yaw at 35."""
    cov = list(getattr(msg, "pose", msg).covariance)
    var_x, var_y, var_yaw = cov[0], cov[7], cov[35]
    position = math.sqrt(max(var_x, 0.0) + max(var_y, 0.0))
    return position, math.sqrt(max(var_yaw, 0.0))


def convergence_state(position_stddev_m: float, yaw_stddev_rad: float) -> str:
    """`converged` once the filter is tight enough to act on, else
    `converging`."""
    if (position_stddev_m <= CONVERGED_POSITION_M
            and yaw_stddev_rad <= CONVERGED_YAW_RAD):
        return "converged"
    return "converging"


# A particle filter disambiguates by MOVING: a room whose walls look alike from
# several places gives near-identical scans, and a filter standing still can
# collapse onto the wrong one of them — tightly, and therefore confidently. So
# convergence is not spread alone; the robot must also have travelled far enough
# for the spread to mean something. Measured in the office world: a stationary
# filter reported ±0.2 m while sitting 4 m and 152° from the truth.
MIN_TRAVEL_M = 1.5

# A particle filter reports how tightly its particles agree, not whether they
# agree about the right place. In a room with repeated structure the cloud can
# collapse onto a wrong hypothesis and report centimetres of spread while the
# robot stands metres away, so the estimate is checked against the map the only
# way that can fail independently: by asking whether the laser the robot is
# seeing right now is the laser it would see from where it thinks it is.
# Measured on this deployment's own map while the robot was known to be right
# and known to be wrong: a correct pose reads 0.70 to 1.00, a pose five metres
# out reads 0.49. Below this the fix is not acted on.
SCAN_FIT_MIN = float(os.environ.get("MAPPING_SCAN_FIT_MIN", "0.60"))
SCAN_FIT_TOLERANCE_CELLS = 3


class _Grid:
    """A saved occupancy map, in map-frame metres.

    Occupied and known are kept apart because a saved map is mostly neither:
    half of a hand-driven map is territory the robot never saw, and a beam that
    ends there says nothing about where the robot is.
    """

    def __init__(self, width, height, occupied, known, resolution, origin):
        self.width = width
        self.height = height
        self.cells = occupied       # bytearray, 1 = occupied
        self.known = known          # bytearray, 1 = free or occupied
        self.resolution = resolution
        self.origin = origin

    def _index(self, x: float, y: float):
        col = int((x - self.origin[0]) / self.resolution)
        row = int((y - self.origin[1]) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return row * self.width + col
        return None

    def is_known(self, x: float, y: float) -> bool:
        i = self._index(x, y)
        return i is not None and bool(self.known[i])

    def occupied_near(self, x: float, y: float, tolerance_cells: int) -> bool:
        col = int((x - self.origin[0]) / self.resolution)
        row = int((y - self.origin[1]) / self.resolution)
        if not (0 <= col < self.width and 0 <= row < self.height):
            return False
        for dr in range(-tolerance_cells, tolerance_cells + 1):
            r = row + dr
            if not (0 <= r < self.height):
                continue
            base = r * self.width
            for dc in range(-tolerance_cells, tolerance_cells + 1):
                c = col + dc
                if 0 <= c < self.width and self.cells[base + c]:
                    return True
        return False


def load_grid(map_dir: str):
    """Read `occupancy.{yaml,pgm}` from a saved map, or None if unreadable.

    Only the occupied cells matter here, so the threshold from the yaml is
    applied once and the grid is kept as one byte per cell.
    """
    yaml_path = os.path.join(map_dir, "occupancy.yaml")
    pgm_path = os.path.join(map_dir, "occupancy.pgm")
    try:
        import yaml as _yaml
        meta = _yaml.safe_load(open(yaml_path, encoding="utf-8"))
        resolution = float(meta["resolution"])
        origin = (float(meta["origin"][0]), float(meta["origin"][1]))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        negate = int(meta.get("negate", 0))
        with open(pgm_path, "rb") as f:
            if f.readline().strip() != b"P5":
                return None
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            width, height = (int(v) for v in line.split()[:2])
            maxval = int(f.readline().split()[0])
            raw = f.read(width * height)
    except Exception as e:  # noqa: BLE001
        log.warning("[localizer] cannot read the saved grid in %s: %s", map_dir, e)
        return None
    # map_server's convention: darker is more occupied unless negate is set.
    # A saved grid is trinary -- occupied, free, and the unobserved value that
    # sits between the two thresholds -- and the middle one is not evidence.
    free_thresh = float(meta.get("free_thresh", 0.25))
    occupied = bytearray(width * height)
    known = bytearray(width * height)
    for i, v in enumerate(raw):
        p_occ = (v / maxval) if negate else (1.0 - v / maxval)
        if p_occ >= occupied_thresh:
            occupied[i] = 1
            known[i] = 1
        elif p_occ <= free_thresh and v >= maxval - 5:
            known[i] = 1
    # The PGM's first row is the top of the map; the grid's first row is the
    # bottom, so the rows are flipped once here rather than on every lookup.
    def _flip(buf):
        out = bytearray(width * height)
        for r in range(height):
            out[r * width:(r + 1) * width] = buf[(height - 1 - r) * width:(height - r) * width]
        return out

    return _Grid(width, height, _flip(occupied), _flip(known), resolution, origin)


def scan_fit_of(grid, msg, pose) -> tuple[float, str]:
    """Fraction of one scan's endpoints that land on a wall of `grid`.

    Returns (fraction, detail); the fraction is -1.0 when the scan says nothing
    about the pose, so "no evidence" is never mistaken for "bad fix". Beams that
    end in territory the map never observed are not evidence either way and are
    left out; beams that end where the map says free space are, and they count
    against the pose.
    """
    if grid is None or msg is None:
        return -1.0, "no map or no scan to check against"
    x, y, yaw = pose
    hits = 0
    total = 0
    angle = msg.angle_min
    for r in msg.ranges:
        a = angle
        angle += msg.angle_increment
        if not (msg.range_min < r < msg.range_max) or r != r:
            continue
        ex = x + r * math.cos(yaw + a)
        ey = y + r * math.sin(yaw + a)
        if grid.occupied_near(ex, ey, SCAN_FIT_TOLERANCE_CELLS):
            hits += 1
            total += 1
        elif grid.is_known(ex, ey):
            # Known-free: the map says there is nothing here, and there is.
            total += 1
    if total == 0:
        return -1.0, ("no beam ended in mapped territory — the robot is looking "
                      "at something the map does not cover")
    return hits / total, f"{hits}/{total} beams over mapped ground land on a wall"


def scan_fit(node, grid, pose, timeout_s: float = 6.0) -> tuple[float, str]:
    """`scan_fit_of` on the next scan to arrive on the configured topic."""
    if grid is None:
        return -1.0, "no saved grid to check against"
    try:
        from sensor_msgs.msg import LaserScan
    except Exception as e:  # noqa: BLE001
        return -1.0, f"sensor_msgs unavailable: {e}"
    import threading

    got = threading.Event()
    latest: dict = {"msg": None}

    def _on_scan(msg) -> None:
        latest["msg"] = msg
        got.set()

    topic = _CONFIG.get("scan_topic") or "/scan"
    sub = node.create_subscription(LaserScan, topic, _on_scan, 5)
    try:
        got.wait(timeout_s)
    finally:
        try:
            node.destroy_subscription(sub)
        except Exception:  # noqa: BLE001
            pass
    if latest["msg"] is None:
        return -1.0, f"no scan on {topic} within {timeout_s:.0f}s"
    return scan_fit_of(grid, latest["msg"], pose)




# Accepting a static match. The likelihood is high for any pose that explains
# the scan; what makes an answer trustworthy is that no OTHER place explains it
# nearly as well. Correct matches on this map lead their nearest real rival by
# 12 to 39 points, so a margin of 8 leaves room while still refusing a tie --
# and a tie is exactly the case where the operator should be asked for a nudge.
STATIC_MATCH_MIN = float(os.environ.get("MAPPING_STATIC_MATCH_MIN", "0.75"))
STATIC_MATCH_MARGIN = float(os.environ.get("MAPPING_STATIC_MATCH_MARGIN", "0.08"))


def _global_match_module():
    """Import the matcher lazily; it needs numpy, which not every image has."""
    from mapping_rbnx import global_match
    return global_match


def relocalize_statically(node, grid, timeout_s: float = 8.0):
    """Find the robot on `grid` from one scan, without moving it.

    Returns (pose, detail) with pose None when the scan does not single a place
    out. Standing still is the normal case on a robot that has just been
    switched on or carried somewhere, and a particle filter cannot serve it:
    its update is driven by motion, so with the robot still the cloud never
    sharpens. Matching the scan against the map has no such requirement.
    """
    try:
        matcher = _global_match_module()
    except Exception as e:  # noqa: BLE001
        return None, f"static matching unavailable: {e}"
    try:
        from sensor_msgs.msg import LaserScan
    except Exception as e:  # noqa: BLE001
        return None, f"sensor_msgs unavailable: {e}"
    import threading

    got = threading.Event()
    latest: dict = {"msg": None}

    def _on_scan(msg) -> None:
        latest["msg"] = msg
        got.set()

    topic = _CONFIG.get("scan_topic") or "/scan"
    from rclpy.qos import qos_profile_sensor_data
    sub = node.create_subscription(LaserScan, topic, _on_scan, qos_profile_sensor_data)
    try:
        got.wait(timeout_s)
    finally:
        try:
            node.destroy_subscription(sub)
        except Exception:  # noqa: BLE001
            pass
    if latest["msg"] is None:
        return None, f"no scan on {topic} within {timeout_s:.0f}s"

    match = matcher.global_scan_match(grid, latest["msg"])
    if match is None:
        return None, "the scan does not cover enough of the map to place the robot"
    if match.score < STATIC_MATCH_MIN:
        return None, (f"no place on the map explains this scan ({match.detail}) — "
                      f"is the robot on this map at all?")
    if match.score - match.runner_up < STATIC_MATCH_MARGIN:
        return None, (f"two places on the map explain this scan equally well "
                      f"({match.detail}) — move the robot a metre and try again")
    return match.pose, f"matched the map standing still: {match.detail}"


def current_pose(node, timeout_s: float = 5.0):
    """The filter's latest estimate as (x, y, yaw), or None if none arrives.

    Used at the moment another node is ready to take the pose over: the
    estimate the filter converged on is stale by then, because the robot kept
    driving while that node started, and handing over a stale pose put the
    robot metres behind where it stood.
    """
    try:
        from geometry_msgs.msg import PoseWithCovarianceStamped
    except Exception:  # noqa: BLE001
        return None
    import threading

    got = threading.Event()
    latest: dict = {"msg": None}

    def _on_pose(msg) -> None:
        latest["msg"] = msg
        got.set()

    sub = node.create_subscription(PoseWithCovarianceStamped, POSE_TOPIC, _on_pose, 10)
    try:
        got.wait(timeout_s)
    finally:
        try:
            node.destroy_subscription(sub)
        except Exception:  # noqa: BLE001
            pass
    msg = latest["msg"]
    if msg is None:
        return None
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (p.x, p.y, yaw)


def wait_for_convergence(node, timeout_s: float = 60.0,
                         min_travel_m: float = MIN_TRAVEL_M,
                         map_dir: str = "") -> tuple[bool, tuple[float, float, float], str]:
    """Watch the filter until it agrees with itself, has moved, and matches the
    map, or give up.

    Returns (converged, (x, y, yaw), detail). The pose is the filter's own
    estimate off `POSE_TOPIC` — the whole point of the localizer slot is to
    recover it after a load with no prior, so once it is recovered the filter
    has done its job and the SLAM engine can carry on from it.

    All three conditions are needed and none implies another: a stationary
    filter collapses tightly wherever it started, a moving one can collapse
    tightly onto the wrong room, and only the scan check can tell the wrong
    room from the right one. Failing the scan check re-scatters the particles
    rather than giving up, because the robot is still driving and the right
    hypothesis is still reachable.
    """
    import math

    try:
        from geometry_msgs.msg import PoseWithCovarianceStamped
    except Exception as e:  # noqa: BLE001
        return False, (0.0, 0.0, 0.0), f"geometry_msgs unavailable: {e}"

    try:
        from nav_msgs.msg import Odometry
    except Exception as e:  # noqa: BLE001
        return False, (0.0, 0.0, 0.0), f"nav_msgs unavailable: {e}"

    grid = load_grid(map_dir) if map_dir else None
    latest: dict = {"msg": None}
    travel: dict = {"m": 0.0, "last": None}
    rejected = 0

    def _on_pose(msg) -> None:
        latest["msg"] = msg

    def _on_odom(msg) -> None:
        p = msg.pose.pose.position
        last = travel["last"]
        if last is not None:
            travel["m"] += math.hypot(p.x - last[0], p.y - last[1])
        travel["last"] = (p.x, p.y)

    odom_topic = os.environ.get("MAPPING_ODOM_TOPIC", _CONFIG.get("odom_topic") or "/odom")
    sub = node.create_subscription(PoseWithCovarianceStamped, POSE_TOPIC, _on_pose, 10)
    odom_sub = node.create_subscription(Odometry, odom_topic, _on_odom, 20)
    try:
        deadline = time.monotonic() + timeout_s
        best = ""
        while time.monotonic() < deadline:
            msg = latest["msg"]
            if msg is not None:
                position_sd, yaw_sd = spread(msg)
                moved = travel["m"]
                best = (f"position ±{position_sd:.2f} m, heading ±{math.degrees(yaw_sd):.0f}°, "
                        f"travelled {moved:.2f} m")
                if convergence_state(position_sd, yaw_sd) == "converged" and moved >= min_travel_m:
                    p, q = msg.pose.pose.position, msg.pose.pose.orientation
                    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    fit, fit_detail = scan_fit(node, grid, (p.x, p.y, yaw))
                    if fit < 0.0:
                        best = f"{best}; scan check skipped: {fit_detail}"
                        return True, (p.x, p.y, yaw), f"converged ({best})"
                    best = f"{best}, scan fit {fit:.0%} ({fit_detail})"
                    if fit >= SCAN_FIT_MIN:
                        return True, (p.x, p.y, yaw), f"converged ({best})"
                    rejected += 1
                    log.warning("[localizer] rejecting a tight fix that does not match "
                                "the map: %s", best)
                    global_localize(node)
                    travel["m"] = 0.0
                    time.sleep(2.0)
                    continue
            time.sleep(0.25)
        if not best:
            return False, (0.0, 0.0, 0.0), f"{name()} published no pose within {timeout_s:.0f}s"
        if travel["m"] < min_travel_m:
            return False, (0.0, 0.0, 0.0), (
                f"{name()} needs the robot to move to tell similar places apart: "
                f"only {travel['m']:.2f} m of the {min_travel_m:.1f} m it wants "
                f"within {timeout_s:.0f}s ({best})")
        if rejected:
            return False, (0.0, 0.0, 0.0), (
                f"{name()} converged {rejected} time(s) on a pose the laser does not "
                f"support and was re-scattered each time; last was {best}")
        return False, (0.0, 0.0, 0.0), f"{name()} did not converge within {timeout_s:.0f}s ({best})"
    finally:
        for s_ in (sub, odom_sub):
            try:
                node.destroy_subscription(s_)
            except Exception:  # noqa: BLE001
                pass
