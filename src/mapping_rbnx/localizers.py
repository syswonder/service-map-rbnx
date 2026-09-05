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


def start(map_dir: str, map_id: str, log_path: str = "") -> tuple[bool, str]:
    """Launch map_server + the localizer on the saved map in `map_dir`.

    Replaces any running instance (loading a second map must not leave the first
    one publishing `map → odom`). Returns (ok, detail); the caller decides
    whether to seed a pose or ask for global localization.
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
    ]
    out = open(log_path or os.path.join("/tmp", f"localizer_{map_id or 'map'}.log"), "ab", buffering=0)
    try:
        _PROC = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    except Exception as e:  # noqa: BLE001
        return False, f"failed to launch localizer: {e}"
    _ACTIVE_MAP = map_id
    log.info("[localizer] %s on %s (%s)", name(), map_yaml, " ".join(shlex.quote(c) for c in cmd[3:]))
    return True, f"{name()} started on {map_yaml}"


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
