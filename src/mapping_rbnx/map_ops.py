# SPDX-License-Identifier: MulanPSL-2.0
"""Runtime map-management operations for the mapping service.

Backs three RPC+MCP capabilities (declared in atlas_bridge):
  - save_map      snapshot the live SLAM map to disk under a map_id
  - load_map      switch rtabmap onto a saved map (localization / mapping)
  - pose_estimate seed a pose so rtabmap's localization re-converges

These talk to the *running* rtabmap (launched as a separate process by
start_engine.sh in the same ROS graph) over DDS — this module spins its own
lightweight rclpy node, independent of the SLAM launch.

load_map strategy (per design): try rtabmap's runtime services FIRST
(`/rtabmap/load_database` + `/set_mode_localization|mapping`); only if those
are unavailable does the caller fall back to a process restart. Each impl
returns a plain dict whose keys match the contract's response fields, so the
gRPC servicer and the MCP handler in atlas_bridge can share one code path.
"""
from __future__ import annotations

import math
import os
import threading
import time
from typing import Optional

import logging

from mapping_rbnx import lifecycle

log = logging.getLogger("mapping_rbnx.map_ops")

MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")
PKG_HOST_DIR = os.environ.get("ROBONIX_PKG_HOST_DIR", "/mapping")

# rtabmap node name prefix; the launch runs the slam node as `/rtabmap/rtabmap`
# so its services live under `/rtabmap/...`.
RTABMAP_NS = os.environ.get("MAPPING_RTABMAP_NS", "/rtabmap")
# Where rtabmap subscribes for an externally-seeded pose. The launch remaps
# the standard `/initialpose` into rtabmap; keep them in sync.
INITIALPOSE_TOPIC = os.environ.get("MAPPING_INITIALPOSE_TOPIC", "/initialpose")
# Live map-frame pose (PoseWithCovarianceStamped), published by the tf_to_pose
# adapter on the bound `robonix/service/map/pose` contract. get_pose reads it.
POSE_TOPIC = os.environ.get("MAPPING_POSE_TOPIC", "/robonix/map/pose")


def _sanitize_map_id(map_id: str) -> str:
    import re
    return re.sub(r"[^A-Za-z0-9._-]", "_", (map_id or "").strip()) or "default"


# ── rclpy node (lazy, shared, own spin thread) ────────────────────────────────
_node = None
_node_lock = threading.Lock()


def _get_node():
    """Create (once) an rclpy node + background executor so map ops can call
    rtabmap services / publish poses. Returns None if rclpy is unavailable
    (e.g. ROS not sourced) — callers degrade to a clear error."""
    global _node
    with _node_lock:
        if _node is not None:
            return _node
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node("mapping_map_ops")
            ex = SingleThreadedExecutor()
            ex.add_node(node)
            t = threading.Thread(target=ex.spin, daemon=True)
            t.start()
            node._robonix_executor = ex  # keep refs alive
            node._robonix_spin = t
            _node = node
            log.info("map_ops rclpy node up (ns=%s)", RTABMAP_NS)
            return _node
        except Exception as e:  # noqa: BLE001
            log.warning("map_ops: rclpy node unavailable: %s", e)
            return None


def _yaw_to_quat(theta: float):
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


def _call_service(node, srv_type, name: str, request, timeout_s: float = 5.0):
    """Blocking service call from the spinning node. Returns (ok, result_or_err)."""
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return False, f"service {name} unavailable"
    fut = cli.call_async(request)
    deadline = time.time() + timeout_s
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not fut.done():
        return False, f"service {name} timed out"
    return True, fut.result()


# ── pose_estimate ─────────────────────────────────────────────────────────────
def pose_estimate_impl(x: float, y: float, theta: float,
                       cov_xy: float = 0.0, cov_theta: float = 0.0) -> dict:
    """Publish a PoseWithCovarianceStamped to INITIALPOSE_TOPIC so rtabmap
    re-localizes from the given guess. Returns {ok, detail}."""
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    try:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(float(theta))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Diagonal covariance: [x, y, z, roll, pitch, yaw]. Default to the
        # conventional rviz "2D Pose Estimate" values when caller passes 0.
        var_xy = (cov_xy or 0.25) ** 2 if cov_xy else 0.25
        var_yaw = (cov_theta or 0.07) ** 2 if cov_theta else 0.068
        cov = [0.0] * 36
        cov[0] = var_xy
        cov[7] = var_xy
        cov[35] = var_yaw
        msg.pose.covariance = cov
        # Latch one publish; a fresh publisher each call keeps this stateless.
        pub = node.create_publisher(PoseWithCovarianceStamped, INITIALPOSE_TOPIC, 1)
        # DDS needs a beat to match the subscriber before the sample is kept.
        time.sleep(0.3)
        pub.publish(msg)
        time.sleep(0.2)
        node.destroy_publisher(pub)
        return {"ok": True, "detail": f"seeded pose ({x:.2f}, {y:.2f}, {theta:.2f}) on {INITIALPOSE_TOPIC}"}
    except Exception as e:  # noqa: BLE001
        log.exception("pose_estimate failed")
        return {"ok": False, "detail": str(e)}


# ── load_map ──────────────────────────────────────────────────────────────────
def load_map_impl(map_id: str, mode: str = "localization",
                  has_initial_pose: bool = False,
                  x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> dict:
    """Switch the running rtabmap onto <map_id>'s database.

    Strategy B (preferred, no restart): call `/rtabmap/load_database` with the
    saved db path, then `/rtabmap/set_mode_localization` or `set_mode_mapping`.
    If those services don't exist on this rtabmap build, return ok=False with a
    clear hint so the operator can fall back to a service restart (strategy A).
    """
    map_id = _sanitize_map_id(map_id)
    mode = (mode or "localization").strip().lower()
    if mode not in ("localization", "mapping"):
        return {"ok": False, "detail": f"mode={mode!r} invalid (localization|mapping)"}
    db_path = os.path.join(MAPS_DIR, map_id, "rtabmap.db")
    if not os.path.isfile(db_path):
        return {"ok": False, "detail": f"no saved map at {db_path}"}

    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}

    try:
        # 1. load_database (rtabmap_msgs/srv/LoadDatabase: string database_path, bool clear)
        try:
            from rtabmap_msgs.srv import LoadDatabase
        except Exception:  # noqa: BLE001
            return {"ok": False,
                    "detail": "rtabmap_msgs/LoadDatabase not available — fall back to "
                              "restart with map_mode/map_id config (strategy A)"}
        req = LoadDatabase.Request()
        req.database_path = db_path
        req.clear = False
        ok, res = _call_service(node, LoadDatabase, f"{RTABMAP_NS}/load_database", req, timeout_s=15.0)
        if not ok:
            return {"ok": False, "detail": f"{res} — fall back to restart (strategy A)"}

        # 2. mode switch
        ok2, info2 = _set_mode(node, mode)
        if not ok2:
            return {"ok": False, "detail": f"loaded db but mode switch failed: {info2}"}

        # 3. optional pose seed for fast convergence
        seeded = ""
        if has_initial_pose:
            ps = pose_estimate_impl(x, y, theta)
            seeded = f"; {ps['detail']}"
        set_current_mode(mode)
        # Broadcast the new identity: a localization load keeps the saved
        # map's frame epoch; a mapping load re-derives it (see lifecycle.py).
        lifecycle.set_state(map_id, mode, bump=(mode == "mapping"))
        return {"ok": True, "detail": f"loaded {map_id} in {mode} mode{seeded}"}
    except Exception as e:  # noqa: BLE001
        log.exception("load_map failed")
        return {"ok": False, "detail": str(e)}


# ── mode tracking (get_mode) ──────────────────────────────────────────────────
# Single source of truth for "which SLAM mode is in effect right now", updated
# by init (startup map_mode), switch_mode and load_map — so get_mode reflects
# the real runtime mode regardless of how it changed (config, MCP, or webui).
_current_mode: str = ""


def set_current_mode(mode: str) -> None:
    """Record the SLAM mode now in effect. Called by atlas_bridge.init with the
    startup map_mode and by switch_mode_impl / load_map_impl on success."""
    global _current_mode
    if mode:
        _current_mode = mode.strip().lower()


def get_mode_impl() -> dict:
    """Return the SLAM mode currently in effect (read-only). Returns
    {ok, mode, detail}; mode is "" with ok=False before init has run."""
    if not _current_mode:
        return {"ok": False, "mode": "", "detail": "mode not initialized yet"}
    return {"ok": True, "mode": _current_mode, "detail": ""}


def get_pose_impl(timeout_s: float = 2.0) -> dict:
    """Read the robot's current pose in the MAP frame from the live pose topic
    (PoseWithCovarianceStamped on POSE_TOPIC). Returns
    {ok, x, y, theta (yaw rad), frame_id, detail}. ok=False with a hint if no
    pose arrives within timeout_s (mapping not localized / not publishing)."""
    node = _get_node()
    if node is None:
        return {"ok": False, "x": 0.0, "y": 0.0, "theta": 0.0, "frame_id": "",
                "detail": "rclpy node unavailable (ROS not running?)"}
    try:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                               DurabilityPolicy, HistoryPolicy)
        got = threading.Event()
        holder: dict = {}

        def _cb(msg):
            holder["msg"] = msg
            got.set()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        sub = node.create_subscription(PoseWithCovarianceStamped, POSE_TOPIC, _cb, qos)
        try:
            got.wait(timeout=timeout_s)
        finally:
            node.destroy_subscription(sub)
        if "msg" not in holder:
            return {"ok": False, "x": 0.0, "y": 0.0, "theta": 0.0, "frame_id": "",
                    "detail": f"no pose on {POSE_TOPIC} within {timeout_s:.1f}s "
                              "(is mapping localized / publishing?)"}
        msg = holder["msg"]
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {"ok": True, "x": float(p.position.x), "y": float(p.position.y),
                "theta": float(yaw), "frame_id": msg.header.frame_id or "map",
                "detail": ""}
    except Exception as e:  # noqa: BLE001
        log.exception("get_pose failed")
        return {"ok": False, "x": 0.0, "y": 0.0, "theta": 0.0, "frame_id": "",
                "detail": str(e)}


# ── switch_mode ───────────────────────────────────────────────────────────────
def _set_mode(node, mode: str) -> tuple[bool, str]:
    """Call rtabmap's set_mode_localization|set_mode_mapping (std_srvs/Empty)."""
    from std_srvs.srv import Empty
    srv = "set_mode_localization" if mode == "localization" else "set_mode_mapping"
    ok, res = _call_service(node, Empty, f"{RTABMAP_NS}/{srv}", Empty.Request(), timeout_s=5.0)
    return (ok, srv if ok else str(res))


def switch_mode_impl(mode: str) -> dict:
    """Flip the running rtabmap between mapping and localization on the CURRENT
    map — no map load, no restart. Returns {ok, detail}."""
    mode = (mode or "").strip().lower()
    if mode not in ("localization", "mapping"):
        return {"ok": False, "detail": f"mode={mode!r} invalid (localization|mapping)"}
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    try:
        ok, info = _set_mode(node, mode)
        if not ok:
            return {"ok": False, "detail": f"{info} — rtabmap may lack the mode service "
                                           "(fall back to restart with config map_mode)"}
        set_current_mode(mode)
        # Mode flip only — the live frame does not move, so no generation bump.
        lifecycle.set_mode(mode)
        return {"ok": True, "detail": f"switched to {mode} mode"}
    except Exception as e:  # noqa: BLE001
        log.exception("switch_mode failed")
        return {"ok": False, "detail": str(e)}


# ── reset_map ─────────────────────────────────────────────────────────────────
def reset_map_impl() -> dict:
    """Wipe the running rtabmap's map (working memory + live database) and
    restart SLAM from scratch — for when mapping has diverged and you want a
    clean rebuild without a full redeploy. Calls rtabmap's `/rtabmap/reset`
    (std_srvs/Empty).

    Caveat: rtabmap restarts with the robot's CURRENT pose as the new origin,
    so the rebuilt map's frame will NOT align with the pre-reset one (origin
    drift). Saved maps on disk are untouched. Returns {ok, detail}.
    """
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    try:
        from std_srvs.srv import Empty
        ok, res = _call_service(node, Empty, f"{RTABMAP_NS}/reset", Empty.Request(), timeout_s=10.0)
        if not ok:
            return {"ok": False, "detail": f"{res} — rtabmap /reset unavailable "
                                           "(fall back to restart with config)"}
        # Same map_id, new origin: bump the frame epoch so consumers know
        # their stored map-frame coordinates just went stale.
        lifecycle.mark_reset()
        return {"ok": True, "detail": "map cleared — rebuilding from current pose "
                                      "(origin reset; new frame won't match the old map)"}
    except Exception as e:  # noqa: BLE001
        log.exception("reset_map failed")
        return {"ok": False, "detail": str(e)}


# ── save_map ──────────────────────────────────────────────────────────────────
def save_map_impl(map_id: str, note: str = "",
                  active_db: Optional[str] = None) -> dict:
    """Snapshot the current SLAM map under {MAPS_DIR}/<map_id>/.

    The rtabmap database is written live by rtabmap at its database_path; this
    adds the portable preview artifacts (occupancy pgm/png, cloud pcd, meta)
    via scripts/save_map.py and, when the live db lives elsewhere (ephemeral
    run), copies it in so <map_id> is fully self-contained and loadable later.
    Returns {ok, map_id, database_path, detail}.
    """
    map_id = _sanitize_map_id(map_id)
    map_dir = os.path.join(MAPS_DIR, map_id)
    db_path = os.path.join(map_dir, "rtabmap.db")
    try:
        os.makedirs(map_dir, exist_ok=True)

        # Resolve the live rtabmap db to copy so the saved map is
        # self-contained. The caller passes active_db for a named-map run; an
        # ephemeral run (no map_id) leaves it empty, and rtabmap then writes to
        # its default ~/.ros/rtabmap.db. Fall back to that (and an explicit
        # RTABMAP_DATABASE_PATH override) so a Save in ephemeral mode still
        # captures a real db instead of a preview-only "no db" entry.
        live_db = active_db if (active_db and os.path.isfile(active_db)) else None
        if live_db is None:
            for cand in (
                os.environ.get("RTABMAP_DATABASE_PATH", ""),
                os.path.expanduser("~/.ros/rtabmap.db"),
            ):
                if cand and os.path.isfile(cand):
                    live_db = cand
                    break
        if live_db and os.path.abspath(live_db) != os.path.abspath(db_path):
            import shutil
            shutil.copy2(live_db, db_path)

        # Portable preview (pgm/png/pcd/meta) from the live /map + cloud topics.
        # Locate save_map.py RELATIVE TO THIS MODULE, not via PKG_HOST_DIR:
        # PKG_HOST_DIR is the *host* path (for atlas advertising) and does not
        # exist inside the docker container where this code runs, so using it
        # made the file check fail and the preview (occupancy.png) was silently
        # skipped — the saved map then showed a broken image in the UI library.
        # __file__ is <pkg>/src/mapping_rbnx/map_ops.py in both host and
        # container, so ../../scripts/save_map.py resolves correctly either way.
        import subprocess
        script = os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "..", "scripts", "save_map.py")
        )
        if os.path.isfile(script):
            subprocess.run(
                ["python3", script, "--out-dir", map_dir, "--timeout", "10"],
                check=False, timeout=30,
            )
        else:
            log.warning("save_map.py not found at %s — db saved, preview skipped", script)

        have_db = os.path.isfile(db_path)
        return {
            "ok": True,
            "map_id": map_id,
            "database_path": db_path if have_db else "",
            "detail": f"saved {map_id} → {map_dir}" + ("" if have_db else " (preview only; live db elsewhere)"),
        }
    except Exception as e:  # noqa: BLE001
        log.exception("save_map failed for %s", map_id)
        return {"ok": False, "map_id": map_id, "database_path": "", "detail": str(e)}


# ── delete_map ────────────────────────────────────────────────────────────────
def delete_map_impl(map_id: str) -> dict:
    """Remove a saved map's directory ({MAPS_DIR}/<map_id>/) and all its
    artifacts (db + preview). Refuses an empty id or a missing map. Does not
    touch the live SLAM session — only on-disk storage. Returns {ok, detail}."""
    map_id = _sanitize_map_id(map_id)
    map_dir = os.path.join(MAPS_DIR, map_id)
    try:
        if not os.path.isdir(map_dir):
            return {"ok": False, "map_id": map_id, "detail": f"no saved map {map_id!r}"}
        import shutil
        shutil.rmtree(map_dir)
        return {"ok": True, "map_id": map_id, "detail": f"deleted {map_id}"}
    except Exception as e:  # noqa: BLE001
        log.exception("delete_map failed for %s", map_id)
        return {"ok": False, "map_id": map_id, "detail": str(e)}
