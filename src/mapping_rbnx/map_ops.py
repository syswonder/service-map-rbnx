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

log = logging.getLogger("mapping_rbnx.map_ops")

MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")
PKG_HOST_DIR = os.environ.get("ROBONIX_PKG_HOST_DIR", "/mapping")

# rtabmap node name prefix; the launch runs the slam node as `/rtabmap/rtabmap`
# so its services live under `/rtabmap/...`.
RTABMAP_NS = os.environ.get("MAPPING_RTABMAP_NS", "/rtabmap")
# Where rtabmap subscribes for an externally-seeded pose. The launch remaps
# the standard `/initialpose` into rtabmap; keep them in sync.
INITIALPOSE_TOPIC = os.environ.get("MAPPING_INITIALPOSE_TOPIC", "/initialpose")


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
        from std_srvs.srv import Empty
        mode_srv = "set_mode_localization" if mode == "localization" else "set_mode_mapping"
        ok2, res2 = _call_service(node, Empty, f"{RTABMAP_NS}/{mode_srv}", Empty.Request(), timeout_s=5.0)
        if not ok2:
            return {"ok": False, "detail": f"loaded db but {mode_srv} failed: {res2}"}

        # 3. optional pose seed for fast convergence
        seeded = ""
        if has_initial_pose:
            ps = pose_estimate_impl(x, y, theta)
            seeded = f"; {ps['detail']}"
        return {"ok": True, "detail": f"loaded {map_id} in {mode} mode{seeded}"}
    except Exception as e:  # noqa: BLE001
        log.exception("load_map failed")
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

        # If rtabmap's live db is a different file (ephemeral / different id),
        # copy it in so the saved map is self-contained.
        if active_db and os.path.isfile(active_db) and os.path.abspath(active_db) != os.path.abspath(db_path):
            import shutil
            shutil.copy2(active_db, db_path)

        # Portable preview (pgm/png/pcd/meta) from the live /map + cloud topics.
        import subprocess
        script = os.path.join(PKG_HOST_DIR, "scripts", "save_map.py")
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
