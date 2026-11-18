# SPDX-License-Identifier: MulanPSL-2.0
"""Runtime map-management operations for the mapping service.

Backs map RPC+MCP capabilities (declared in atlas_bridge):
  - save_map      snapshot the live SLAM map to disk under a map_id
  - load_map      switch rtabmap onto a saved map (localization / mapping)
  - pose_estimate seed a pose so localization can be verified
  - get_pose_candidates offer Top-K only when the scan is ambiguous

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

import json
import math
import os
import shutil
import glob
import sqlite3
import threading
import time
import functools
from pathlib import Path

import logging

from mapping_rbnx import engines, lifecycle, localizers
from mapping_rbnx.localization_flow import LocalizationFlow

log = logging.getLogger("mapping_rbnx.map_ops")

MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")
PKG_HOST_DIR = os.environ.get("ROBONIX_PKG_HOST_DIR", "/mapping")
RUNTIME_DB_DIR = os.environ.get("MAPPING_RUNTIME_DB_DIR", "/tmp/robonix-mapping-runtime")

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


def _sqlite_quick_check(db_path: str) -> tuple[bool, str]:
    """Return whether db_path is a readable SQLite database.

    RTAB-Map stores maps in SQLite. Loading a partially-copied live DB can
    crash rtabmap, so validate before exposing or loading a saved map.
    """
    if not os.path.isfile(db_path):
        return False, "missing rtabmap.db"
    try:
        con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=30.0)
        try:
            row = con.execute("PRAGMA quick_check").fetchone()
            msg = str(row[0]) if row else "no quick_check result"
            return msg.lower() == "ok", msg
        finally:
            con.close()
    except Exception as e:  # noqa: BLE001
        return False, str(e)


def _sqlite_backup(src: str, dst: str) -> tuple[bool, str]:
    """Consistently snapshot a live SQLite DB using sqlite3's backup API.

    A plain file copy of ~/.ros/rtabmap.db races RTAB-Map's writer and can
    produce "database disk image is malformed" on load. SQLite backup takes a
    transactionally-consistent snapshot while the source remains live.
    """
    if not os.path.isfile(src):
        return False, f"live database not found: {src}"
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    tmp = dst + ".tmp"
    try:
        if os.path.exists(tmp):
            os.remove(tmp)
        src_con = sqlite3.connect(f"file:{src}?mode=ro", uri=True, timeout=60.0)
        dst_con = sqlite3.connect(tmp, timeout=60.0)
        try:
            src_con.backup(dst_con, pages=1024, sleep=0.05)
        finally:
            dst_con.close()
            src_con.close()
        ok, detail = _sqlite_quick_check(tmp)
        if not ok:
            try:
                os.remove(tmp)
            except OSError:
                pass
            return False, f"backup integrity check failed: {detail}"
        os.replace(tmp, dst)
        return True, "sqlite backup ok"
    except Exception as e:  # noqa: BLE001
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
        except OSError:
            pass
        return False, str(e)


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


# One owner for startup, operator selection and Relocalize.
_LOCALIZATION = LocalizationFlow()

# Only slam_toolbox enters these locks. Commands serialize cancellation before
# taking the engine lock, so cancelling a handover cannot deadlock its worker.
_SLAM_COMMAND = threading.RLock()
_SLAM_OPERATION = threading.RLock()
_SLAM_SESSION = None
_SLAM_SUBSCRIPTION = None
_SLAM_REENTRY = None


def _slam_command(function):
    @functools.wraps(function)
    def call(*args, **kwargs):
        if active_algo() != "slam_toolbox":
            return function(*args, **kwargs)
        with _SLAM_COMMAND:
            return function(*args, **kwargs)
    return call


def _slam_session():
    global _SLAM_SESSION, _SLAM_SUBSCRIPTION
    if active_algo() != "slam_toolbox":
        raise ValueError("mapping checkpoints are supported only by slam_toolbox")
    with _SLAM_OPERATION:
        if _SLAM_SESSION is None:
            from .slam_checkpoints import MappingSession
            def capture(directory):
                node, ops = _get_node(), _engine()
                if node is None or ops is None:
                    raise RuntimeError("mapping engine unavailable")
                if lifecycle.current().get("mode") != "mapping":
                    raise RuntimeError("mapping is not active")
                metadata = {"frames": {"base": ops._base_frame, "odom": ops._odom_frame, "map": ops._map_frame},
                            "scan_topic": ops._scan_topic, "use_sim_time": ops._use_sim_time}
                from .slam_checkpoints import atomic_json
                metadata["sensor"] = _SLAM_SESSION.guard.sensor_metadata()
                atomic_json(Path(directory)/"sensor.json",metadata["sensor"])
                # Store the ACTUAL engine parameters, not just launch defaults.
                from rcl_interfaces.srv import GetParameters
                from rclpy.parameter import parameter_value_to_python
                names = ["resolution", "max_laser_range", "minimum_travel_distance", "minimum_travel_heading",
                         "scan_buffer_size", "loop_search_maximum_distance", "do_loop_closing",
                         "loop_match_minimum_response_fine", "loop_match_minimum_response_coarse",
                         "ceres_loss_function", "mode"]
                ok, response = engines._call_service(node, GetParameters, ops._srv("get_parameters"),
                    GetParameters.Request(names=names), 5.)
                if not ok:
                    raise RuntimeError(str(response))
                metadata["engine_parameters"] = dict(zip(names, [parameter_value_to_python(p) for p in response.values]))
                started = time.monotonic()
                metadata["serialize_started_at"] = time.time()
                ok, detail = ops.snapshot(node, str(directory), 60.)
                if not ok:
                    raise RuntimeError(detail)
                metadata["serialize_ms"] = (time.monotonic()-started)*1000
                metadata["serialize_completed_at"] = time.time()
                import importlib.metadata
                metadata["slam_toolbox_version"] = importlib.metadata.version("slam-toolbox")
                return metadata
            _SLAM_SESSION = MappingSession(
                os.environ.get("MAPPING_CHECKPOINT_DIR", str(Path(MAPS_DIR).parent/"slam-checkpoints")),
                capture, _SLAM_OPERATION,
                area_m2=float(os.environ.get("MAPPING_CHECKPOINT_AREA_M2", "20")),
                min_interval_s=float(os.environ.get("MAPPING_CHECKPOINT_MIN_INTERVAL_S", "15")),
                keep=int(os.environ.get("MAPPING_CHECKPOINT_KEEP", "20")),
                max_interval_s=float(os.environ.get("MAPPING_CHECKPOINT_MAX_INTERVAL_S", "60")))
        if _SLAM_SUBSCRIPTION is None and _get_node() is not None:
            from nav_msgs.msg import OccupancyGrid
            from rclpy.qos import QoSProfile, DurabilityPolicy
            _SLAM_SUBSCRIPTION = _get_node().create_subscription(OccupancyGrid, "/slam_toolbox/map",
                _SLAM_SESSION.observe, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            from .slam_guard import RuntimeGuard
            _SLAM_SESSION.guard = RuntimeGuard(_get_node(), _SLAM_SESSION, _engine())
            def withdraw(detail):
                log.error("mapping withdrawn for review: %s", detail)
                _engine().stop_engine()
                lifecycle.set_state("", "localization", bump=True)
            _SLAM_SESSION.on_suspect = withdraw
        return _SLAM_SESSION


def mapping_status_impl():
    if active_algo() != "slam_toolbox":
        return {"supported": False}
    return {"supported": True, **_slam_session().status()}


def checkpoint_impl():
    if active_algo() != "slam_toolbox":
        return {"ok": False, "detail": "checkpoints require slam_toolbox"}
    return _slam_session().request()


def list_checkpoints_impl():
    if active_algo() != "slam_toolbox":
        return {"ok": False, "detail": "checkpoints require slam_toolbox", "checkpoints": []}
    return {"ok": True, "checkpoints": _slam_session().store.list()}


def localization_map_directory():
    """A read-only runtime snapshot can differ from its immutable named base."""
    if active_algo()=="slam_toolbox" and _SLAM_REENTRY:
        return _SLAM_REENTRY["source"]
    map_id=lifecycle.current().get("map_id", "")
    return os.path.join(MAPS_DIR,map_id) if map_id else ""


def _saved_grid(map_dir):
    from types import SimpleNamespace as NS
    import numpy as np
    import yaml
    from PIL import Image
    directory = Path(map_dir)
    cfg = yaml.safe_load((directory/"occupancy.yaml").read_text())
    with Image.open(directory/cfg["image"]) as img:
        pixels = np.asarray(img.convert("L"))
    probability = pixels/255. if cfg.get("negate", 0) else (255.-pixels)/255.
    values = np.flipud(np.where(probability>cfg["occupied_thresh"], 100,
                               np.where(probability<cfg["free_thresh"], 0, -1))).astype(np.int8)
    x,y,yaw = cfg["origin"]
    return NS(header=NS(frame_id=_engine()._map_frame if active_algo()=="slam_toolbox" else os.environ.get("MAPPING_MAP_FRAME", "map")),
              data=values.ravel(), info=NS(width=values.shape[1], height=values.shape[0],
              resolution=cfg["resolution"], origin=NS(position=NS(x=x,y=y),
              orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))))


@_slam_command
def start_mapping_impl(base_map_id="", has_initial_pose=False, x=0., y=0., theta=0., *, _source=None, _parent=None):
    """Fresh and resumed mapping share the same executable and session owner."""
    global _SLAM_REENTRY
    if active_algo() != "slam_toolbox":
        return {"ok": False, "detail": "unified mapping requires slam_toolbox"}
    try:
        session = _slam_session()
        from .slam_checkpoints import graph_tool, inspect_graph
        if not os.access(graph_tool(), os.X_OK):
            return {"ok": False, "detail": "build the graph verifier with scripts/build_slam_graph_tool.sh first"}
        source = str(_source) if _source else ""
        if base_map_id:
            if base_map_id in {".", ".."} or _sanitize_map_id(base_map_id) != base_map_id:
                return {"ok": False, "detail": "invalid base map id"}
            source = source or os.path.join(MAPS_DIR, base_map_id)
        if has_initial_pose and not all(math.isfinite(float(v)) for v in (x,y,theta)):
            return {"ok": False, "detail": "initial pose must be finite"}
        # Validate BEFORE stopping a usable live mapping/localization session.
        if source:
            saved_engine = map_engine(source)
            if saved_engine and saved_engine != "slam_toolbox":
                return {"ok": False, "detail": "base map belongs to another engine"}
            import tempfile
            with tempfile.TemporaryDirectory(prefix="slam-validate-") as tmp:
                for name in engines.SlamToolboxOps.graph_files:
                    shutil.copy2(os.path.join(source,name), os.path.join(tmp,name))
                graph = inspect_graph(tmp)
                reference = [json.loads(s) for s in Path(tmp,"nodes.jsonl").read_text().splitlines()]
                graph["node_ids"] = [n["id"] for n in reference]
                graph["reference_nodes"] = {str(n["id"]): n["pose"] for n in reference}
            baseline = _saved_grid(source)
        else:
            graph, baseline = None, None
        _LOCALIZATION.cancel()
        session.suspend()
        node, ops = _get_node(), _engine()
        if node is None or ops is None:
            return {"ok": False, "detail": "ROS engine unavailable"}
        if source:
            _SLAM_REENTRY = {"source": source, "mode": "mapping", "graph": graph,
                             "baseline": baseline, "parent": _parent}
            lifecycle.set_state(base_map_id, "localization", bump=True)
            backend = _LocalizationBackend(node, ops, source, base_map_id, target_mode="mapping",
                graph_info=graph, baseline_grid=baseline, restored_from=_parent)
            seed = (float(x),float(y),float(theta)) if has_initial_pose else None
            run_id = _LOCALIZATION.start(backend, base_map_id, lifecycle.current()["generation"], seed)
            return {"ok": True, "detail": "verifying initial localization before continuing the saved graph",
                    "run_id": run_id, "mode": "localization", "relocalizing": True}
        with _SLAM_OPERATION:
            _SLAM_REENTRY = None
            ok, detail = ops._start_mapping_node()
            if not ok:
                return {"ok": False, "detail": detail}
            lifecycle.set_state("", "mapping", bump=True)
            session.begin(metadata={"generation": lifecycle.current()["generation"]})
        return {"ok": True, "detail": "new mapping session started; area checkpoints enabled"}
    except Exception as exc:
        log.exception("start mapping failed")
        return {"ok": False, "detail": str(exc)}


@_slam_command
def restore_checkpoint_impl(session_id, checkpoint_id):
    if active_algo() != "slam_toolbox":
        return {"ok": False, "detail": "checkpoints require slam_toolbox"}
    try:
        directory, manifest = _slam_session().store.validate(session_id, checkpoint_id)
        ops = _engine()
        frames = manifest["metadata"].get("frames", {})
        if frames and frames != {"base": ops._base_frame, "odom": ops._odom_frame, "map": ops._map_frame}:
            raise ValueError("checkpoint frame configuration differs from this deployment")
        # Copy the checkpoint into a private, immutable load artifact. A new
        # session can then prune its rolling history without removing this base.
        import tempfile
        source = tempfile.mkdtemp(prefix="slam-restore-", dir=RUNTIME_DB_DIR if os.path.isdir(RUNTIME_DB_DIR) else None)
        shutil.copytree(directory, source, dirs_exist_ok=True)
        _write_meta_fields(source, {"engine": "slam_toolbox"})
        return start_mapping_impl("", _source=source, _parent={"session_id": session_id, "checkpoint_id": checkpoint_id})
    except Exception as exc:
        return {"ok": False, "detail": str(exc)}


def _localize_current_slam_graph():
    """Preserve ALL current mapping work before changing to read-only mode."""
    global _SLAM_REENTRY
    try:
        import tempfile
        session = _slam_session()
        _LOCALIZATION.cancel()
        pose = live_pose(_get_node())
        if pose is None:
            return {"ok": False, "detail": "cannot capture the current mapping pose"}
        with _SLAM_OPERATION:
            saved = session.store.create(session.status()["session_id"], session.capture,
                metadata={**session._metadata,"base_map_id": lifecycle.current().get("map_id", "")},
                reason="switch_to_localization",before_commit=session.check_health)
            session.suspend()
            directory = session.store.path(saved["session_id"], saved["checkpoint_id"])
            source = tempfile.mkdtemp(prefix="slam-localization-")
            shutil.copytree(directory, source, dirs_exist_ok=True)
            _write_meta_fields(source, {"engine": "slam_toolbox"})
            _SLAM_REENTRY = {"source": source, "mode": "localization", "graph": saved["graph"],
                             "baseline": _saved_grid(source), "parent": None}
            # This graph includes NEW observations. It must not retain the
            # old library map id: navigation's grid relay would otherwise
            # select that old saved occupancy and hide the newly mapped area.
            lifecycle.set_state("", "localization", bump=True)
            started, detail = _begin_relocalization(_get_node(), _engine(), source,
                lifecycle.current().get("map_id", ""), pose)
        return {"ok": started, "detail": detail}
    except Exception as exc:
        return {"ok": False, "detail": str(exc)}


def _save_slam_map(map_id, map_dir, note):
    """Publish the same mutex-consistent graph used by automatic checkpoints."""
    from .slam_checkpoints import inspect_graph,validate_reference
    staging = ""
    try:
        with _SLAM_COMMAND, _SLAM_OPERATION:
            if os.path.exists(map_dir):
                return {"ok": False, "detail": "saved maps are immutable; choose a new map id"}
            staging = _staging_dir(map_id)
            if lifecycle.current().get("mode") == "mapping":
                captured = _slam_session().capture(staging)
                resolution = captured["engine_parameters"]["resolution"]
            else:
                source = (_SLAM_REENTRY["source"] if _SLAM_REENTRY else
                          os.path.join(MAPS_DIR, lifecycle.current().get("map_id", "")))
                for name in engines.SlamToolboxOps.graph_files:
                    shutil.copy2(os.path.join(source, name), os.path.join(staging, name))
                if os.path.isfile(os.path.join(source,"sensor.json")):
                    shutil.copy2(os.path.join(source,"sensor.json"),os.path.join(staging,"sensor.json"))
                import yaml
                with open(os.path.join(source,"occupancy.yaml")) as stream:
                    resolution = yaml.safe_load(stream)["resolution"]
            graph = inspect_graph(staging, resolution)
            graph.update(validate_reference(staging,_slam_session()._metadata if lifecycle.current().get("mode")=="mapping" else {}))
            _write_meta_fields(staging, {"map_id": map_id, "engine": "slam_toolbox", "note": note,
                                        "session_id": _slam_session().status()["session_id"], "nodes": graph["nodes"]})
            base_id = _slam_session().status().get("base_map_id")
            if isinstance(base_id, str) and base_id:
                source_dir = Path(MAPS_DIR)/_sanitize_map_id(base_id)
                preparation = source_dir/"local-update.json"
                history = source_dir/"local-update-history.json"
                if preparation.is_file():
                    (Path(staging)/"local-update-history.json").write_text(json.dumps({
                        "preparation": json.loads(preparation.read_text()),
                        "saved_from": base_id, "saved_at": time.time()}, indent=2))
                elif history.is_file():
                    shutil.copy2(history, Path(staging)/history.name)
            if lifecycle.current().get("mode") == "mapping":
                _slam_session().check_health()
            _atomic_publish_map_dir(staging, map_dir)
            staging = ""
            _mark_finalized()
            return {"ok": True, "map_id": map_id, "artifact_path": map_dir,
                    "detail": f"saved consistent graph ({graph['nodes']} nodes); mapping was not paused"}
    except Exception as exc:
        return {"ok": False, "detail": str(exc)}
    finally:
        if staging:
            shutil.rmtree(staging, ignore_errors=True)


# ── pose_estimate ─────────────────────────────────────────────────────────────
def pose_estimate_impl(x: float, y: float, theta: float,
                       cov_xy: float = 0.0, cov_theta: float = 0.0,
                       run_id: int | None = None) -> dict:
    """Publish a PoseWithCovarianceStamped to INITIALPOSE_TOPIC so rtabmap
    re-localizes from the given guess. Returns {ok, detail}."""
    try:
        seed = (float(x), float(y), float(theta))
    except (TypeError, ValueError):
        return {"ok": False, "detail": "pose coordinates must be numbers"}
    if not all(math.isfinite(value) for value in seed):
        return {"ok": False, "detail": "pose coordinates must be finite"}
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    # Only slam_toolbox stops its engine while the AMCL/Beluga fallback owns
    # localization. In that window there is no /initialpose subscriber, so an
    # operator choice has to join the checked handover. RTAB-Map keeps its
    # original /initialpose behaviour and never enters this candidate path.
    if active_algo() == "slam_toolbox":
        current = _LOCALIZATION.snapshot()
        live = lifecycle.current()
        if current["map_id"] != live.get("map_id") or current["generation"] != live.get("generation"):
            return {"ok": False, "detail": "map changed; refresh before submitting an initial pose"}
        if (current["phase"] == "failed" and run_id in (None, current["run_id"])) or (current["phase"] == "ready" and run_id is None):
            if current["map_id"] == live.get("map_id") and current["generation"] == live.get("generation"):
                if current["phase"] == "ready":
                    lifecycle.mark_reset("localization")
                started, detail = _begin_relocalization(
                    node, _engine(), os.path.join(MAPS_DIR, current["map_id"]), current["map_id"], seed)
                return {"ok": started, "detail": detail}
        ok, detail = _LOCALIZATION.submit(seed, run_id)
        return {"ok": ok, "detail": detail}
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


def get_pose_candidates_impl(timeout_s: float = 8.0) -> dict:
    """Read the current run's Top-5. This endpoint never starts a search.

    `timeout_s` remains for RPC compatibility; snapshots return immediately.
    """
    live = lifecycle.current()
    map_id = str(live.get("map_id") or "")
    generation = int(live.get("generation") or 0)
    empty = {"map_id": map_id, "generation": generation,
             "needs_selection": False, "candidates_json": "[]"}
    if active_algo() != "slam_toolbox":
        return {"ok": False,
                "detail": "pose candidates are supported only by the slam_toolbox engine",
                **empty}
    if live.get("mode") != "localization" or not map_id:
        return {"ok": False, "detail": "load a saved map in localization mode first", **empty}
    snapshot = _LOCALIZATION.snapshot()
    same_run = (snapshot["map_id"] == map_id and
                snapshot["generation"] == generation)
    candidates = snapshot["candidates"] if same_run and snapshot["phase"] == "choosing" else []
    return {"ok": True, "detail": snapshot["detail"],
            "map_id": map_id, "generation": generation,
            "needs_selection": bool(candidates), "candidates_json": json.dumps(candidates, allow_nan=False)}


# ── list_maps ─────────────────────────────────────────────────────────────────
def list_maps_impl() -> dict:
    """List saved maps under MAPS_DIR.

    Returns {ok, detail, maps_json}. The JSON payload mirrors the mapping Web
    UI library rows but is exposed through the standard map capability surface
    so consumers such as scene never depend on debug HTTP or shared volumes.
    """
    maps = []
    running = active_algo()
    try:
        if not os.path.isdir(MAPS_DIR):
            return {"ok": True, "detail": "", "maps_json": "[]"}
        for name in sorted(os.listdir(MAPS_DIR)):
            d = os.path.join(MAPS_DIR, name)
            if not os.path.isdir(d):
                continue
            db = os.path.join(d, "rtabmap.db")
            meta = {}
            mp = os.path.join(d, "meta.yaml")
            if os.path.isfile(mp):
                try:
                    with open(mp, "r", encoding="utf-8") as fh:
                        for line in fh:
                            if ":" in line:
                                k, v = line.split(":", 1)
                                meta[k.strip()] = v.strip()
                except Exception:  # noqa: BLE001
                    pass
            # Which engine wrote this map decides which files are its graph.
            # Row shape stays the same for every engine so the web UI and the
            # capability consumers do not branch on it; only `engine` and the
            # artifact path differ.
            engine_name = meta.get("engine") or ("rtabmap" if os.path.isfile(db) else running)
            ops = engines.engine_for(engine_name)
            graph = [os.path.join(d, f) for f in (ops.graph_files if ops else ("rtabmap.db",))]
            has_artifact = all(os.path.isfile(f) for f in graph)
            if not has_artifact:
                artifact_ok, artifact_detail = False, "missing spatial artifact"
            elif ops is not None:
                artifact_ok, artifact_detail = ops.graph_ready(d)
            else:
                artifact_ok, artifact_detail = _rtabmap_graph_ready(d)
            if has_artifact and engine_name != running:
                artifact_detail = (f"built by {engine_name}; this deployment runs {running}"
                                   + (f" ({artifact_detail})" if artifact_detail else ""))
            preview = os.path.join(d, "occupancy.png")
            maps.append({
                "map_id": name,
                "engine": engine_name,
                "loadable_here": bool(artifact_ok) and engine_name == running,
                "has_spatial_artifact": has_artifact,
                "spatial_ok": bool(artifact_ok),
                "artifact_detail": artifact_detail,
                "has_preview": os.path.isfile(preview),
                "artifact_path": graph[0] if has_artifact else "",
                "preview_path": preview if os.path.isfile(preview) else "",
                "artifact_size": sum(os.path.getsize(f) for f in graph) if has_artifact else 0,
                "updated": int(max(os.path.getmtime(f) for f in graph)) if has_artifact else 0,
                "meta": meta,
            })
        return {"ok": True, "detail": "", "maps_json": json.dumps(maps, ensure_ascii=False)}
    except Exception as e:  # noqa: BLE001
        log.exception("list_maps failed")
        return {"ok": False, "detail": str(e), "maps_json": "[]"}


# ── load_map ──────────────────────────────────────────────────────────────────
def _runtime_db_copy(saved_db: str, map_id: str) -> str:
    """Copy an immutable saved DB to a runtime DB used by RTAB-Map.

    Loading RTAB-Map directly on /mapping/maps/<map_id>/rtabmap.db makes the
    supposedly saved artifact mutable again. Use a runtime copy instead; the
    saved map remains a read-only artifact for Robonix semantics. The runtime
    copy must be loaded with LoadDatabase.clear=false; clear=true deletes an
    existing target DB before opening it in RTAB-Map.
    """
    os.makedirs(RUNTIME_DB_DIR, exist_ok=True)
    safe_id = _sanitize_map_id(map_id)
    for old in glob.glob(os.path.join(RUNTIME_DB_DIR, f"{safe_id}-*.db")):
        try:
            os.remove(old)
        except OSError:
            pass
    runtime_db = os.path.join(RUNTIME_DB_DIR, f"{safe_id}-{int(time.time() * 1000)}.db")
    shutil.copy2(saved_db, runtime_db)
    db_ok, db_detail = _sqlite_quick_check(runtime_db)
    if not db_ok:
        raise RuntimeError(f"runtime db copy failed integrity check: {db_detail}")
    return runtime_db


def _publish_full_map(node, timeout_s: float = 30.0) -> tuple[bool, str]:
    """Ask RTAB-Map to republish the global optimized map after load."""
    try:
        from rtabmap_msgs.srv import PublishMap
    except Exception as e:  # noqa: BLE001
        return False, f"rtabmap_msgs/PublishMap unavailable: {e}"
    req = PublishMap.Request()
    req.global_map = True
    req.optimized = True
    req.graph_only = False
    ok, res = _call_service(node, PublishMap, f"{RTABMAP_NS}/publish_map", req, timeout_s=timeout_s)
    if not ok:
        return False, str(res)
    return True, "published optimized global map"


def _load_database(node, runtime_db: str, timeout_s: float) -> tuple[bool, str]:
    """Load one runtime database, isolated for ordering tests."""
    try:
        from rtabmap_msgs.srv import LoadDatabase
    except Exception as e:  # noqa: BLE001
        return False, f"rtabmap_msgs/LoadDatabase unavailable: {e}"
    req = LoadDatabase.Request()
    req.database_path = runtime_db
    req.clear = False
    ok, res = _call_service(
        node, LoadDatabase, f"{RTABMAP_NS}/load_database", req, timeout_s=timeout_s
    )
    return (ok, "load_database completed" if ok else str(res))


def _occupancy_sample_ready(msg) -> tuple[bool, str]:
    """Return whether an occupancy sample is non-empty and structurally valid.

    Map identity comes from the successful database load plus the lifecycle
    epoch. RTAB-Map may optimize cell boundaries when it republishes a loaded
    database, so occupancy pixels are readiness data, not an identity hash.
    """
    info = msg.info
    width = int(info.width)
    height = int(info.height)
    resolution = float(info.resolution)
    cells = width * height
    data = msg.data
    known = sum(1 for value in data if int(value) >= 0)
    summary = (
        f"{width}x{height}@{resolution:.6f} "
        f"origin=({info.origin.position.x:.3f},{info.origin.position.y:.3f}) "
        f"cells={len(data)} known={known}"
    )
    ready = (
        width > 0
        and height > 0
        and resolution > 0.0
        and len(data) == cells
        and known > 0
    )
    return ready, summary


def _confirm_engine_pose(node, grid) -> tuple[bool, str]:
    """Check the pose the ENGINE now reports, not the one it was handed.

    Between being told where it is and settling there, a scan matcher can move;
    the only pose worth reporting to a consumer is the one it will actually
    publish.
    """
    pose = live_pose(node)
    if pose is None:
        return False, "the engine published no pose to check"
    fit, fit_detail = localizers.scan_fit(node, grid, pose)
    if fit < 0.0:
        return False, f"engine pose could not be verified: {fit_detail}"
    if fit < localizers.SCAN_FIT_MIN:
        return False, (f"it reports ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}) where the "
                       f"laser scores only {fit:.0%}")
    return True, f"engine settled at ({pose[0]:.2f}, {pose[1]:.2f}), laser {fit:.0%}"


def live_pose(node, timeout_s: float = 5.0):
    """The engine's own current pose in the map frame, from tf."""
    try:
        import tf2_ros
        from rclpy.duration import Duration
        from rclpy.time import Time
    except Exception:  # noqa: BLE001
        return None
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf, node, spin_thread=False)
    deadline = time.monotonic() + timeout_s
    base = os.environ.get("MAPPING_BASE_FRAME", "base_link")
    frame = os.environ.get("MAPPING_MAP_FRAME", "map")
    while time.monotonic() < deadline:
        try:
            tf = buf.lookup_transform(frame, base, Time(), Duration(seconds=0.2))
        except Exception:  # noqa: BLE001
            time.sleep(0.2)
            continue
        finally:
            pass
        t, q = tf.transform.translation, tf.transform.rotation
        del listener
        return (t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    del listener
    return None


def _confirm_against_map(node, grid, pose, require_scan: bool = False) -> tuple[bool, str]:
    """Second opinion on a recovered pose, from the laser the robot sees now."""
    fit, detail = localizers.scan_fit(node, grid, pose)
    if fit < 0.0:
        if require_scan:
            return False, f"laser verification unavailable: {detail}"
        return True, f"scan check skipped: {detail}"
    if fit < localizers.SCAN_FIT_MIN:
        return False, (f"the laser does not support that pose ({fit:.0%}, "
                       f"{detail}) — it is not where the map says")
    return True, f"confirmed by the laser ({fit:.0%})"


class _LocalizationBackend:
    """ROS operations for LocalizationFlow; no UI/session state lives here."""
    def __init__(self, node, ops, map_dir, map_id, *, target_mode="localization", graph_info=None,
                 baseline_grid=None, restored_from=None):
        self.node, self.ops = node, ops
        self.map_dir, self.map_id = map_dir, map_id
        self.grid = None
        self.engine_started = False
        self.target_mode, self.graph_info = target_mode, graph_info
        self.baseline_grid, self.restored_from = baseline_grid, restored_from
        self.odom_anchor = None
        self.recovered_odom = None
        self.filter_message = None

    def prepare(self, run):
        with _SLAM_OPERATION:
            ok, detail = self.ops.stop_engine()
        if not ok:
            raise RuntimeError(detail)
        run.check()
        from .grid_edits import effective_directory
        # Pin occupancy inputs for this initialization run. The original map_dir
        # remains the source of the SLAM graph during handover.
        self.grid_dir = effective_directory(self.map_dir)
        tracking_source = self.map_dir
        if self.target_mode == "mapping" and active_algo() == "slam_toolbox":
            # Old exports may have clipped Karto's outermost endpoint cells.
            # Rebuild a PRIVATE view of the immutable graph for initialization;
            # retain explicit grid edits as the user's localization surface.
            import tempfile
            from .slam_checkpoints import inspect_graph
            self.reentry_grid = tempfile.TemporaryDirectory(prefix="slam-reentry-grid-")
            tracking_source = self.reentry_grid.name
            for name in engines.SlamToolboxOps.graph_files:
                shutil.copy2(os.path.join(self.map_dir,name),os.path.join(tracking_source,name))
            inspect_graph(tracking_source)
            if self.grid_dir == self.map_dir:
                self.grid_dir = tracking_source
        self.grid = localizers.load_grid(self.grid_dir)
        self.tracking_grid = (self.grid if self.grid_dir == tracking_source
                              else localizers.load_grid(tracking_source))
        if self.grid is None or self.tracking_grid is None:
            raise RuntimeError("saved occupancy grid unavailable")
        if self.target_mode == "mapping":
            from collections import OrderedDict, deque
            import tf2_ros
            from rclpy.duration import Duration
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from sensor_msgs.msg import LaserScan
            from rclpy.qos import qos_profile_sensor_data
            self.odom_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=60))
            self.odom_listener = tf2_ros.TransformListener(self.odom_buffer, self.node, spin_thread=False)
            self.first_poses = deque(maxlen=20)
            self.scans = OrderedDict()
            def scan_sample(msg):
                self.scans[(msg.header.stamp.sec,msg.header.stamp.nanosec)] = msg
                while len(self.scans)>300:
                    self.scans.popitem(last=False)
            self.scan_subscription = self.node.create_subscription(LaserScan, self.ops._scan_topic,
                scan_sample, qos_profile_sensor_data)
            self.pose_subscription = self.node.create_subscription(PoseWithCovarianceStamped, "/pose", self.first_poses.append, 20)
            self.filter_subscription = self.node.create_subscription(PoseWithCovarianceStamped,
                localizers.POSE_TOPIC, lambda msg: setattr(self, "filter_message", msg), 20)

    def match(self, run):
        # A saved partial graph needs wider hypothesis retention. Do not alter
        # the mature ordinary-localization or any RTAB path.
        search = ({"fine_candidate_budget": 256}
                  if self.target_mode == "mapping" and active_algo() == "slam_toolbox" else {})
        result = localizers.match_statically(self.node, self.grid, **search)
        if self.target_mode == "mapping" and result[0] is not None and hasattr(result[0], "scan_stamp"):
            from rclpy.time import Time
            from rclpy.duration import Duration
            from .slam_motion import tuple_from_transform
            self.recovered_odom = tuple_from_transform(self.odom_buffer.lookup_transform(
                self.ops._odom_frame,self.ops._base_frame,Time.from_msg(result[0].scan_stamp),Duration(seconds=.5)))
        return result

    @staticmethod
    def conclusive(match):
        return (match.score >= localizers.STATIC_MATCH_MIN and
                match.score - match.runner_up >= localizers.STATIC_MATCH_MARGIN)

    def filter(self, run):
        if run.signal.is_set():
            return None, "initial pose supplied"
        ok, detail = localizers.start(self.grid_dir, self.map_id, owns_tf=False, map_topic="/map")
        if not ok:
            return None, f"Particle filter could not start: {detail}"
        try:
            run.check()
            ready, detail = localizers.wait_ready(self.node)
            run.check()
            if run.signal.is_set():
                return None, "initial pose supplied"
            if not ready:
                return None, f"Particle filter is unavailable: {detail}"
            ok, detail = localizers.global_localize(self.node)
            if not ok:
                return None, f"Global particle initialization failed: {detail}"
            _LOCALIZATION.filter_running(run)
            timeout = float(os.environ.get("MAPPING_RELOCALIZE_TIMEOUT_S", "180"))
            ok, pose, detail = localizers.wait_for_convergence(
                self.node, timeout, map_dir=self.grid_dir, interrupt_event=run.signal)
            run.check()
            if not ok:
                return None, detail
            if self.target_mode == "mapping" and self.filter_message is not None:
                from rclpy.time import Time
                from rclpy.duration import Duration
                from .slam_motion import tuple_from_transform
                message = self.filter_message
                self.recovered_odom = tuple_from_transform(self.odom_buffer.lookup_transform(
                    self.ops._odom_frame,self.ops._base_frame,Time.from_msg(message.header.stamp),Duration(seconds=.5)))
                p,q=message.pose.pose.position,message.pose.pose.orientation
                pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                return pose, f"Recovered by {localizers.name()}: {detail}"
            return localizers.current_pose(self.node) or pose, f"Recovered by {localizers.name()}: {detail}"
        finally:
            localizers.stop()

    @staticmethod
    def stop_filter():
        localizers.stop()

    def verify(self, run, pose):
        if self.target_mode == "mapping":
            from .slam_motion import advance, tuple_from_transform
            from rclpy.time import Time
            from rclpy.duration import Duration
            # Require three independent live scans. Transport the seed using
            # odom so manual movement during verification does not stale it.
            deadline = time.monotonic()+5.
            while not self.odom_buffer.can_transform(self.ops._odom_frame, self.ops._base_frame, Time()):
                run.check()
                if time.monotonic() >= deadline:
                    return False, "odometry TF unavailable for mapping initialization"
                time.sleep(.02)
            self.odom_anchor = (self.recovered_odom if run.seed is None and self.recovered_odom is not None else
                tuple_from_transform(self.odom_buffer.lookup_transform(self.ops._odom_frame, self.ops._base_frame, Time())))
            for _ in range(3):
                run.check()
                scan = localizers._next_scan(self.node, 6.)
                if scan is None:
                    return False, "no live laser for mapping initialization"
                odom = tuple_from_transform(self.odom_buffer.lookup_transform(
                    self.ops._odom_frame, self.ops._base_frame, Time.from_msg(scan.header.stamp), Duration(seconds=.5)))
                current = advance(pose, self.odom_anchor, odom)
                from .slam_regions import retained_start
                reason = retained_start(self.map_dir, current, self.tracking_grid)
                if reason:
                    return False, reason
                offset = localizers.sensor_offset_xy(self.node, scan.header.frame_id)
                for grid in (self.grid, self.tracking_grid):
                    fit, detail = localizers.scan_fit_of(grid, scan, current, offset)
                    if fit < localizers.SCAN_FIT_MIN:
                        return False, f"initial pose rejected by live laser ({fit:.0%}): {detail}"
            self.handover_seed, self.odom_anchor = current, odom
            return True, "three live scans verified; odometry anchored for moving handover"
        return _confirm_against_map(self.node, self.grid, pose, require_scan=True)

    def handover(self, run, pose):
        self.engine_started = True
        with _SLAM_OPERATION:
            if self.target_mode == "mapping":
                pose = self.handover_seed
                ok, detail = self.ops._start_mapping_node(self.map_dir, pose, self.odom_anchor)
            else:
                ok, detail = self.ops.switch_mode(self.node, "localization", self.map_dir, self.map_id, 30.0, pose)
        run.check()
        if not ok:
            return False, detail
        ok, ready = self.ops.wait_ready(self.node, 90.0)
        run.check()
        if not ok:
            return False, ready
        if self.target_mode == "mapping":
            ok, verified = self._verify_mapping_handover(run, pose)
        else:
            if run.cancelled.wait(3.0):
                run.check()
            ok, verified = _confirm_engine_pose(self.node, self.tracking_grid)
        if ok and self.target_mode == "mapping":
            import tempfile
            from .slam_checkpoints import inspect_graph,validate_reference
            with _SLAM_OPERATION, tempfile.TemporaryDirectory(prefix="slam-handover-") as tmp:
                run.check()
                saved, why = self.ops.snapshot(self.node, tmp, 30.)
                if not saved:
                    return False, why
                graph = inspect_graph(tmp)
                graph.update(validate_reference(tmp,{"reference_nodes":self.graph_info.get("reference_nodes",{}) if self.graph_info else {}}))
                ids = {json.loads(s)["id"] for s in Path(tmp,"nodes.jsonl").read_text().splitlines()}
                if self.graph_info and not set(self.graph_info.get("node_ids", [])).issubset(ids):
                    return False, "loaded graph is missing baseline nodes"
                run.check()
                from std_srvs.srv import Trigger
                released,response = engines._call_service(self.node,Trigger,"/slam_scan_gate/release_frame",Trigger.Request(),5.)
                if not released or not response.success:
                    return False, f"verified map frame could not be released: {response}"
                lifecycle.set_mode("mapping")
                _slam_session().begin(base_map_id=self.map_id, baseline_grid=self.baseline_grid,
                    parent=self.restored_from, metadata={"generation": lifecycle.current()["generation"],
                    "baseline_nodes": self.graph_info["nodes"] if self.graph_info else 0,
                    "reference_nodes": self.graph_info.get("reference_nodes", {}) if self.graph_info else {},
                    "guard_anchor": self.guard_anchor})
                # Every continued session starts with a verifiable checkpoint,
                # even when the robot remains within the old known area.
                _slam_session().request("session_start")
                verified += f"; {graph['nodes']} nodes, old graph retained"
        return ok, f"{detail}; {verified}"

    def _verify_mapping_handover(self, run, seed):
        from .slam_motion import advance, tuple_from_transform
        from rclpy.time import Time
        from rclpy.duration import Duration
        deadline = time.monotonic()+20.
        while not self.first_poses:
            run.check()
            from rcl_interfaces.srv import GetParameters
            ok, response = engines._call_service(self.node,GetParameters,"/slam_scan_gate/get_parameters",
                GetParameters.Request(names=["bootstrap_state"]),1.)
            if ok and response.values and response.values[0].string_value.startswith("failed:"):
                return False,response.values[0].string_value
            if time.monotonic() >= deadline:
                return False, "mapping engine did not produce an initial pose"
            time.sleep(.02)
        # A stationary robot legitimately adds just one node. Do not require
        # motion merely to obtain three native outputs: three input scans were
        # already verified before loading. Check any immediately available
        # native corrections, including the mandatory first one.
        run.cancelled.wait(.2)
        run.check()
        # Verify the INITIAL scan timestamps, rather than the robot's later
        # position after it may already have entered previously unknown space.
        fits = []
        for number, message in enumerate(list(self.first_poses)[:3]):
            scan = self.scans.get((message.header.stamp.sec,message.header.stamp.nanosec))
            if scan is None:
                return False, "initial mapping scan was not retained for verification"
            odom = tuple_from_transform(self.odom_buffer.lookup_transform(self.ops._odom_frame,
                self.ops._base_frame, Time.from_msg(message.header.stamp), Duration(seconds=.5)))
            expected = advance(seed,self.odom_anchor,odom)
            p,q = message.pose.pose.position,message.pose.pose.orientation
            actual = (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            distance = math.hypot(actual[0]-expected[0],actual[1]-expected[1])
            angle = abs(math.atan2(math.sin(actual[2]-expected[2]),math.cos(actual[2]-expected[2])))
            travel = math.hypot(expected[0]-seed[0],expected[1]-seed[1])
            if number == 0 and travel>1.:
                return False, "robot moved more than 1 m before graph handover; initialize closer to a known area"
            if distance>.1 or angle>math.radians(3):
                return False, f"mapping handover departed from verified odometry ({distance:.2f} m, {math.degrees(angle):.1f} deg)"
            fit, detail = localizers.scan_fit_of(self.tracking_grid,scan,actual,
                localizers.sensor_offset_xy(self.node,scan.header.frame_id))
            fits.append(fit)
            # A new viewpoint can expose previously unseen surfaces. Initial
            # localization was already laser-verified on THREE scans; after
            # motion, use short-baseline odometry and graph identity to verify
            # transport, rather than demanding the old grid explain new space.
            if travel<.1 and fit<localizers.SCAN_FIT_MIN:
                return False, f"initial mapping scan rejected ({fit:.0%}): {detail}"
            self.guard_anchor = {"stamp": Time.from_msg(message.header.stamp).nanoseconds,
                                 "pose": actual, "odom": odom}
        return True, f"initial mapping poses match verified seed and odometry; old-grid laser fits {fits}"

    def cleanup(self, run, completed):
        localizers.stop()
        if self.engine_started and not completed:
            with _SLAM_OPERATION:
                self.ops.stop_engine()
                if self.target_mode == "mapping":
                    _slam_session().suspend()
                    lifecycle.set_mode("localization")
        listener = getattr(self, "odom_listener", None)
        if listener is not None:
            listener.unregister()
        for name in ("scan_subscription", "pose_subscription", "filter_subscription"):
            subscription = getattr(self, name, None)
            if subscription is not None:
                self.node.destroy_subscription(subscription)
        private_grid = getattr(self, "reentry_grid", None)
        if private_grid is not None:
            private_grid.cleanup()


def _begin_relocalization(node, ops, map_dir: str, map_id: str, seed) -> tuple[bool, str]:
    if active_algo() != "slam_toolbox":
        return False, "initial localization is only supported by slam_toolbox"
    if ops is None:
        return False, "no engine to hand a recovered pose to"
    live = lifecycle.current()
    target = _SLAM_REENTRY
    if target:
        _slam_session().suspend()
    backend = (_LocalizationBackend(node, ops, target["source"], map_id, target_mode=target["mode"],
                graph_info=target["graph"], baseline_grid=target["baseline"], restored_from=target["parent"])
               if target else _LocalizationBackend(node, ops, map_dir, map_id))
    run_id = _LOCALIZATION.start(backend,
                                map_id, int(live.get("generation") or 0), seed)
    return True, f"initial localization run {run_id} started; searching the saved map"


def _begin_target_map_wait(node) -> dict:
    """Wait for the latched occupancy published after a successful map load."""
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)

    ready = threading.Event()
    observed = {"summary": "no occupancy sample received"}

    def on_map(msg):
        try:
            valid, summary = _occupancy_sample_ready(msg)
            observed["summary"] = summary
            if valid:
                ready.set()
        except Exception as exc:  # noqa: BLE001
            observed["summary"] = f"invalid occupancy sample: {exc}"

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    topic = os.environ.get("MAPPING_OCCUPANCY_TOPIC", "/map")
    sub = node.create_subscription(OccupancyGrid, topic, on_map, qos)
    return {
        "event": ready,
        "observed": observed,
        "subscription": sub,
        "topic": topic,
    }


def _finish_target_map_wait(node, barrier: dict, timeout_s: float) -> tuple[bool, str]:
    try:
        matched = barrier["event"].wait(timeout=max(0.0, timeout_s))
    finally:
        node.destroy_subscription(barrier["subscription"])
    if not matched:
        return False, (
            f"fresh non-empty occupancy was not observed on {barrier['topic']} "
            f"within {timeout_s:.1f}s; observed={barrier['observed']['summary']}"
        )
    return True, (
        f"verified fresh occupancy on {barrier['topic']} "
        f"({barrier['observed']['summary']})"
    )


@_slam_command
def load_map_impl(map_id: str, mode: str = "localization",
                  has_initial_pose: bool = False,
                  x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> dict:
    """Load an immutable saved map through a runtime DB copy.

    Saved spatial maps are artifacts and must not be modified after save.
    Therefore this always loads a copied database and switches RTAB-Map to
    localization mode, then forces a full optimized map publish so /map reflects
    the saved artifact instead of the previous live mapping session.
    """
    global _SLAM_REENTRY
    if active_algo() == "slam_toolbox" and mode == "mapping":
        return start_mapping_impl(map_id, has_initial_pose, x, y, theta)
    map_id = _sanitize_map_id(map_id)
    requested_mode = (mode or "localization").strip().lower()
    if requested_mode not in ("localization", "mapping"):
        return {"ok": False, "detail": f"mode={requested_mode!r} invalid (localization|mapping)"}
    mode = "localization"
    map_dir = os.path.join(MAPS_DIR, map_id)
    if not os.path.isdir(map_dir):
        return {"ok": False, "detail": f"no saved map at {map_dir}"}
    # A saved map belongs to the engine that built it: the graph file and the
    # service that reads it are engine-specific, so a mismatch is refused here,
    # naming both sides, rather than failing obscurely inside the load.
    algo, saved_engine = active_algo(), map_engine(map_dir)
    if saved_engine and saved_engine != algo:
        return {"ok": False,
                "detail": f"map {map_id!r} was built by {saved_engine}, but this deployment "
                          f"runs {algo}; load it on a {saved_engine} deployment, or rebuild "
                          f"the map with {algo}"}
    ops = _engine()
    graph_ok, graph_detail = (ops.graph_ready(map_dir) if ops is not None
                              else _rtabmap_graph_ready(map_dir))
    if not graph_ok:
        return {"ok": False, "detail": f"saved {algo} map is not loadable: {graph_detail}"}
    db_path = os.path.join(map_dir, "rtabmap.db")
    source_path = (db_path if algo == "rtabmap" else
                   os.path.join(map_dir, engines.SLAM_TOOLBOX_STEM))

    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}

    try:
        if algo == "slam_toolbox":
            _LOCALIZATION.cancel()
            _slam_session().suspend()
            _SLAM_REENTRY = None
        started = time.monotonic()
        log.info("load_map[%s] stage=prepare source=%s requested_mode=%s", map_id,
                 source_path, requested_mode)
        # Particle-filter localization on the saved occupancy grid, when the
        # deployment asked for it: it is the path that can relocalize with no
        # prior pose, so `load_map` without a pose stops meaning "hope the scan
        # matcher converges from wherever the robot thinks it is".
        # RTAB-Map is deliberately not routed through here. It opens a saved
        # database in localization mode further down and relocalizes against it
        # itself; the staged flow below is for an engine whose own node cannot
        # localize without also editing the map.
        if algo == "slam_toolbox" and localizers.enabled():
            # Loading a map runs in three stages, and each one has exactly one
            # publisher of `/map` and of `map -> odom`.
            #
            #   1. the mapping node stops. Its transform is the session's
            #      answer, not the saved map's, and nobody should be acting on
            #      a pose no one has checked. The saved grid goes on /map so
            #      the operator still sees what they loaded.
            #   2. the particle filter recovers the robot on that grid, without
            #      publishing a transform, while the robot drives.
            #   3. slam_toolbox comes back as its localization node, started at
            #      the recovered pose. It holds only the saved graph, does not
            #      add to it, and owns /map and map -> odom from then on.
            #
            # The call returns after stage 1, because an operator who loads a
            # map wants to see the map rather than a minute of nothing.
            lifecycle.set_mode("localization")
            lifecycle.set_state(map_id, "localization", bump=False)
            # A saved map's frame is the artifact's own, not the previous live
            # session's: consumers holding map-frame coordinates must be told.
            lifecycle.mark_reset("localization")
            seed = (x, y, theta) if has_initial_pose else None
            started, detail_reloc = _begin_relocalization(node, ops, map_dir, map_id, seed)
            return {
                "ok": True,
                "detail": f"loaded {map_id}; {detail_reloc}",
                "map_id": map_id,
                "mode": "localization",
                "localizer": localizers.name(),
                "relocalizing": started,
            }
        if algo != "rtabmap":
            # Engine-owned restore: slam_toolbox deserializes its pose graph and
            # continues from it. The localizer branch above already returned when
            # a particle filter is configured, so this is the bare-engine path.
            # Without a given pose the robot is stood where the engine has it
            # now, not at the graph's origin: the node keeps adding the live
            # scans, and placed at the origin it drew them in the wrong place
            # and grew the saved map. Right after a save the two frames are
            # the same; for an older map this is still the better guess.
            pose = (x, y, theta) if has_initial_pose else live_pose(node)
            ok_e, detail_e = ops.activate(
                node, map_dir, map_id,
                float(os.environ.get("MAPPING_LOAD_DATABASE_TIMEOUT_S", "180")), pose)
            if not ok_e:
                return {"ok": False, "detail": f"{algo} failed to load {map_id!r}: {detail_e}"}
            lifecycle.set_mode(mode)
            lifecycle.set_state(map_id, mode, bump=False)
            return {"ok": True, "detail": detail_e, "map_id": map_id,
                    "mode": mode, "engine": algo}
        runtime_db = _runtime_db_copy(db_path, map_id)

        # RTAB-Map only restores the saved 2D occupancy grid when the database
        # is opened in localization mode. Loading first while still in mapping
        # mode omits that grid; a second load then appears to fix the UI/RViz.
        log.info("load_map[%s] stage=switch_mode target=localization", map_id)
        ok2, info2 = _set_mode(node, mode)
        if not ok2:
            return {"ok": False, "detail": f"failed to switch localization before load: {info2}"}
        # RTAB-Map is in localization from here on, whether or not the rest of
        # the load succeeds. Publish that before the database swap so a load
        # that fails half way cannot leave consumers reading the old mode.
        lifecycle.set_mode(mode)

        load_timeout_s = float(os.environ.get("MAPPING_LOAD_DATABASE_TIMEOUT_S", "180"))
        log.info("load_map[%s] stage=load_database runtime_db=%s timeout=%.1fs",
                 map_id, runtime_db, load_timeout_s)
        ok, load_detail = _load_database(node, runtime_db, load_timeout_s)
        if not ok:
            return {"ok": False,
                    "detail": f"load_database failed: {load_detail} after {load_timeout_s:.0f}s"}
        # The swap succeeded: runtime_db is the live database now. Record it
        # here rather than in the callers, so a save_map issued after a web UI
        # load snapshots this database and not the pre-load one.
        set_active_db(runtime_db)

        # The database load is the map-identity authority. Publish that
        # identity before waiting for occupancy so consumers can bind the
        # following grid to the correct map epoch without pixel comparison.
        lifecycle.set_state(map_id, mode, bump=(mode == "mapping"))

        publish_timeout_s = float(os.environ.get("MAPPING_PUBLISH_MAP_TIMEOUT_S", "45"))
        log.info("load_map[%s] stage=publish_map timeout=%.1fs", map_id, publish_timeout_s)
        pub_ok, pub_detail = _publish_full_map(node, timeout_s=publish_timeout_s)
        if not pub_ok:
            # The database swap already happened via _load_database above --
            # rtabmap is genuinely serving runtime_db now even though the
            # preview publish failed. Surface it so callers (atlas_bridges
            # _record_load_result) can sync active-db bookkeeping instead of
            # leaving save_map pointed at the stale pre-load database.
            return {"ok": False, "runtime_db_path": runtime_db,
                    "detail": f"loaded {map_id}, but full map publish failed: {pub_detail}"}
        # /map is transient-local. Subscribing after publish receives the
        # latest grid and avoids accepting the previous map's latched sample.
        barrier = _begin_target_map_wait(node)
        verify_timeout_s = float(os.environ.get("MAPPING_VERIFY_MAP_TIMEOUT_S", "30"))
        verified, verify_detail = _finish_target_map_wait(node, barrier, verify_timeout_s)
        if not verified:
            log.error("load_map[%s] stage=verify failed: %s", map_id, verify_detail)
            # Same reasoning as the publish-failure branch above: the
            # database itself is already switched, only the post-load
            # verification failed.
            return {"ok": False, "runtime_db_path": runtime_db,
                    "detail": f"loaded {map_id}, but {verify_detail}"}
        elapsed = time.monotonic() - started
        log.info("load_map[%s] stage=complete elapsed=%.3fs %s", map_id, elapsed,
                 verify_detail)

        seeded = ""
        if has_initial_pose:
            ps = pose_estimate_impl(x, y, theta)
            seeded = f"; {ps['detail']}"
        note = "" if requested_mode == "localization" else f"; requested {requested_mode} coerced to localization"
        return {"ok": True,
                "runtime_db_path": runtime_db,
                "detail": f"loaded immutable map {map_id} via runtime copy; {pub_detail}; "
                          f"{verify_detail}; elapsed={elapsed:.1f}s{seeded}{note}"}
    except Exception as e:  # noqa: BLE001
        log.exception("load_map failed")
        return {"ok": False, "detail": str(e)}


# ── runtime session state ─────────────────────────────────────────────────────
# Two facts describe a live SLAM session: which mode is in effect, and which
# database RTAB-Map currently holds open. The mode already has an owner —
# lifecycle, which broadcasts (map_id, mode, generation) to consumers — so
# get_mode reads it there rather than keeping a second copy that can drift.
# The live database path has no such owner, so it lives here.
#
# Both are updated by the impls in this module, never by their callers. The
# gRPC servicers, the MCP handlers and the web UI are therefore interchangeable
# entry points: whichever one an operator uses, the next save_map snapshots the
# database RTAB-Map is actually writing.
_active_db: str = ""
_finalized: bool = False


def _rtabmap_graph_ready(map_dir: str) -> tuple[bool, str]:
    """A saved RTAB-Map graph is one sqlite database that opens cleanly."""
    db = os.path.join(map_dir, "rtabmap.db")
    if not os.path.isfile(db):
        return False, f"missing rtabmap.db in {map_dir}"
    return _sqlite_quick_check(db)


def _register_engines() -> None:
    """Give the registry RTAB-Map's implementations (they live in this module,
    so they are injected rather than imported to keep the import one-way)."""
    engines.register(engines.RtabmapOps(
        graph_ready=_rtabmap_graph_ready,
        snapshot=lambda node, staging_dir, timeout_s: (False, "rtabmap snapshots via save_map_impl"),
        activate=lambda node, map_dir, map_id, timeout_s, pose=None: (False, "rtabmap activates via load_map_impl"),
        reset=lambda node, timeout_s: (False, "rtabmap resets via reset_map_impl"),
        set_mode=_set_mode,
    ))
    # The engines share this deployment's localizer: slam_toolbox hands it
    # map -> odom while the engine is frozen on a saved map.
    engines.bind_localizer(localizers.enabled, localizers.start, localizers.stop)


def read_meta(map_dir: str) -> dict[str, str]:
    """Parse `<map_dir>/meta.yaml` into a flat dict (missing file -> {})."""
    meta: dict[str, str] = {}
    path = os.path.join(map_dir, "meta.yaml")
    if not os.path.isfile(path):
        return meta
    try:
        with open(path, "r", encoding="utf-8") as fh:
            for line in fh:
                if ":" in line:
                    k, v = line.split(":", 1)
                    meta[k.strip()] = v.strip()
    except OSError:
        pass
    return meta


def map_engine(map_dir: str) -> str:
    """Engine recorded for a saved map. Maps written before the field existed
    hold an RTAB-Map database, so that is the honest default."""
    meta = read_meta(map_dir)
    recorded = (meta.get("engine") or "").strip().lower()
    if recorded:
        return recorded
    return "rtabmap" if os.path.isfile(os.path.join(map_dir, "rtabmap.db")) else ""


def _write_meta_fields(map_dir: str, fields: dict[str, str]) -> None:
    """Add or replace `fields` in `<map_dir>/meta.yaml`, keeping the rest."""
    path = os.path.join(map_dir, "meta.yaml")
    lines: list[str] = []
    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as fh:
                lines = [l for l in fh
                         if l.split(":", 1)[0].strip() not in fields]
        except OSError:
            lines = []
    try:
        with open(path, "w", encoding="utf-8") as fh:
            fh.writelines(lines)
            for k, v in fields.items():
                fh.write(f"{k}: {v}\n")
    except OSError as e:
        log.warning("could not record %s in %s: %s", list(fields), path, e)


def active_algo() -> str:
    """The engine this process is running, as `atlas_bridge` recorded it.

    `MAPPING_ALGO` is set in-process at init; `/tmp/mapping_algo` is the copy
    `start_engine.sh` reads, and is used here as the fallback so a map operation
    issued from a helper process still routes to the right engine.
    """
    algo = (os.environ.get("MAPPING_ALGO") or "").strip().lower()
    if not algo:
        try:
            algo = open("/tmp/mapping_algo", encoding="utf-8").read().strip().lower()
        except OSError:
            algo = ""
    return algo or "rtabmap"


def _engine():
    """Map operations for the running engine, or None when it has none."""
    return engines.engine_for(active_algo())


def set_active_db(path: str) -> None:
    """Record the database RTAB-Map now holds open and clear the finalized flag.

    A newly opened database has not been published under a map_id yet, so it is
    a candidate for the shutdown snapshot again. Called by atlas_bridge.init
    with the resolved startup database_path, and by load_map_impl once
    RTAB-Map has switched onto a runtime copy.
    """
    global _active_db, _finalized
    _active_db = path or ""
    _finalized = False


def get_active_db() -> str:
    """Path of the database RTAB-Map holds open; "" before init has run."""
    return _active_db


def _mark_finalized() -> None:
    """Record that the live database has been published under a map_id, so the
    shutdown snapshot does not dump the same session a second time."""
    global _finalized
    _finalized = True


def map_finalized() -> bool:
    """True once save_map has published the live database under a map_id, so
    shutdown does not dump the same session a second time."""
    return _finalized


def get_mode_impl() -> dict:
    """Return the SLAM mode in effect (read-only), as carried by the lifecycle
    broadcast — the same value consumers see. Returns {ok, mode, detail}; mode
    is "" with ok=False before init has seeded it."""
    mode = str(lifecycle.current().get("mode") or "")
    if not mode:
        return {"ok": False, "mode": "", "detail": "mode not initialized yet"}
    return {"ok": True, "mode": mode, "detail": ""}


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
    mode_timeout_s = float(os.environ.get("MAPPING_SET_MODE_TIMEOUT_S", "60"))
    ok, res = _call_service(node, Empty, f"{RTABMAP_NS}/{srv}", Empty.Request(), timeout_s=mode_timeout_s)
    return (ok, srv if ok else f"{res} after {mode_timeout_s:.0f}s")


def relocalize_impl() -> dict:
    """Find the robot again on the map it is already localized on.

    The case this exists for is a robot that was carried: its pose is now wrong
    and nothing about the running system knows that, because a scan matcher
    tracking frame to frame follows the robot into the wrong place quite
    happily. Re-running the same search the load runs settles it, and the robot
    does not have to be driven for it.
    """
    if active_algo() != "slam_toolbox":
        return {"ok": False, "detail": "initial localization is only supported by slam_toolbox"}
    live = lifecycle.current()
    map_id = live.get("map_id") or ""
    if (live.get("mode") or "") != "localization" or (not map_id and not _SLAM_REENTRY):
        return {"ok": False, "detail": "not localizing on a saved map; load one first"}
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    ops = engines.engine_for(active_algo())
    if ops is None:
        return {"ok": False, "detail": f"{active_algo()} cannot relocalize"}
    map_dir = os.path.join(MAPS_DIR, map_id)
    # The controller cancels the old run and serializes engine teardown before
    # the new search. HTTP never waits for a previous CPU search to finish.
    lifecycle.mark_reset("localization")
    started, detail = _begin_relocalization(node, ops, map_dir, map_id, None)
    return {"ok": started, "map_id": map_id, "relocalizing": started,
            "detail": detail}


@_slam_command
def switch_mode_impl(mode: str) -> dict:
    """Flip the running engine between mapping and localization on the map it
    already has — no map load, no restart. Returns {ok, detail}.

    How the flip is achieved is the engine's business: RTAB-Map has a service
    for it, slam_toolbox is frozen and hands `map -> odom` to the localizer.
    What every backend owes the caller is the same behaviour — in localization
    mode the loaded map does not change and something still reports where the
    robot is.
    """
    mode = (mode or "").strip().lower()
    if mode not in ("localization", "mapping"):
        return {"ok": False, "detail": f"mode={mode!r} invalid (localization|mapping)"}
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    algo = active_algo()
    if algo == "slam_toolbox":
        if mode == get_mode_impl().get("mode"):
            return {"ok": True, "detail": f"already in {mode} mode"}
        if mode == "mapping":
            current = _LOCALIZATION.snapshot()
            if current["phase"] != "ready":
                return {"ok": False, "detail": "finish initial localization before continuing mapping"}
            pose = live_pose(node)
            if pose is None:
                return {"ok": False, "detail": "no current localized pose"}
            source = _SLAM_REENTRY["source"] if _SLAM_REENTRY else None
            return start_mapping_impl(lifecycle.current().get("map_id", ""), True, *pose, _source=source)
        return _localize_current_slam_graph()
    ops = engines.engine_for(algo)
    if ops is None:
        return {"ok": False, "detail": f"{algo} does not implement map persistence"}
    live = lifecycle.current()
    map_id = live.get("map_id", "")
    map_dir = os.path.join(MAPS_DIR, map_id) if map_id else ""
    try:
        if mode == "mapping" and algo == "slam_toolbox":
            _LOCALIZATION.cancel()
        ok, detail = ops.switch_mode(node, mode, map_dir, map_id, 20.0)
        if not ok:
            return {"ok": False, "detail": detail}
        # Mode flip only — the live frame does not move, so no generation bump.
        lifecycle.set_mode(mode)
        return {"ok": True, "detail": f"switched to {mode} mode: {detail}"}
    except Exception as e:  # noqa: BLE001
        log.exception("switch_mode failed")
        return {"ok": False, "detail": str(e)}


# ── reset_map ─────────────────────────────────────────────────────────────────
@_slam_command
def reset_map_impl() -> dict:
    """Wipe the running rtabmap's map (working memory + live database) and
    restart SLAM from scratch — for when mapping has diverged and you want a
    clean rebuild without a full redeploy. Calls rtabmap's `/rtabmap/reset`
    (std_srvs/Empty).

    Caveat: rtabmap restarts with the robot's CURRENT pose as the new origin,
    so the rebuilt map's frame will NOT align with the pre-reset one (origin
    drift). Saved maps on disk are untouched. Returns {ok, detail}.
    """
    if active_algo() == "slam_toolbox":
        return start_mapping_impl()
    node = _get_node()
    if node is None:
        return {"ok": False, "detail": "rclpy node unavailable (ROS not running?)"}
    algo = active_algo()
    try:
        if algo == "slam_toolbox":
            _LOCALIZATION.cancel()
        if algo != "rtabmap":
            # Same contract for every engine: clear the live graph, bump the
            # frame epoch (the origin moves), stay in mapping mode.
            ops = _engine()
            if ops is None:
                return {"ok": False, "detail": f"engine {algo!r} cannot reset its map"}
            ok_e, detail_e = ops.reset(node, 10.0)
            if not ok_e:
                return {"ok": False, "detail": f"{algo} reset failed: {detail_e}"}
            lifecycle.mark_reset()
            lifecycle.set_mode("mapping")
            return {"ok": True, "detail": f"map cleared — rebuilding from current pose "
                                          f"(origin reset; new frame won't match the old map); {detail_e}"}
        from std_srvs.srv import Empty
        ok, res = _call_service(node, Empty, f"{RTABMAP_NS}/reset", Empty.Request(), timeout_s=10.0)
        if not ok:
            return {"ok": False, "detail": f"{res} — rtabmap /reset unavailable "
                                           "(fall back to restart with config)"}
        # The origin moved the moment the reset succeeded, so bump the frame
        # epoch here rather than after the mode switch below: consumers holding
        # map-frame coordinates must be told they are stale even if the rest of
        # this call fails. Same map_id, new origin.
        lifecycle.mark_reset()
        mode_ok, mode_detail = _set_mode(node, "mapping")
        if not mode_ok:
            return {"ok": False,
                    "detail": "map cleared and the frame epoch was bumped (stored "
                              "map-frame coordinates are stale), but rtabmap did not "
                              f"switch back to mapping mode: {mode_detail}"}
        # Reset resumes in mapping mode — broadcast that too.
        lifecycle.set_mode("mapping")
        return {"ok": True, "detail": "map cleared — rebuilding from current pose "
                                      "(origin reset; new frame won't match the old map); switched to mapping mode"}
    except Exception as e:  # noqa: BLE001
        log.exception("reset_map failed")
        return {"ok": False, "detail": str(e)}




def _set_rtabmap_paused(node, paused: bool, timeout_s: float = 10.0) -> tuple[bool, str]:
    """Pause/resume RTAB-Map processing around live database snapshots.

    RTAB-Map writes statistics and node data while mapping. A concurrent SQLite
    backup can otherwise make the RTAB-Map process abort with
    "database is locked". Treat pause failure as a hard save failure; an
    unchecked live snapshot is worse than refusing to save.
    """
    try:
        from std_srvs.srv import Empty
    except Exception as e:  # noqa: BLE001
        return False, f"std_srvs/Empty unavailable: {e}"
    service = f"{RTABMAP_NS}/{'pause' if paused else 'resume'}"
    ok, res = _call_service(node, Empty, service, Empty.Request(), timeout_s=timeout_s)
    if not ok:
        return False, str(res)
    return True, f"rtabmap {'paused' if paused else 'resumed'}"

def _flush_rtabmap_database(node, live_db: str, timeout_s: float = 180.0) -> tuple[bool, str, str]:
    """Ask RTAB-Map to serialize memory without switching databases.

    Do not use LoadDatabase(live_db, clear=false) as a save shortcut: that
    callback closes the current database, clears runtime state, and reloads the
    requested DB. In long Webots mapping sessions this can drop the live rtabmap
    node and leave only wrapper/viz processes alive. RTAB-Map provides a
    dedicated /backup service that saves memory, writes the 2D map cache, copies
    live_db to live_db + ".back", and reinitializes the same database.
    """
    try:
        from std_srvs.srv import Empty
    except Exception as e:  # noqa: BLE001
        return False, f"std_srvs/Empty unavailable: {e}", ""
    started_at = time.time()
    ok, res = _call_service(node, Empty, f"{RTABMAP_NS}/backup", Empty.Request(), timeout_s=timeout_s)
    if not ok:
        return False, str(res), ""
    back = f"{live_db}.back"
    if not os.path.isfile(back):
        return False, f"rtabmap backup completed but did not produce {back}", ""
    # RTAB-Map's backup service serializes working memory and writes a stable
    # sibling copy. Refuse stale copies so save_map cannot publish an old graph
    # with a fresh occupancy preview.
    if os.path.getmtime(back) + 1.0 < started_at:
        return False, f"rtabmap backup file is stale: {back}", ""
    ok2, detail2 = _sqlite_quick_check(back)
    if not ok2:
        return False, f"rtabmap backup integrity check failed: {detail2}", ""
    return True, "rtabmap backup completed; source=rtabmap backup artifact", back


# ── save_map ──────────────────────────────────────────────────────────────────
def _atomic_publish_map_dir(staging_dir: str, map_dir: str) -> None:
    """Publish a completed staged map directory without leaving half-saves.

    Directory replacement cannot be a single POSIX rename over a non-empty
    existing directory, so keep the old map beside it until the staged directory
    is in place. If publishing fails, restore the old map when possible.
    """
    previous_dir = f"{map_dir}.previous-{os.getpid()}-{int(time.time() * 1000)}"
    if os.path.exists(previous_dir):
        shutil.rmtree(previous_dir, ignore_errors=True)
    moved_previous = False
    try:
        if os.path.exists(map_dir):
            os.replace(map_dir, previous_dir)
            moved_previous = True
        os.replace(staging_dir, map_dir)
        if moved_previous:
            shutil.rmtree(previous_dir, ignore_errors=True)
    except Exception:
        if moved_previous and not os.path.exists(map_dir) and os.path.exists(previous_dir):
            os.replace(previous_dir, map_dir)
        raise


def _staging_dir(map_id: str) -> str:
    """Fresh, empty staging directory for one save. Hidden (leading dot) and
    pid/time stamped so a concurrent save cannot collide, and so `list_maps`
    never shows a half-written map."""
    path = os.path.join(MAPS_DIR, f".{map_id}.staging-{os.getpid()}-{int(time.time() * 1000)}")
    if os.path.exists(path):
        shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path, exist_ok=False)
    return path


def _run_preview_snapshot(map_dir: str) -> bool:
    """Write occupancy preview artifacts for the map library UI."""
    import subprocess
    candidates = [
        os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "scripts", "save_map.py")),
        os.path.join(PKG_HOST_DIR, "scripts", "save_map.py"),
        "/mapping/scripts/save_map.py",
    ]
    script = next((c for c in candidates if c and os.path.isfile(c)), "")
    if not script:
        log.warning("save_map.py not found in candidates %s", candidates)
        return False
    try:
        proc = subprocess.run(
            ["python3", script, "--out-dir", map_dir, "--timeout", "12"],
            check=False, timeout=40, text=True, capture_output=True,
        )
        occupancy_ok = os.path.isfile(os.path.join(map_dir, "occupancy.png"))
        if proc.returncode != 0 and not occupancy_ok:
            log.warning("save_map.py failed for %s rc=%s stdout=%s stderr=%s",
                        map_dir, proc.returncode, proc.stdout[-1000:], proc.stderr[-1000:])
            return False
        if proc.returncode != 0:
            log.info("save_map.py wrote occupancy preview for %s but optional artifacts were incomplete: %s",
                     map_dir, proc.stdout[-1000:])
        else:
            log.info("save_map.py wrote preview for %s: %s", map_dir, proc.stdout[-1000:])
        return occupancy_ok
    except Exception as e:  # noqa: BLE001
        log.warning("save_map.py failed for %s: %s", map_dir, e)
        return False


def _save_map_via_engine(map_id: str, map_dir: str, algo: str, note: str = "") -> dict:
    """Snapshot the live map for an engine other than RTAB-Map.

    Deliberately the same shape as the RTAB-Map path: refuse to overwrite a
    published map, stage into a temporary directory, ask the engine for its
    graph, render the occupancy/cloud preview with the same script, record the
    metadata (including which engine wrote it) and publish atomically. Consumers
    and the web UI see one directory layout regardless of engine.
    """
    if algo == "slam_toolbox":
        return _save_slam_map(map_id, map_dir, note)
    ops = engines.engine_for(algo)
    if ops is None:
        return {"ok": False, "map_id": map_id, "artifact_path": "",
                "detail": f"engine {algo!r} does not implement map persistence"}
    if os.path.isdir(map_dir) and ops.graph_ready(map_dir)[0]:
        return {"ok": False, "map_id": map_id, "artifact_path": map_dir,
                "detail": f"spatial map {map_id!r} already exists and is immutable; "
                          "update scene annotations/objects separately"}
    node = _get_node()
    if node is None:
        return {"ok": False, "map_id": map_id, "artifact_path": "",
                "detail": "rclpy node unavailable; cannot ask the engine to save"}
    staging_dir = ""
    try:
        staging_dir = _staging_dir(map_id)
        timeout_s = float(os.environ.get("MAPPING_SAVE_BACKUP_TIMEOUT_S", "180"))
        state = lifecycle.current()
        loaded = os.path.join(MAPS_DIR, state.get("map_id") or "")
        if (state.get("mode") == "localization" and state.get("map_id")
                and ops.graph_ready(loaded)[0]):
            # A localization node does not change the map it holds, so the
            # session is the loaded artifact: copy it, graph and preview alike.
            shutil.copytree(loaded, staging_dir, dirs_exist_ok=True)
            detail = f"copied the loaded map {state['map_id']!r}"
        else:
            # The graph and the grid must be one state. Taken from a running
            # engine they were not: the grid lags the graph and both kept
            # growing between the two captures, so a reload rendered a larger
            # map than the saved preview. Hold the engine until its grid stops
            # changing, take both, then let it map again.
            held, held_detail = ops.freeze(node, 10.0)
            if not held:
                log.warning("save_map[%s]: %s; saving from a moving engine", map_id, held_detail)
            try:
                ok, detail = ops.snapshot(node, staging_dir, timeout_s)
                if not ok:
                    return {"ok": False, "map_id": map_id, "artifact_path": "",
                            "detail": f"{algo} could not serialize its map: {detail}"}
                if not _run_preview_snapshot(staging_dir):
                    return {"ok": False, "map_id": map_id, "artifact_path": "",
                            "detail": "map preview/occupancy snapshot was not produced; "
                                      "refusing to publish an incomplete spatial artifact"}
            finally:
                if held:
                    ops.resume(node, 10.0)
        _write_meta_fields(staging_dir, {"map_id": map_id, "engine": algo,
                                         "note": note or "-"})
        if not os.path.isfile(os.path.join(staging_dir, "occupancy.png")):
            return {"ok": False, "map_id": map_id, "artifact_path": "",
                    "detail": "occupancy preview missing after snapshot"}
        _atomic_publish_map_dir(staging_dir, map_dir)
        staging_dir = ""
        _mark_finalized()
        return {"ok": True, "map_id": map_id, "artifact_path": map_dir,
                "detail": f"saved {algo} map: {detail}"}
    except Exception as e:  # noqa: BLE001
        log.exception("save_map[%s] failed for engine %s", map_id, algo)
        return {"ok": False, "map_id": map_id, "artifact_path": "", "detail": str(e)}
    finally:
        if staging_dir and os.path.isdir(staging_dir):
            shutil.rmtree(staging_dir, ignore_errors=True)


def save_map_impl(map_id: str, note: str = "") -> dict:
    """Snapshot the current SLAM map under {MAPS_DIR}/<map_id>/.

    A spatial map is immutable once published. User annotations and scene
    objects are saved by scene under the same map_id; this function owns only
    the RTAB-Map spatial artifact.
    """
    map_id = _sanitize_map_id(map_id)
    map_dir = os.path.join(MAPS_DIR, map_id)
    db_path = os.path.join(map_dir, "rtabmap.db")
    staging_dir = ""
    algo = active_algo()
    try:
        os.makedirs(MAPS_DIR, exist_ok=True)
        if algo != "rtabmap":
            # Engine-owned snapshot: same directory layout, same preview and
            # metadata, only the graph file(s) differ (see engines.py).
            return _save_map_via_engine(map_id, map_dir, algo, note)

        if os.path.isfile(db_path):
            db_ok, db_detail = _sqlite_quick_check(db_path)
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": db_path,
                "detail": f"spatial map {map_id!r} already exists and is immutable; update scene annotations/objects separately ({db_detail})",
            }

        # Resolution order: the database this module recorded when init or
        # load_map last opened one, then the historical fallbacks. The recorded
        # path is what makes a save work after a load — the runtime copy
        # load_map switched to has neither of the fallback names.
        live_db = ""
        tried = []
        for cand in (get_active_db(),
                     os.environ.get("RTABMAP_DATABASE_PATH", ""),
                     os.path.expanduser("~/.ros/rtabmap.db")):
            if not cand:
                continue
            tried.append(cand)
            if os.path.isfile(cand):
                live_db = cand
                break
        if not live_db:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": "no live rtabmap database found to snapshot (tried: "
                          + (", ".join(tried) or "no candidate paths") + ")",
            }

        node = _get_node()
        if node is None:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": "rclpy node unavailable; cannot ask RTAB-Map to save",
            }

        flush_ok, flush_detail, snapshot_src = _flush_rtabmap_database(
            node,
            live_db,
            float(os.environ.get("MAPPING_SAVE_BACKUP_TIMEOUT_S",
                                 os.environ.get("MAPPING_SAVE_FLUSH_TIMEOUT_S", "180"))),
        )
        if not flush_ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"rtabmap save/flush failed: {flush_detail}",
            }

        staging_dir = _staging_dir(map_id)
        staged_db = os.path.join(staging_dir, "rtabmap.db")

        ok, detail = _sqlite_backup(snapshot_src, staged_db)
        if not ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"failed to snapshot flushed RTAB-Map database: {detail}",
            }
        flush_detail = f"{flush_detail}; sqlite_backup={detail}"

        db_ok, db_detail = _sqlite_quick_check(staged_db)
        if not db_ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"staged database failed integrity check: {db_detail}",
            }

        pub_ok, pub_detail = _publish_full_map(
            node, timeout_s=float(os.environ.get("MAPPING_PUBLISH_MAP_TIMEOUT_S", "45"))
        )
        if not pub_ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"saved DB snapshot but RTAB-Map did not publish a complete map preview: {pub_detail}",
            }
        flush_detail = f"{flush_detail}; {pub_detail}"

        preview_ok = _run_preview_snapshot(staging_dir)
        # The engine that produced the graph is part of the artifact: a map
        # saved by one SLAM engine cannot be loaded by another (the graph files
        # and the services that read them differ), and load_map refuses rather
        # than failing obscurely later.
        _write_meta_fields(staging_dir, {"engine": active_algo()})
        meta_path = os.path.join(staging_dir, "meta.yaml")
        if os.path.isfile(meta_path):
            try:
                with open(meta_path, "r", encoding="utf-8") as fh:
                    lines = fh.readlines()
                with open(meta_path, "w", encoding="utf-8") as fh:
                    wrote = False
                    for line in lines:
                        if line.startswith("map_id:"):
                            fh.write(f"map_id: {map_id}\n")
                            wrote = True
                        else:
                            fh.write(line)
                    if not wrote:
                        fh.write(f"map_id: {map_id}\n")
            except Exception as e:  # noqa: BLE001
                log.warning("failed to normalize metadata map_id for %s: %s", staging_dir, e)
        if not preview_ok or not os.path.isfile(os.path.join(staging_dir, "occupancy.png")):
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": "map preview/occupancy snapshot was not produced; refusing to publish incomplete spatial artifact",
            }

        # Re-check after preview generation so the published directory is known
        # loadable at the exact point it becomes visible to list/load calls.
        db_ok, db_detail = _sqlite_quick_check(staged_db)
        if not db_ok:
            return {
                "ok": False,
                "map_id": map_id,
                "artifact_path": "",
                "detail": f"staged database failed final integrity check: {db_detail}",
            }

        _atomic_publish_map_dir(staging_dir, map_dir)
        staging_dir = ""
        _mark_finalized()
        return {
            "ok": True,
            "map_id": map_id,
            "artifact_path": db_path,
            "detail": f"saved spatial map {map_id}; {flush_detail}",
        }
    except Exception as e:  # noqa: BLE001
        log.exception("save_map failed for %s", map_id)
        return {"ok": False, "map_id": map_id, "artifact_path": "", "detail": str(e)}
    finally:
        if staging_dir and os.path.exists(staging_dir):
            shutil.rmtree(staging_dir, ignore_errors=True)


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


_register_engines()
