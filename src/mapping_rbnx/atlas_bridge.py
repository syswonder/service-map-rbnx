# SPDX-License-Identifier: MulanPSL-2.0
"""mapping_rbnx atlas bridge — robonix v0.1 Capability flow.

Responsibilities (kept tight; SLAM nodes do the actual work):
  1. Register `mapping` as a capability with atlas, declare a
     `service/map/driver` interface.
  2. Wait for `Driver(CMD_INIT, config_json)` from rbnx — config arrives
     ONLY through this gRPC channel (NEVER from disk / env). The cfg
     dict carries:
       - algo:    rtabmap | dlio | fastlio2[broken]
       - sensors: lidar2d / lidar3d / rgb / rgbd / imu / odom booleans
       - platform: x86_desktop / jetson_orin
  3. Resolve sensor primitives via atlas, write `/tmp/<algo>_resolved.yaml`
     for the launch file (start_engine.sh greps these out).
  4. Declare the algo-appropriate output endpoints on atlas, ALL under
     the SAME contract surface (robonix/service/map/*) so consumers
     (scene, nav) are algo-agnostic.

slam_toolbox + cartographer were removed in favour of rtabmap after
extensive head-to-head testing in webots: rtabmap handles curve-motion
mapping, loop closure, and depth fusion better than either alternative
for the cost of one extra dependency.
"""
from __future__ import annotations

import logging
import os
import sys
import time
from pathlib import Path
from typing import Optional

logging.basicConfig(level=logging.INFO,
                    format="[atlas_bridge] %(levelname)s %(message)s")
log = logging.getLogger("mapping_rbnx.atlas_bridge")


# ── Generated proto stubs ─────────────────────────────────────────────────────
# Codegen drops them under <pkg>/rbnx-build/codegen/proto_gen/. PYTHONPATH
# is set by docker entrypoint.sh.
import grpc  # noqa: E402

import atlas_pb2 as pb  # type: ignore
import atlas_pb2_grpc as pb_grpc  # type: ignore

from robonix_api import Service, Ok, Err, Deferred, ATLAS  # noqa: E402


# ── Config ────────────────────────────────────────────────────────────────────
ATLAS_ENDPOINT = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
CAP_ID = os.environ.get("ROBONIX_CAPABILITY_ID", "mapping")
NAMESPACE = "robonix/service/map"
PKG_HOST_DIR = os.environ.get("ROBONIX_PKG_HOST_DIR", "/mapping")
RESOLVED_DIR = os.environ.get("MAPPING_RESOLVED_DIR", "/tmp")
HEARTBEAT_PERIOD_S = 10.0

# Persistent map store. Each named map lives under {MAPS_DIR}/{map_id}/ and
# survives container restarts ONLY when the deploy bind-mounts a host dir
# here (docker/compose.yaml mounts it; see CAPABILITY.md "persistence").
# Without map_id the package stays ephemeral — exactly the pre-persistence
# behaviour (temp db, wiped each boot).
MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")


def _truthy(s: str) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "on")


# ── Map persistence (map_id / mapping vs localization) ────────────────────────
import re  # noqa: E402

_VALID_MODES = ("mapping", "localization")


def _sanitize_map_id(map_id: str) -> str:
    """Map ids become a directory name, so restrict to a filesystem-safe
    charset. Mirrors scene's ObjectStore sanitiser so a given map_id keys
    the SAME spatial map (mapping) and semantic map (scene) consistently."""
    return re.sub(r"[^A-Za-z0-9._-]", "_", map_id.strip()) or "default"


def _resolve_persistence(cfg: dict) -> dict[str, str]:
    """Turn the map_id / map_mode / reset_map config into the concrete
    keys the launch needs: database_path, map_mode, reset_map.

    Binding contract: the operator sets the SAME `map_id` here and in the
    scene service's config; mapping owns the spatial map (rtabmap db +
    map frame), scene keys its semantic objects to that id. A stable
    map frame across restarts requires map_mode=localization against a
    previously-saved db.

    Returns {} when no map_id is configured (ephemeral mode — keeps the
    legacy delete-db-on-start behaviour). Raises on an invalid combo so a
    misconfigured deploy fails loud instead of silently mapping into a
    throwaway db.
    """
    raw_id = cfg.get("map_id")
    if not raw_id:
        return {}  # ephemeral — no persistence
    map_id = _sanitize_map_id(str(raw_id))
    mode = str(cfg.get("map_mode", "mapping")).strip().lower()
    if mode not in _VALID_MODES:
        raise RuntimeError(
            f"map_mode={mode!r} invalid — must be one of {list(_VALID_MODES)}"
        )
    reset = _truthy(cfg.get("reset_map", False))
    map_dir = os.path.join(MAPS_DIR, map_id)
    db_path = os.path.join(map_dir, "rtabmap.db")
    if mode == "localization":
        if not os.path.isfile(db_path):
            raise RuntimeError(
                f"map_mode=localization but no saved map at {db_path!r}. "
                f"Run a mapping session with map_id={map_id!r} first "
                f"(map_mode=mapping), or check MAPPING_MAPS_DIR is mounted."
            )
        if reset:
            raise RuntimeError("reset_map=true is meaningless in localization mode")
    os.makedirs(map_dir, exist_ok=True)
    log.info("persistence: map_id=%s mode=%s db=%s reset=%s",
             map_id, mode, db_path, reset)
    return {
        "map_id": map_id,
        "map_mode": mode,
        "database_path": db_path,
        "reset_map": "true" if reset else "false",
    }


# ── Per-algo output endpoint map ──────────────────────────────────────────────
# Each algo publishes its outputs on different topic names; the bridge
# collapses them onto the SAME contract surface so consumers don't need
# to know which algo is running. Only declare a contract when the algo
# actually publishes it.
# The exported capability surface is FIXED regardless of algo — all
# algos must back the same set of contracts so consumers (scene, nav,
# etc.) only care about contracts, never about which algo is running.
# When an algo can't natively publish a contract, the launch file is
# expected to spawn an adapter node so the topic still exists at the
# bound name. Adding a new contract to this surface is a versioning
# event for the package — bump package.version and update every algo.
_EXPORTED_CONTRACTS: tuple[str, ...] = (
    "robonix/service/map/occupancy_grid",
    "robonix/service/map/pointcloud",
    # SLAM-corrected map-frame pose (loop-closed; can jump on closure).
    # Consumed by scene self-tracker / nav / explore so they don't fall
    # back to raw chassis odom.
    "robonix/service/map/pose",
    # SLAM-corrected continuous odom (frame: odom, smooth between loop
    # closures). For high-rate velocity controllers and trajectory
    # tracking that can't tolerate the jumps in /map/pose.
    "robonix/service/map/odom",
)

# Per-algo: which internal ROS topic backs each exported contract.
# Each entry MUST cover every contract in _EXPORTED_CONTRACTS — that
# invariant is asserted at startup so we fail loud on a misconfigured
# algo rather than silently exporting fewer caps.
_ALGO_TOPIC_BINDINGS: dict[str, dict[str, str]] = {
    "rtabmap": {
        # 2D OccupancyGrid on /map (lidar + depth fusion); 3D fused
        # cloud on /rtabmap/cloud_map (useful for scene's table/chair
        # occlusion layer).
        "robonix/service/map/occupancy_grid": "/map",
        "robonix/service/map/pointcloud":     "/rtabmap/cloud_map",
        # /robonix/map/pose: PoseWithCovarianceStamped published by our
        # tf_to_pose adapter (scripts/tf_to_pose.py, launched alongside
        # rtabmap in rtabmap_2d.launch.py). The adapter polls tf2 for
        # `map → base_link` at 10 Hz and republishes — necessary
        # because rtabmap does NOT publish /localization_pose in
        # mapping mode (only in localization mode after a db is loaded).
        # Earlier this contract was bound to /rtabmap/localization_pose,
        # which silently never produced messages, and consumers
        # (scene's self-tracker) fell back to raw chassis /odom.
        "robonix/service/map/pose":           "/robonix/map/pose",
        # /rtabmap/odom: continuous odometry. When external odom is
        # supplied, rtabmap republishes it under this name with the
        # same map→odom correction baked in via /tf. When icp_odometry
        # is running internally, this IS the odom source.
        "robonix/service/map/odom":           "/rtabmap/odom",
    },
    "dlio": {
        # Direct LiDAR-Inertial Odometry: real-robot 3D livox path.
        # No native 2D OccupancyGrid — projected from /dlio's cloud
        # by a pointcloud_to_grid adapter spawned in the launch.
        "robonix/service/map/occupancy_grid": "/robonix/map/occupancy_grid",
        "robonix/service/map/pointcloud":     "/dlio/odom_node/pointcloud/deskewed",
        # DLIO publishes a single Odometry topic (no separate
        # loop-closed pose stream). We bind both contracts to it; the
        # docstring on /map/pose notes "can jump on closure" but DLIO
        # has no loop closure so it never jumps anyway.
        "robonix/service/map/pose":           "/dlio/odom_node/pose",
        "robonix/service/map/odom":           "/dlio/odom_node/odom",
    },
    "fastlio2": {
        # [BROKEN: drift] same shape as dlio, kept for repro only.
        "robonix/service/map/occupancy_grid": "/robonix/map/occupancy_grid",
        "robonix/service/map/pointcloud":     "/fastlio2/world_cloud",
        "robonix/service/map/pose":           "/fastlio2/lio_odom",
        "robonix/service/map/odom":           "/fastlio2/lio_odom",
    },
}


def _check_binding_complete(algo: str) -> None:
    """Fail loud if an algo doesn't bind every exported contract."""
    bindings = _ALGO_TOPIC_BINDINGS.get(algo, {})
    missing = [c for c in _EXPORTED_CONTRACTS if c not in bindings]
    if missing:
        raise RuntimeError(
            f"algo={algo!r} missing topic binding for contracts: {missing}. "
            f"Every algo must back the full exported surface "
            f"{list(_EXPORTED_CONTRACTS)}; add an adapter node to the "
            f"launch file or update _ALGO_TOPIC_BINDINGS."
        )


# ── Sensor → contract resolution ──────────────────────────────────────────────
# Maps `sensors.*` config flags to the atlas contract IDs scene asks
# atlas to resolve. Each entry: (config_key, contract_id, role_in_lio_yaml).
_SENSOR_CONTRACTS = [
    # (config-key,   contract,                                  yaml-key)
    ("lidar3d",  "robonix/primitive/lidar/lidar3d",       "lidar_topic"),
    ("lidar2d",  "robonix/primitive/lidar/lidar",         "scan_topic"),
    ("imu",      "robonix/primitive/imu/imu",             "imu_topic"),
    ("rgbd",     "robonix/primitive/camera/depth",        "depth_topic"),
    ("rgb",      "robonix/primitive/camera/rgb",          "rgb_topic"),
    ("odom",     "robonix/primitive/chassis/odom",        "odom_topic"),
]


def _enabled_sensors(cfg: dict) -> dict:
    """Read config.sensors. The deploy manifest is authoritative —
    every robot has different sensors, so we refuse to guess.

    A missing or empty `sensors:` block is a configuration error: the
    operator forgot to declare what the robot has, and silently
    picking "lidar2d + rgbd" would mask Mid360 deploys (where the
    correct answer is lidar3d + rgbd) and headless deploys (where the
    correct answer is rgbd-only). Fail loud instead.
    """
    sensors = cfg.get("sensors")
    if not isinstance(sensors, dict) or not sensors:
        raise RuntimeError(
            "mapping config has no `sensors:` block. Declare which "
            "sensors the robot has, e.g.:\n"
            "  sensors: { lidar2d: true, rgbd: true, odom: true }     # webots tiago\n"
            "  sensors: { lidar3d: true, rgbd: true, odom: true, imu: true }  # mid360 robot\n"
            "Supported keys: " + ", ".join(k for k, _, _ in _SENSOR_CONTRACTS)
        )
    out = {}
    for key, _contract, _yaml in _SENSOR_CONTRACTS:
        out[key] = _truthy(sensors.get(key, False))
    if not any(out.values()):
        raise RuntimeError(
            "mapping config has `sensors:` block but every entry is "
            "false/missing — at least one sensor must be enabled."
        )
    return out


# ── Atlas helpers (use Capability's wrapped stub) ────────────────────────────
def _resolve_sensor_endpoint(cap: Service, contract_id: str) -> Optional[str]:
    """ATLAS.find_capability + connect_capability for one ROS2 contract. Returns the topic
    string atlas resolved, or None when no provider is online yet.
    The opened Channel is closed immediately — we just want the
    endpoint string, atlas's bookkeeping for "I'm consuming this"
    is the side benefit."""
    recs = ATLAS.find_capability(contract_id=contract_id, transport="ros2")
    if not recs:
        return None
    rec = recs[0]
    try:
        ch = mapping.connect_capability(rec, contract_id=contract_id, transport="ros2")
    except Exception as e:  # noqa: BLE001
        log.warning("connect %s/%s failed: %s", rec.provider_id, contract_id, e)
        return None
    endpoint = (ch.endpoint or "").strip()
    ch.close()
    return endpoint or None


def _resolve_sensors(cap: Service, cfg: dict) -> dict[str, str]:
    """For each enabled sensor, ask atlas for the topic. Empty when
    the primitive isn't online yet (resolved.yaml still useful — the
    launch picks a sane default)."""
    enabled = _enabled_sensors(cfg)
    resolved: dict[str, str] = {}
    for cfg_key, contract_id, yaml_key in _SENSOR_CONTRACTS:
        if not enabled.get(cfg_key):
            continue
        ep = _resolve_sensor_endpoint(cap, contract_id)
        if ep:
            resolved[yaml_key] = ep
            log.info("resolved %s → %s = %s", contract_id, yaml_key, ep)
        else:
            log.info("sensor %s (%s) not available on atlas yet", cfg_key, contract_id)
    return resolved


def _retry_resolve(cap: Service, cfg: dict, deadline_s: float = 30.0,
                   settle_s: float = 8.0) -> dict[str, str]:
    """Wait until at least one sensor lands, then absorb late arrivals
    for `settle_s` so all enabled inputs end up in resolved.yaml."""
    deadline = time.time() + deadline_s
    resolved: dict[str, str] = {}
    while time.time() < deadline:
        resolved = _resolve_sensors(cap, cfg)
        if resolved:
            break
        time.sleep(2.0)
    if not resolved:
        log.warning("no sensors discovered within %.1fs — launch will use defaults", deadline_s)
        return {}
    settle_until = time.time() + settle_s
    while time.time() < settle_until:
        time.sleep(2.0)
        more = _resolve_sensors(cap, cfg)
        if len(more) > len(resolved):
            log.info("sensor-resolve settle: %d → %d", len(resolved), len(more))
            resolved = more
    return resolved


def _declare_outputs(cap: Service, algo: str) -> None:
    """DeclareInterface(transport=ROS2) for every exported contract.
    All algos publish the same contract surface — only the topic differs."""
    bindings = _ALGO_TOPIC_BINDINGS[algo]
    declared = 0
    for contract_id in _EXPORTED_CONTRACTS:
        topic = bindings[contract_id]
        try:
            mapping.declare_ros2_topic(contract_id, topic, qos="reliable")
            declared += 1
            log.info("declared %s → ROS2 topic %s", contract_id, topic)
        except Exception as e:  # noqa: BLE001
            log.warning("declare %s failed: %s", contract_id, e)
    log.info("declared %d/%d output(s) for algo=%s",
             declared, len(_EXPORTED_CONTRACTS), algo)


def _write_resolved_yaml(algo: str, resolved: dict[str, str]) -> str:
    """Write /tmp/<algo>_resolved.yaml with k=v lines. start_engine.sh
    `grep`s these out — yaml-lite is fine, no parser needed."""
    out = os.path.join(RESOLVED_DIR, f"{algo}_resolved.yaml")
    # tf frames default for webots tiago. Real-robot deploys without a
    # base_link TF (no chassis driver, no soma URDF yet) override
    # base_frame to the lidar's own frame (e.g. livox_frame for MID-360)
    # via cfg in the deploy manifest. use_sim_time follows the same
    # cfg path so real-robot bring-ups don't get stuck waiting for a
    # /clock that never comes.
    defaults = {
        "base_frame": "base_link",
        "odom_frame": "odom",
        "map_frame": "map",
        "use_sim_time": "true",
    }
    merged = {**defaults, **resolved}
    with open(out, "w") as f:
        for k, v in merged.items():
            f.write(f"{k}: {v}\n")
    log.info("wrote resolved config → %s (%d keys)", out, len(merged))
    return out


# ── Capability + lifecycle ────────────────────────────────────────────────────
mapping = Service(id=CAP_ID, namespace=NAMESPACE)


@mapping.on_init
def init(cfg: dict):
    """REGISTERED → INITIALIZED. Receives the config dict from
    rbnx via Driver(CMD_INIT, config_json). The mapping cap NEVER reads
    config from disk / env — this gRPC channel is the only sanctioned
    delivery path.

    Order:
      1. Validate algo + sensors block.
      2. Persist algo for start_engine.sh (it greps /tmp/mapping_algo).
      3. Resolve enabled sensor topics from atlas (with retry+settle —
         primitives may still be warming up when CMD_INIT lands).
      4. Write /tmp/<algo>_resolved.yaml for the launch file.
      5. DeclareInterface(ROS2) for every exported map output.
    """
    algo = cfg.get("algo", "rtabmap")
    if algo not in _ALGO_TOPIC_BINDINGS:
        return Err(
            f"unknown algo {algo!r} — supported: {list(_ALGO_TOPIC_BINDINGS)}"
        )
    try:
        _check_binding_complete(algo)
    except RuntimeError as e:
        return Err(str(e))
    if algo == "fastlio2":
        log.warning("algo=fastlio2 is BROKEN (drift); use only for repro/debug")

    log.info("CMD_INIT: algo=%s atlas=%s cap=%s", algo, ATLAS_ENDPOINT, CAP_ID)

    # Persist algo for start_engine.sh.
    os.environ["MAPPING_ALGO"] = algo
    Path("/tmp/mapping_algo").write_text(algo)

    # Discover sensors → resolved.yaml. Raises if `sensors:` block missing.
    try:
        resolved = _retry_resolve(mapping, cfg)
    except RuntimeError as e:
        return Err(str(e))
    # TF / time-source overrides from cfg. Real-robot bring-ups without
    # a chassis driver pass base_frame=livox_frame so rtabmap doesn't
    # block waiting for base_link. use_sim_time=false on real hardware.
    for key in ("base_frame", "odom_frame", "map_frame", "use_sim_time"):
        if key in cfg:
            resolved[key] = str(cfg[key]).lower() if key == "use_sim_time" else str(cfg[key])

    # Map persistence (map_id / mapping vs localization). Adds map_mode +
    # database_path + reset_map to resolved.yaml; start_engine.sh / the
    # launch read them. Empty when no map_id (ephemeral — legacy behaviour).
    try:
        persist = _resolve_persistence(cfg)
    except RuntimeError as e:
        return Err(str(e))
    resolved.update(persist)
    # Remember the active map_id so on_shutdown (and the save_map helper)
    # know where to dump the human-previewable map. Only rtabmap persists
    # today; other algos ignore this.
    _ACTIVE.update({"algo": algo, "map_id": persist.get("map_id", ""),
                    "map_dir": os.path.dirname(persist["database_path"])
                    if persist else ""})

    _write_resolved_yaml(algo, resolved)

    # Declare outputs (after resolved.yaml so launch can start in parallel).
    _declare_outputs(mapping, algo)
    return Ok()


# Active-session scratch — set by init, read by on_shutdown / save helpers.
_ACTIVE: dict[str, str] = {}


@mapping.on_shutdown
def shutdown():
    """Best-effort: when a named map (map_id) was being built, dump a
    human-previewable snapshot (occupancy .pgm/.yaml/.png + cloud .pcd +
    meta) next to the rtabmap db before the SLAM nodes are torn down, so
    the map is usable offline without rtabmap's database viewer. The db
    itself is already persistent (rtabmap writes it live at database_path);
    this only adds the portable artifacts. Never fail shutdown on a save
    error — the db is the source of truth."""
    map_dir = _ACTIVE.get("map_dir")
    map_id = _ACTIVE.get("map_id")
    if not map_dir or not map_id or _ACTIVE.get("algo") != "rtabmap":
        return Ok()
    try:
        _export_map_snapshot(map_dir)
    except Exception as e:  # noqa: BLE001
        log.warning("map snapshot export failed (db still saved): %s", e)
    return Ok()


def _export_map_snapshot(map_dir: str) -> None:
    """Run scripts/save_map.py to snapshot the live occupancy grid + cloud
    into map_dir (pgm/yaml/png/pcd/meta). Bounded so a stuck topic can't
    hang shutdown."""
    import subprocess
    script = os.path.join(PKG_HOST_DIR, "scripts", "save_map.py")
    if not os.path.isfile(script):
        log.warning("save_map.py not found at %s — skipping snapshot", script)
        return
    log.info("exporting map snapshot → %s", map_dir)
    subprocess.run(
        ["python3", script, "--out-dir", map_dir, "--timeout", "10"],
        check=False, timeout=30,
    )


def main() -> int:
    mapping.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
