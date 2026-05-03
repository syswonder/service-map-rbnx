# SPDX-License-Identifier: MulanPSL-2.0
"""mapping_rbnx atlas bridge — new robonix dev-packaging API.

Responsibilities (kept tight; SLAM nodes do the actual work):
  1. Register `com.robonix.service.mapping` as a capability with atlas.
  2. Read the manifest config (RBNX_CONFIG_FILE) to learn:
       - algo:    rtabmap (2D lidar+RGBD, default for both webots and
                  real-robot scenarios with 2D scan + RGBD camera)
                | dlio    (3D LiDAR-Inertial Odometry; primary choice
                  for real robot with Livox / 3D scanner + IMU)
                | fastlio2 [BROKEN: drift] (kept reachable for repro;
                  do NOT pick for production until the global drift
                  issue is fixed — see open issue tracker)
       - sensors: lidar2d / lidar3d / rgb / rgbd / imu booleans
       - platform: x86_desktop / jetson_orin
  3. Resolve sensor primitives via atlas (QueryCapabilities + ConnectCapability),
     write `/tmp/<algo>_resolved.yaml` for the launch file.
  4. Declare the algo-appropriate output endpoints on atlas, ALL under
     the SAME contract surface (robonix/service/map/*) so consumers
     (scene, nav) are algo-agnostic.

slam_toolbox + cartographer were removed in favour of rtabmap after
extensive head-to-head testing in webots: rtabmap handles curve-motion
mapping, loop closure, and depth fusion better than either alternative
for the cost of one extra dependency.
"""
from __future__ import annotations

import json
import logging
import os
import sys
import threading
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


# ── Config ────────────────────────────────────────────────────────────────────
ATLAS_ENDPOINT = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
CAP_ID = os.environ.get("ROBONIX_CAPABILITY_ID", "com.robonix.service.mapping")
NAMESPACE = "robonix/service/map"
PKG_HOST_DIR = os.environ.get("ROBONIX_PKG_HOST_DIR", "/mapping")
RESOLVED_DIR = os.environ.get("MAPPING_RESOLVED_DIR", "/tmp")
HEARTBEAT_PERIOD_S = 10.0


def _truthy(s: str) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "on")


def _load_manifest_config() -> dict:
    """Parse RBNX_CONFIG_FILE (json from rbnx) into a dict.
    Returns {} when unset / unreadable — defaults are applied later."""
    path = os.environ.get("RBNX_CONFIG_FILE", "")
    if not path or not os.path.isfile(path):
        return {}
    try:
        with open(path) as f:
            return json.load(f)
    except Exception as e:  # noqa: BLE001
        log.warning("RBNX_CONFIG_FILE unreadable (%s): %s — using defaults", path, e)
        return {}


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
    },
    "dlio": {
        # Direct LiDAR-Inertial Odometry: real-robot 3D livox path.
        # No native 2D OccupancyGrid — projected from /dlio's cloud
        # by a pointcloud_to_grid adapter spawned in the launch.
        "robonix/service/map/occupancy_grid": "/robonix/map/occupancy_grid",
        "robonix/service/map/pointcloud":     "/dlio/odom_node/pointcloud/deskewed",
    },
    "fastlio2": {
        # [BROKEN: drift] same shape as dlio, kept for repro only.
        "robonix/service/map/occupancy_grid": "/robonix/map/occupancy_grid",
        "robonix/service/map/pointcloud":     "/fastlio2/world_cloud",
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


# ── Atlas client helpers ──────────────────────────────────────────────────────
def _atlas_stub() -> pb_grpc.AtlasStub:
    chan = grpc.insecure_channel(ATLAS_ENDPOINT)
    return pb_grpc.AtlasStub(chan)


def _register_self(stub) -> None:
    """RegisterCapability with atlas. Idempotent on re-deploy: atlas
    rejects duplicates with ALREADY_EXISTS, which we treat as a soft
    success (re-using an existing instance)."""
    try:
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=CAP_ID,
            namespace=NAMESPACE,
            capability_md_path=str(Path(PKG_HOST_DIR) / "CAPABILITY.md") if (Path(PKG_HOST_DIR) / "CAPABILITY.md").exists() else "",
        ))
        log.info("registered cap %s namespace=%s", CAP_ID, NAMESPACE)
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.ALREADY_EXISTS:
            log.info("cap %s already registered (re-deploy); reusing", CAP_ID)
        else:
            raise


def _declare_outputs(stub, algo: str) -> None:
    """DeclareInterface(transport=ROS2) for every exported contract.
    All algos publish the same contract surface — only the underlying
    ROS topic differs — so consumers never need to know which algo
    is running."""
    bindings = _ALGO_TOPIC_BINDINGS[algo]
    declared = 0
    for contract_id in _EXPORTED_CONTRACTS:
        topic = bindings[contract_id]
        try:
            stub.DeclareInterface(pb.DeclareInterfaceRequest(
                capability_id=CAP_ID,
                contract_id=contract_id,
                transport=pb.TRANSPORT_ROS2,
                endpoint=topic,
                params=pb.TransportParams(
                    ros2=pb.Ros2Params(qos_profile="reliable"),
                ),
            ))
            declared += 1
            log.info("declared %s → ROS2 topic %s", contract_id, topic)
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.ALREADY_EXISTS:
                log.info("%s already declared (re-deploy); ok", contract_id)
            else:
                log.warning("declare %s failed: %s", contract_id, e)
    log.info("declared %d/%d output(s) for algo=%s",
             declared, len(_EXPORTED_CONTRACTS), algo)


def _resolve_sensor_endpoint(stub, contract_id: str) -> Optional[str]:
    """QueryCapabilities for the contract over ROS2; if found,
    ConnectCapability to read the endpoint (atlas only discloses
    endpoints after a Connect)."""
    try:
        resp = stub.QueryCapabilities(pb.QueryCapabilitiesRequest(
            contract_id=contract_id,
            transport=pb.TRANSPORT_ROS2,
        ))
    except grpc.RpcError as e:
        log.warning("query %s failed: %s", contract_id, e)
        return None
    for rec in resp.records:
        for iface in rec.interfaces:
            if iface.contract_id != contract_id or iface.transport != pb.TRANSPORT_ROS2:
                continue
            try:
                conn = stub.ConnectCapability(pb.ConnectCapabilityRequest(
                    consumer_id=CAP_ID,
                    capability_id=rec.capability_id,
                    contract_id=contract_id,
                    transport=pb.TRANSPORT_ROS2,
                ))
                if conn.endpoint:
                    return conn.endpoint
            except grpc.RpcError as e:
                log.warning("connect %s failed: %s", contract_id, e)
    return None


def _resolve_sensors(stub, cfg: dict) -> dict[str, str]:
    """For each enabled sensor, ask atlas for the topic. Falls back
    to no-key when the primitive isn't online yet (resolved.yaml will
    still be useful — launch picks a sane default)."""
    enabled = _enabled_sensors(cfg)
    resolved: dict[str, str] = {}
    for cfg_key, contract_id, yaml_key in _SENSOR_CONTRACTS:
        if not enabled.get(cfg_key):
            continue
        ep = _resolve_sensor_endpoint(stub, contract_id)
        if ep:
            resolved[yaml_key] = ep
            log.info("resolved %s → %s = %s", contract_id, yaml_key, ep)
        else:
            log.info("sensor %s (%s) not available on atlas yet", cfg_key, contract_id)
    return resolved


def _retry_resolve(stub, cfg: dict, deadline_s: float = 30.0,
                   settle_s: float = 8.0) -> dict[str, str]:
    """Same retry+settle pattern as scene: wait until at least one
    sensor lands, then keep absorbing late arrivers for `settle_s`."""
    deadline = time.time() + deadline_s
    resolved: dict[str, str] = {}
    while time.time() < deadline:
        resolved = _resolve_sensors(stub, cfg)
        if resolved:
            break
        time.sleep(2.0)
    if not resolved:
        log.warning("no sensors discovered within %.1fs — launch will use defaults", deadline_s)
        return {}
    settle_until = time.time() + settle_s
    while time.time() < settle_until:
        time.sleep(2.0)
        more = _resolve_sensors(stub, cfg)
        if len(more) > len(resolved):
            log.info("sensor-resolve settle: %d → %d", len(resolved), len(more))
            resolved = more
    return resolved


def _write_resolved_yaml(algo: str, resolved: dict[str, str]) -> str:
    """Write /tmp/<algo>_resolved.yaml with k=v lines. start_engine.sh
    `grep`s these out — yaml-lite is fine, no parser needed."""
    out = os.path.join(RESOLVED_DIR, f"{algo}_resolved.yaml")
    # tf frames default for webots tiago
    defaults = {
        "base_frame": "base_link",
        "odom_frame": "odom",
        "map_frame": "map",
    }
    merged = {**defaults, **resolved}
    with open(out, "w") as f:
        for k, v in merged.items():
            f.write(f"{k}: {v}\n")
    log.info("wrote resolved config → %s (%d keys)", out, len(merged))
    return out


# ── Main loop ─────────────────────────────────────────────────────────────────
def _heartbeat_loop(stub) -> None:
    """Atlas evicts caps that don't heartbeat for 60s. Send every 10s."""
    while True:
        try:
            stub.Heartbeat(pb.HeartbeatRequest(capability_id=CAP_ID))
        except Exception:
            pass
        time.sleep(HEARTBEAT_PERIOD_S)


def main() -> int:
    cfg = _load_manifest_config()
    algo = cfg.get("algo", "rtabmap")
    if algo not in _ALGO_TOPIC_BINDINGS:
        log.error("unknown algo %r — supported: %s",
                  algo, list(_ALGO_TOPIC_BINDINGS))
        return 2
    _check_binding_complete(algo)
    if algo == "fastlio2":
        log.warning("algo=fastlio2 is BROKEN (drift); use only for repro/debug")

    log.info("starting bridge: algo=%s atlas=%s cap=%s",
             algo, ATLAS_ENDPOINT, CAP_ID)

    # Persist algo for start_engine.sh.
    os.environ["MAPPING_ALGO"] = algo
    Path("/tmp/mapping_algo").write_text(algo)

    # Reach atlas (with brief retry — atlas may have just started).
    stub = _atlas_stub()
    for _ in range(10):
        try:
            stub.RegisterCapability(pb.RegisterCapabilityRequest(
                capability_id=CAP_ID, namespace=NAMESPACE, capability_md_path=""))
            break
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.ALREADY_EXISTS:
                break
            time.sleep(1.0)
    log.info("registered cap %s", CAP_ID)

    # Discover sensors → resolved.yaml.
    resolved = _retry_resolve(stub, cfg)
    _write_resolved_yaml(algo, resolved)

    # Declare outputs (after resolved.yaml so launch can start in parallel).
    _declare_outputs(stub, algo)

    # Heartbeat forever — exit only on signal.
    threading.Thread(target=_heartbeat_loop, args=(stub,), daemon=True).start()
    log.info("atlas_bridge ready; idling on heartbeat")
    try:
        while True:
            time.sleep(60.0)
    except KeyboardInterrupt:
        log.info("interrupted; shutting down")
    return 0


if __name__ == "__main__":
    sys.exit(main())
