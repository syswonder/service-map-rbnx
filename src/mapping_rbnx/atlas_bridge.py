#!/usr/bin/env python3
"""mapping_rbnx Atlas bridge — registers SLAM capabilities with Robonix Atlas
and serves gRPC data-plane endpoints bridged from ROS2 topics.

Architecture:
                     ┌── Atlas ──────────────────────────────────────────┐
                     │  QueryNodes(contract_id=robonix/prm/sensor/lidar) │
                     │  NegotiateChannel(transport=ros2)                 │
                     └──────────┬────────────────────────────────────────┘
                                │ discover primitive endpoints
                                ▼
  PRM providers ──ROS2 topics──► atlas_bridge ──gRPC──► consumers
  (lidar/imu)                        │
                                     ▼
                           FASTLIO2_ROS2 (SLAM engine)
                           ├── fastlio2 (LIO odometry)
                           ├── pgo (loop closure + pose graph)
                           └── localizer (ICP relocalization)

Primitive consumption flow:
  1. QueryNodes(contract_id="robonix/prm/sensor/lidar3d") → find provider node
  2. NegotiateChannel(transport="ros2") → get ROS2 topic from metadata
  3. Subscribe to discovered topic → feed into fastlio2

gRPC data-plane services provided (codegen'd from robonix contracts):
  PrmBaseOdom.Stream                — stream nav_msgs/Odometry
  SrvSlamStatus.Call                — get SLAM status
  SrvSlamSaveMap.Call               — save map (via /pgo/save_maps)
  SrvSlamLoadMap.Call               — load map (via /localizer/relocalize)
  SrvSlamSwitchMode.Call            — switch mode
  SrvSlamSetInitialPose.Call        — set initial pose

Env vars:
  ROBONIX_ATLAS             Atlas endpoint (default: localhost:50051)
  SLAM_MODE                 Initial mode: mapping | localization (default: mapping)
  MAP_FILE                  Pre-built map path for localization mode
  MAPPING_GRPC_PORT         gRPC data-plane listen port (default: 50120)
  MAP_SAVE_DIR              Directory for saved maps (default: /maps)
"""
import json
import logging
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="[mapping_rbnx] %(levelname)s %(message)s",
)
log = logging.getLogger("mapping_rbnx")

# ── Proto path setup ──────────────────────────────────────────────────────────

def _ensure_proto_gen() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_gen()

import grpc
from concurrent import futures as _grpc_futures
from google.protobuf import empty_pb2

import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc
import robonix_contracts_pb2_grpc
import nav_msgs_pb2
import slam_pb2

# ── ROS2 imports (deferred) ──────────────────────────────────────────────────

_rclpy = None
_OdometryMsg = None
_PointCloud2Msg = None

def _import_ros2():
    global _rclpy, _OdometryMsg, _PointCloud2Msg
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        _rclpy = rclpy
        _OdometryMsg = Odometry
        _PointCloud2Msg = PointCloud2
        return True
    except ImportError:
        log.warning("rclpy not available — running without ROS2 subscriptions")
        return False


# ── Shared state ──────────────────────────────────────────────────────────────

class SlamState:
    """Thread-safe container for latest SLAM data from ROS2 topics."""

    def __init__(self):
        self._lock = threading.Lock()
        self._odom_cond = threading.Condition(self._lock)
        self.mode = os.environ.get("SLAM_MODE", "mapping")
        self.last_odom = None
        self.last_odom_time = 0.0
        self.last_cloud_time = 0.0
        self.odom_count = 0
        self.start_time = time.time()
        self.map_file = os.environ.get("MAP_FILE", "")
        self.map_save_dir = os.environ.get("MAP_SAVE_DIR", "/maps")

    def update_odom(self, msg):
        with self._odom_cond:
            self.last_odom = msg
            self.last_odom_time = time.time()
            self.odom_count += 1
            self._odom_cond.notify_all()

    def update_cloud(self, msg):
        with self._lock:
            self.last_cloud_time = time.time()

    def wait_for_odom(self, timeout: float = 1.0) -> bool:
        with self._odom_cond:
            return self._odom_cond.wait(timeout=timeout)

    def get_status_proto(self) -> slam_pb2.GetSlamStatus_Response:
        with self._lock:
            now = time.time()
            odom_age = now - self.last_odom_time if self.last_odom_time > 0 else -1
            cloud_age = now - self.last_cloud_time if self.last_cloud_time > 0 else -1
            elapsed = now - self.start_time
            odom_hz = self.odom_count / elapsed if elapsed > 1.0 else 0

            status = slam_pb2.SlamStatus()
            status.header.stamp.sec = int(now)
            status.header.stamp.nanosec = int((now % 1) * 1e9) % 1_000_000_000
            status.mode = self.mode
            status.odom_alive = odom_age >= 0 and odom_age < 2.0
            status.odom_hz = odom_hz
            status.cloud_alive = cloud_age >= 0 and cloud_age < 5.0
            status.map_file = self.map_file
            status.total_odom_frames = self.odom_count

            resp = slam_pb2.GetSlamStatus_Response()
            resp.status.CopyFrom(status)
            return resp

    def get_last_odom_proto(self):
        """Convert latest ROS2 Odometry to protobuf nav_msgs_pb2.Odometry."""
        with self._lock:
            if self.last_odom is None:
                return None
            return _ros2_odom_to_proto(self.last_odom)


state = SlamState()


# ── ROS2 → Proto conversion ──────────────────────────────────────────────────

def _ros2_odom_to_proto(msg) -> nav_msgs_pb2.Odometry:
    odom = nav_msgs_pb2.Odometry()
    odom.header.frame_id = msg.header.frame_id
    odom.header.stamp.sec = int(msg.header.stamp.sec)
    odom.header.stamp.nanosec = int(msg.header.stamp.nanosec)
    odom.child_frame_id = msg.child_frame_id

    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    odom.pose.pose.position.x = p.x
    odom.pose.pose.position.y = p.y
    odom.pose.pose.position.z = p.z
    odom.pose.pose.orientation.x = o.x
    odom.pose.pose.orientation.y = o.y
    odom.pose.pose.orientation.z = o.z
    odom.pose.pose.orientation.w = o.w

    if hasattr(msg.pose, 'covariance') and len(msg.pose.covariance) == 36:
        for v in msg.pose.covariance:
            odom.pose.covariance.append(float(v))

    t = msg.twist.twist
    odom.twist.twist.linear.x = t.linear.x
    odom.twist.twist.linear.y = t.linear.y
    odom.twist.twist.linear.z = t.linear.z
    odom.twist.twist.angular.x = t.angular.x
    odom.twist.twist.angular.y = t.angular.y
    odom.twist.twist.angular.z = t.angular.z

    if hasattr(msg.twist, 'covariance') and len(msg.twist.covariance) == 36:
        for v in msg.twist.covariance:
            odom.twist.covariance.append(float(v))

    return odom


# ── gRPC Servicers ────────────────────────────────────────────────────────────

class PrmBaseOdomServicer(robonix_contracts_pb2_grpc.PrmBaseOdomServicer):
    """Contract: robonix/prm/base/odom — stream Odometry from fastlio2."""

    def Stream(self, request, context):
        log.info("PrmBaseOdom.Stream client connected")
        last_count = 0
        while context.is_active():
            state.wait_for_odom(timeout=1.0)
            with state._lock:
                if state.odom_count == last_count:
                    continue
                last_count = state.odom_count
            odom_proto = state.get_last_odom_proto()
            if odom_proto:
                yield odom_proto
        log.info("PrmBaseOdom.Stream client disconnected")


class SrvSlamStatusServicer(robonix_contracts_pb2_grpc.SrvSlamStatusServicer):
    """Contract: robonix/srv/slam/status"""

    def Call(self, request, context):
        return state.get_status_proto()


class SrvSlamSaveMapServicer(robonix_contracts_pb2_grpc.SrvSlamSaveMapServicer):
    """Contract: robonix/srv/slam/save_map — calls /pgo/save_maps ROS2 service."""

    def Call(self, request, context):
        filename = request.filename.strip() or "map"
        save_dir = Path(state.map_save_dir)
        save_dir.mkdir(parents=True, exist_ok=True)
        save_path = save_dir / filename

        resp = slam_pb2.SaveMap_Response()

        if _rclpy is not None:
            try:
                # Call the PGO save_maps service (interface/srv/SaveMaps)
                result = subprocess.run(
                    ["ros2", "service", "call", "/pgo/save_maps",
                     "interface/srv/SaveMaps",
                     f"{{file_path: '{save_path}', save_patches: true}}"],
                    capture_output=True, text=True, timeout=60,
                )
                resp.success = result.returncode == 0
                resp.path = str(save_path)
                resp.message = "Map saved via PGO" if resp.success else result.stderr
            except Exception as e:
                resp.success = False
                resp.message = str(e)
        else:
            resp.success = False
            resp.message = "ROS2 not available"

        return resp


class SrvSlamLoadMapServicer(robonix_contracts_pb2_grpc.SrvSlamLoadMapServicer):
    """Contract: robonix/srv/slam/load_map — calls /localizer/relocalize ROS2 service."""

    def Call(self, request, context):
        map_path = request.path.strip()
        resp = slam_pb2.LoadMap_Response()

        if not map_path or not Path(map_path).exists():
            resp.success = False
            resp.message = f"Map file not found: {map_path}"
            return resp

        state.map_file = map_path
        state.mode = "localization"

        if _rclpy is not None:
            try:
                # Call the localizer relocalize service (interface/srv/Relocalize)
                # Initial pose defaults to origin; use set_initial_pose for offset
                result = subprocess.run(
                    ["ros2", "service", "call", "/localizer/relocalize",
                     "interface/srv/Relocalize",
                     f"{{pcd_path: '{map_path}', x: 0.0, y: 0.0, z: 0.0, "
                     f"yaw: 0.0, pitch: 0.0, roll: 0.0}}"],
                    capture_output=True, text=True, timeout=30,
                )
                resp.success = result.returncode == 0
                resp.message = f"Relocalization initiated: {map_path}" if resp.success else result.stderr
            except Exception as e:
                resp.success = False
                resp.message = str(e)
        else:
            resp.success = False
            resp.message = "ROS2 not available"

        return resp


class SrvSlamSwitchModeServicer(robonix_contracts_pb2_grpc.SrvSlamSwitchModeServicer):
    """Contract: robonix/srv/slam/switch_mode"""

    def Call(self, request, context):
        mode = request.mode.strip().lower()
        resp = slam_pb2.SwitchMode_Response()

        if mode not in ("mapping", "localization", "idle"):
            resp.success = False
            resp.current_mode = state.mode
            resp.message = f"Invalid mode: {mode}. Use: mapping, localization, idle"
            return resp

        state.mode = mode
        log.info("SLAM mode → %s", mode)
        resp.success = True
        resp.current_mode = mode
        resp.message = f"Switched to {mode}"
        return resp


class SrvSlamSetInitialPoseServicer(robonix_contracts_pb2_grpc.SrvSlamSetInitialPoseServicer):
    """Contract: robonix/srv/slam/set_initial_pose — calls /localizer/relocalize with pose."""

    def Call(self, request, context):
        resp = slam_pb2.SetInitialPose_Response()
        pose = request.pose

        if _rclpy is not None and state.map_file:
            try:
                p = pose.pose.position
                o = pose.pose.orientation
                # Convert quaternion to euler for the Relocalize service
                import math
                # yaw from quaternion (simplified)
                siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
                cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                sinp = 2.0 * (o.w * o.y - o.z * o.x)
                pitch = math.asin(max(-1.0, min(1.0, sinp)))
                sinr_cosp = 2.0 * (o.w * o.x + o.y * o.z)
                cosr_cosp = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
                roll = math.atan2(sinr_cosp, cosr_cosp)

                result = subprocess.run(
                    ["ros2", "service", "call", "/localizer/relocalize",
                     "interface/srv/Relocalize",
                     f"{{pcd_path: '{state.map_file}', "
                     f"x: {p.x}, y: {p.y}, z: {p.z}, "
                     f"yaw: {yaw}, pitch: {pitch}, roll: {roll}}}"],
                    capture_output=True, text=True, timeout=30,
                )
                resp.success = result.returncode == 0
                resp.message = (
                    f"Relocalization with pose [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]"
                    if resp.success else result.stderr
                )
            except Exception as e:
                resp.success = False
                resp.message = str(e)
        else:
            resp.success = False
            resp.message = "ROS2 not available or no map loaded"

        return resp


# ── Primitive discovery via Atlas ─────────────────────────────────────────────

# Consumed primitives: contract_id → (ros2_msg_type, fallback_topic, callback)
# Note: no camera — FASTLIO2 is LiDAR-Inertial only
_CONSUMED_PRIMITIVES = {
    "robonix/prm/sensor/lidar3d": {
        "ros2_msg_type": "livox_ros_driver2/msg/CustomMsg",
        "fallback_topic": "/livox/lidar",
        "description": "3D LiDAR point cloud (Livox CustomMsg)",
    },
    "robonix/prm/sensor/imu": {
        "ros2_msg_type": "sensor_msgs/msg/Imu",
        "fallback_topic": "/livox/imu",
        "description": "IMU measurements",
    },
}

# FASTLIO2 output topics (produced by SLAM engine, consumed by bridge)
_SLAM_OUTPUT_TOPICS = {
    "odom": "/fastlio2/lio_odom",
    "cloud": "/fastlio2/body_cloud",
}


def _discover_primitive_topic(stub, node_id: str, contract_id: str, fallback: str) -> str:
    """Query Atlas for a primitive provider and negotiate a ROS2 channel.

    Returns the ROS2 topic name to subscribe to. Falls back to `fallback`
    if Atlas is unavailable or no provider is registered.

    Flow:
      1. QueryNodes(contract_id=...) → find provider with ros2 transport
      2. NegotiateChannel(consumer_id=self, provider=..., transport="ros2")
      3. Parse endpoint/metadata for ros2_topic
    """
    try:
        resp = stub.QueryNodes(pb.QueryNodesRequest(
            contract_id=contract_id,
            transport="ros2",
        ))

        if not resp.nodes:
            log.info("  [%s] no provider found, using fallback: %s", contract_id, fallback)
            return fallback

        provider = resp.nodes[0]
        provider_node_id = provider.node_id

        interface_name = ""
        ros2_topic = fallback
        for iface in provider.interfaces:
            if iface.contract_id == contract_id:
                interface_name = iface.name
                try:
                    meta = json.loads(iface.metadata_json) if iface.metadata_json else {}
                    ros2_topic = meta.get("ros2_topic", fallback)
                except (json.JSONDecodeError, AttributeError):
                    pass
                break

        if not interface_name:
            log.info("  [%s] provider %s has no matching interface, fallback: %s",
                     contract_id, provider_node_id, fallback)
            return fallback

        ch_resp = stub.NegotiateChannel(pb.NegotiateChannelRequest(
            consumer_id=node_id,
            provider_node_id=provider_node_id,
            interface_name=interface_name,
            transport="ros2",
        ))

        # Atlas NegotiateChannel returns the allocated endpoint — this is the
        # provider's declared topic when there is no conflict, else a UUID channel.
        # Trust Atlas's allocation (the channel endpoint is the source of truth).
        negotiated_topic = ch_resp.endpoint if ch_resp.endpoint else ros2_topic
        if negotiated_topic.startswith("topic:"):
            negotiated_topic = negotiated_topic[len("topic:"):]

        log.info("  [%s] discovered: provider=%s topic=%s (channel=%s)",
                 contract_id, provider_node_id, negotiated_topic, ch_resp.channel_id)
        return negotiated_topic

    except Exception as e:
        log.warning("  [%s] discovery failed: %s — using fallback: %s",
                    contract_id, e, fallback)
        return fallback


def _discover_all_primitives(stub, node_id: str) -> dict:
    """Discover consumed primitives. Prefer IMU from the same provider as lidar3d."""
    log.info("Discovering consumed primitives via Atlas...")
    resolved = {}

    # Resolve lidar3d first and remember which provider_node_id won
    lidar_cid = "robonix/prm/sensor/lidar3d"
    preferred_provider = ""
    try:
        resp = stub.QueryNodes(pb.QueryNodesRequest(contract_id=lidar_cid, transport="ros2"))
        if resp.nodes:
            preferred_provider = resp.nodes[0].node_id
    except Exception as e:
        log.warning("lidar3d lookup for provider preference failed: %s", e)

    def _resolve_with_pref(contract_id, fallback, pref_provider):
        """Like _discover_primitive_topic, but prefer a specific provider if available."""
        try:
            resp = stub.QueryNodes(pb.QueryNodesRequest(contract_id=contract_id, transport="ros2"))
            if not resp.nodes:
                return fallback
            # pick provider: same as preferred, else first
            chosen = None
            for n in resp.nodes:
                if n.node_id == pref_provider:
                    chosen = n; break
            if chosen is None:
                chosen = resp.nodes[0]
            topic = fallback
            iface_name = ""
            for iface in chosen.interfaces:
                if iface.contract_id == contract_id:
                    iface_name = iface.name
                    try:
                        meta = json.loads(iface.metadata_json) if iface.metadata_json else {}
                        topic = meta.get("ros2_topic", fallback)
                    except Exception:
                        pass
                    break
            try:
                stub.NegotiateChannel(pb.NegotiateChannelRequest(
                    consumer_id=node_id, provider_node_id=chosen.node_id,
                    interface_name=iface_name, transport="ros2"))
            except Exception:
                pass
            log.info("  [%s] provider=%s topic=%s (preferred=%s)", contract_id, chosen.node_id, topic, pref_provider or "(none)")
            return topic
        except Exception as e:
            log.warning("  [%s] discovery error: %s, fallback=%s", contract_id, e, fallback)
            return fallback

    for contract_id, info in _CONSUMED_PRIMITIVES.items():
        topic = _resolve_with_pref(contract_id, info["fallback_topic"], preferred_provider)
        resolved[contract_id] = topic
    return resolved


# ── ROS2 subscriber thread ───────────────────────────────────────────────────

def _ros2_spin_thread(primitive_topics: dict):
    """Spin a ROS2 node that subscribes to:
    1. Discovered primitive topics (forwarded to FASTLIO2 via topic remapping)
    2. FASTLIO2 output topics (odom, cloud → SlamState for gRPC serving)
    """
    _rclpy.init()
    node = _rclpy.create_node("mapping_rbnx_bridge")

    # Subscribe to FASTLIO2 outputs (produced by SLAM engine)
    odom_topic = _SLAM_OUTPUT_TOPICS["odom"]
    cloud_topic = _SLAM_OUTPUT_TOPICS["cloud"]
    node.create_subscription(_OdometryMsg, odom_topic, state.update_odom, 10)
    node.create_subscription(_PointCloud2Msg, cloud_topic, state.update_cloud, 10)
    log.info("Subscribed to SLAM outputs: %s, %s", odom_topic, cloud_topic)

    # Log discovered primitive inputs (fastlio2 subscribes to these directly;
    # the bridge monitors them for health reporting)
    for contract_id, topic in primitive_topics.items():
        desc = _CONSUMED_PRIMITIVES[contract_id]["description"]
        log.info("Primitive input [%s]: %s → %s", contract_id, desc, topic)

    _rclpy.spin(node)
    node.destroy_node()
    _rclpy.shutdown()


# ── Atlas registration ────────────────────────────────────────────────────────

def _load_skills() -> list:
    skills = []
    skills_dir = Path(__file__).resolve().parent.parent.parent / "skills"
    if skills_dir.exists():
        for skill_dir in sorted(skills_dir.iterdir()):
            skill_md = skill_dir / "SKILL.md"
            if skill_md.exists():
                content = skill_md.read_text()
                name = skill_dir.name
                description = ""
                if content.startswith("---"):
                    parts = content.split("---", 2)
                    if len(parts) >= 3:
                        try:
                            import yaml
                            fm = yaml.safe_load(parts[1])
                            name = fm.get("name", name)
                            description = fm.get("description", "")
                        except Exception:
                            pass
                skills.append(pb.SkillInfo(
                    name=name,
                    description=description,
                    path=str(skill_md),
                    metadata_json="{}",
                ))
    return skills


def _register_and_discover(grpc_port: int) -> dict:
    """Register with Atlas, declare provided interfaces, discover consumed primitives.

    Returns dict of discovered primitive topics (contract_id → ros2 topic).
    """
    atlas_endpoint = os.environ.get("ROBONIX_ATLAS", "localhost:50051")
    channel = grpc.insecure_channel(atlas_endpoint)
    stub = pb_grpc.RobonixRuntimeStub(channel)
    node_id = "com.robonix.mapping.fastlio2"

    primitive_topics = {}

    try:
        # ── Step 1: Register this node ────────────────────────────────────
        stub.RegisterNode(pb.RegisterNodeRequest(
            node_id=node_id,
            namespace="robonix/srv/slam",
            kind="service",
            distro="humble",
            skills=_load_skills(),
        ))

        # ── Step 2: Declare provided interfaces ──────────────────────────
        odom_topic = _SLAM_OUTPUT_TOPICS["odom"]
        interfaces = [
            ("odom", ["grpc", "ros2"], "robonix/prm/base/odom", {
                "ros2_topic": odom_topic,
                "ros2_msg_type": "nav_msgs/msg/Odometry",
                "grpc_service": "PrmBaseOdom",
            }),
            ("status", ["grpc"], "robonix/srv/slam/status", {
                "grpc_service": "SrvSlamStatus",
            }),
            ("save_map", ["grpc"], "robonix/srv/slam/save_map", {
                "grpc_service": "SrvSlamSaveMap",
            }),
            ("load_map", ["grpc"], "robonix/srv/slam/load_map", {
                "grpc_service": "SrvSlamLoadMap",
            }),
            ("switch_mode", ["grpc"], "robonix/srv/slam/switch_mode", {
                "grpc_service": "SrvSlamSwitchMode",
            }),
            ("set_initial_pose", ["grpc"], "robonix/srv/slam/set_initial_pose", {
                "grpc_service": "SrvSlamSetInitialPose",
            }),
            # ── map data plane ──────────────────────────────────────────
            ("map_pointcloud", ["ros2"], "robonix/srv/common/map/pointcloud", {
                "ros2_topic": "/fastlio2/world_cloud",
                "ros2_msg_type": "sensor_msgs/msg/PointCloud2",
            }),
            ("map_occupancy_grid", ["ros2"], "robonix/srv/common/map/occupancy_grid", {
                "ros2_topic": "/robonix/map/occupancy_grid",
                "ros2_msg_type": "nav_msgs/msg/OccupancyGrid",
            }),
            ("map_scan_2d", ["ros2"], "robonix/srv/common/map/scan_2d", {
                "ros2_topic": "/robonix/map/scan_2d",
                "ros2_msg_type": "sensor_msgs/msg/LaserScan",
            }),
        ]

        for name, transports, contract_id, meta in interfaces:
            stub.DeclareInterface(pb.DeclareInterfaceRequest(
                node_id=node_id,
                name=name,
                supported_transports=transports,
                metadata_json=json.dumps(meta),
                listen_port=grpc_port,
                contract_id=contract_id,
            ))

        log.info("Registered with Atlas: %s — %d interfaces on port %d",
                 node_id, len(interfaces), grpc_port)

        # ── Step 3: Discover consumed primitives ─────────────────────────
        primitive_topics = _discover_all_primitives(stub, node_id)

    except Exception as e:
        log.warning("Atlas registration/discovery failed: %s (using fallback topics)", e)
        for contract_id, info in _CONSUMED_PRIMITIVES.items():
            primitive_topics[contract_id] = info["fallback_topic"]

    # ── Heartbeat thread ─────────────────────────────────────────────────
    def _heartbeat():
        while True:
            try:
                stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
            except Exception:
                pass
            time.sleep(10)

    threading.Thread(target=_heartbeat, daemon=True).start()

    return primitive_topics


# ── gRPC server ───────────────────────────────────────────────────────────────

def _run_grpc_server(port: int):
    server = grpc.server(_grpc_futures.ThreadPoolExecutor(max_workers=8))

    robonix_contracts_pb2_grpc.add_PrmBaseOdomServicer_to_server(
        PrmBaseOdomServicer(), server)
    robonix_contracts_pb2_grpc.add_SrvSlamStatusServicer_to_server(
        SrvSlamStatusServicer(), server)
    robonix_contracts_pb2_grpc.add_SrvSlamSaveMapServicer_to_server(
        SrvSlamSaveMapServicer(), server)
    robonix_contracts_pb2_grpc.add_SrvSlamLoadMapServicer_to_server(
        SrvSlamLoadMapServicer(), server)
    robonix_contracts_pb2_grpc.add_SrvSlamSwitchModeServicer_to_server(
        SrvSlamSwitchModeServicer(), server)
    robonix_contracts_pb2_grpc.add_SrvSlamSetInitialPoseServicer_to_server(
        SrvSlamSetInitialPoseServicer(), server)

    server.add_insecure_port(f"0.0.0.0:{port}")
    server.start()
    log.info("gRPC data-plane on 0.0.0.0:%d", port)
    log.info("  PrmBaseOdom.Stream        (robonix/prm/base/odom)")
    log.info("  SrvSlamStatus.Call        (robonix/srv/slam/status)")
    log.info("  SrvSlamSaveMap.Call       (robonix/srv/slam/save_map)")
    log.info("  SrvSlamLoadMap.Call       (robonix/srv/slam/load_map)")
    log.info("  SrvSlamSwitchMode.Call    (robonix/srv/slam/switch_mode)")
    log.info("  SrvSlamSetInitialPose.Call(robonix/srv/slam/set_initial_pose)")
    return server




def _write_resolved_lio_config(primitive_topics: dict) -> str:
    """Patch fastlio2 config with Atlas-discovered topics. Returns path to resolved yaml.

    Reads the installed lio.yaml template (from colcon install), substitutes
    lidar_topic/imu_topic with values discovered via Atlas QueryNodes, and
    writes to /tmp/lio_resolved.yaml for the launch file to consume.
    """
    lidar = primitive_topics.get("robonix/prm/sensor/lidar3d", "")
    imu   = primitive_topics.get("robonix/prm/sensor/imu", "")
    if not lidar or not imu:
        log.warning("Cannot resolve lio config — missing lidar3d (%s) or imu (%s)", lidar, imu)
        return ""

    # Find template
    candidates = [
        os.path.expanduser("~/wheatfox/packages/mapping_rbnx_ws/install/fastlio2/share/fastlio2/config/lio.yaml"),
        "/opt/ros/humble/share/fastlio2/config/lio.yaml",
    ]
    tmpl = next((p for p in candidates if os.path.exists(p)), "")
    if not tmpl:
        log.error("lio.yaml template not found in %s", candidates)
        return ""

    with open(tmpl) as f:
        src = f.read()

    # Simple key:value replace (YAML is flat at top level)
    import re as _re
    src = _re.sub(r"^lidar_topic:.*", f"lidar_topic: {lidar}", src, flags=_re.MULTILINE)
    src = _re.sub(r"^imu_topic:.*",   f"imu_topic: {imu}",     src, flags=_re.MULTILINE)

    out = "/tmp/lio_resolved.yaml"
    with open(out, "w") as f:
        f.write(src)
    log.info("Wrote resolved lio config -> %s (lidar=%s imu=%s)", out, lidar, imu)
    return out

# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    grpc_port = int(os.environ.get("MAPPING_GRPC_PORT", "50120"))

    # Step 1: Register with Atlas + discover primitive providers
    primitive_topics = _register_and_discover(grpc_port)
    _write_resolved_lio_config(primitive_topics)

    # Step 2: Start ROS2 subscriber thread (uses discovered topics)
    if _import_ros2():
        threading.Thread(
            target=_ros2_spin_thread,
            args=(primitive_topics,),
            daemon=True,
        ).start()
        log.info("ROS2 subscriber thread started")
    else:
        log.info("Running without ROS2 — gRPC streams will block until data arrives")

    # Step 3: Start gRPC data-plane server (blocking)
    server = _run_grpc_server(grpc_port)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        log.info("Shutting down...")
        server.stop(grace=5)


if __name__ == "__main__":
    main()
