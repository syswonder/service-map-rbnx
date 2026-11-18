#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Hold scans until old graph and a timestamp-correct start pose are loaded.

The mapping node only subscribes to this relay's output. During a process swap
the robot may move: transport the verified pose using odom at the FIRST held
scan's timestamp, load the graph, then replay held scans in timestamp order.
"""
import math
import base64
import json
from collections import deque
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger
from slam_toolbox.srv import DeserializePoseGraph
from tf2_ros import Buffer, TransformListener
from mapping_rbnx.slam_motion import advance, tuple_from_transform
from mapping_rbnx.slam_planarity import PlanarScanGuard


class ScanGate(Node):
    def __init__(self):
        super().__init__("slam_scan_gate")
        defaults = {"scan_topic": "/scan", "output_topic": "/slam_toolbox/input_scan",
                    "map_file_name": "", "odom_frame": "odom", "base_frame": "base_link",
                    "map_frame": "map",
                    "start_x": 0., "start_y": 0., "start_yaw": 0.,
                    "anchor_odom_x": 0., "anchor_odom_y": 0., "anchor_odom_yaw": 0.,
                    "bootstrap_state": "waiting", "sensor_signature": "",
                    "max_scan_tilt_deg": 3., "max_scan_height_change_m": .04,
                    "dropped_nonplanar_scans": 0, "dropped_missing_tf_scans": 0,
                    "bootstrap_prime_attempts": 0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.buffer = Buffer(cache_time=Duration(seconds=60))
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(LaserScan, self.p["output_topic"],
            QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
        # The candidate engine computes TF unchanged, on a private bus. Feed
        # its odom inputs through, but publish map->odom globally only AFTER
        # the application has verified the loaded graph and initial poses.
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.internal_tf = self.create_publisher(TFMessage,"/slam_toolbox/internal_tf",tf_qos)
        self.public_tf = self.create_publisher(TFMessage,"/tf",tf_qos)
        self.tf_input = self.create_subscription(TFMessage,"/tf",self.forward_odometry,tf_qos)
        self.tf_output = self.create_subscription(TFMessage,"/slam_toolbox/internal_tf",self.forward_map,tf_qos)
        self.frame_ready = not self.p["map_file_name"]
        self.held_transform = None
        self.tf_history = deque(maxlen=200)
        self.release_service = self.create_service(Trigger,"/slam_scan_gate/release_frame",self.release_frame)
        self.subscription = self.create_subscription(LaserScan, self.p["scan_topic"], self.scan, qos_profile_sensor_data)
        self.client = self.create_client(DeserializePoseGraph, "/slam_toolbox/deserialize_map")
        self.pending = []
        self.planarity = PlanarScanGuard(self.p["max_scan_tilt_deg"], self.p["max_scan_height_change_m"])
        self.nonplanar_count = self.missing_tf_count = 0
        self.future = None
        self.priming = False
        self.anchor_scan = None
        self.last_prime_send = None
        self.prime_attempts = 0
        self.ready = not self.p["map_file_name"]
        self.failed = False
        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped,"/pose",self.prime,20)
        self.timer = self.create_timer(.02, self.tick)
        if self.ready:
            self.set_state("ready")

    def map_transform(self,t):
        return t.header.frame_id==self.p["map_frame"] and t.child_frame_id==self.p["odom_frame"]

    def forward_odometry(self,msg):
        transforms=[t for t in msg.transforms if not self.map_transform(t)]
        if transforms:
            forwarded=TFMessage(transforms=transforms)
            self.tf_history.append(forwarded)
            self.internal_tf.publish(forwarded)

    def forward_map(self,msg):
        transforms=[t for t in msg.transforms if self.map_transform(t)]
        if transforms:
            self.held_transform=TFMessage(transforms=transforms)
            if self.frame_ready:
                self.public_tf.publish(self.held_transform)

    def release_frame(self,request,response):
        if not self.ready or self.failed:
            response.success=False
            response.message="mapping initialization has not completed"
        else:
            self.frame_ready=True
            if self.held_transform:
                self.public_tf.publish(self.held_transform)
            response.success=True
            response.message="verified map frame released"
        return response

    def set_state(self, state):
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter("bootstrap_state", value=state)])

    def scan(self, msg):
        if self.failed:
            return
        # Always queue briefly: TF for this exact acquisition time often
        # arrives after the scan callback. Never substitute a latest transform.
        self.pending.append(msg)
        if len(self.pending) > 200:
            self.failed = True
            self.set_state("failed: scan buffer exceeded")
            self.get_logger().error("Scan/TF backlog exceeded 200 observations")

    def filter_pending(self):
        from rclpy.parameter import Parameter
        while self.pending:
            scan = self.pending[0]
            stamp = Time.from_msg(scan.header.stamp)
            try:
                transform = self.buffer.lookup_transform(self.p["odom_frame"],scan.header.frame_id,stamp)
            except Exception:
                if self.get_clock().now().nanoseconds-stamp.nanoseconds <= 500000000:
                    return False
                self.pending.pop(0)
                self.missing_tf_count += 1
                self.set_parameters([Parameter("dropped_missing_tf_scans",value=self.missing_tf_count)])
                continue
            q = transform.transform.rotation
            reason = self.planarity.check(transform.transform.translation.z,(q.x,q.y,q.z,q.w))
            if reason is None:
                return True
            self.pending.pop(0)
            self.nonplanar_count += 1
            self.set_parameters([Parameter("dropped_nonplanar_scans",value=self.nonplanar_count)])
            if self.nonplanar_count == 1 or self.nonplanar_count % 50 == 0:
                self.get_logger().warning(f"Rejected nonplanar 2D scan: {reason}; total={self.nonplanar_count}")
        return False

    def tick(self):
        if self.priming:
            self.try_prime()
            return
        # During deserialization/priming, keep the chosen anchor immutable.
        if (self.future is None or self.ready) and not self.filter_pending():
            return
        self.try_load()
        if self.ready and self.pending and not self.failed:
            self.publisher.publish(self.pending.pop(0))

    def prime(self,msg):
        if self.priming and not self.failed and msg.header.stamp==self.anchor_scan.header.stamp:
            self.priming=False
            self.ready=True
            self.set_state("ready")

    def try_prime(self):
        # A ready deserialization service does not imply DDS discovery of the
        # scan reader and pose writer has finished. Volatile one-shot delivery
        # can lose the only anchor and leave the mapping swap waiting forever.
        if not self.priming or self.failed or self.ready:
            return
        if self.publisher.get_subscription_count()<1 or self.count_publishers(self.pose_subscription.topic_name)<1:
            return
        now=time.monotonic()
        if self.last_prime_send is not None and now-self.last_prime_send<.25:
            return
        at=self.anchor_scan.header.stamp.sec*1000000000+self.anchor_scan.header.stamp.nanosec
        for message in self.tf_history:
            nearby=[t for t in message.transforms if abs(t.header.stamp.sec*1000000000+t.header.stamp.nanosec-at)<=1000000000]
            if nearby:self.internal_tf.publish(TFMessage(transforms=nearby))
        # Retry the original observation, never invent a new acquisition stamp
        # or move the verified seed while awaiting its native acknowledgment.
        self.publisher.publish(self.anchor_scan)
        self.last_prime_send=now
        self.prime_attempts+=1
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter("bootstrap_prime_attempts",value=self.prime_attempts)])

    def try_load(self):
        if self.ready or self.failed or self.future is not None or not self.pending or not self.client.service_is_ready():
            return
        first = self.pending[0]
        try:
            odom = tuple_from_transform(self.buffer.lookup_transform(
                self.p["odom_frame"], self.p["base_frame"], Time.from_msg(first.header.stamp)))
            if self.p["sensor_signature"]:
                expected = json.loads(base64.b64decode(self.p["sensor_signature"]))
                mount = self.buffer.lookup_transform(self.p["base_frame"],first.header.frame_id,Time())
                t,q = mount.transform.translation,mount.transform.rotation
                actual_t, actual_q = (t.x,t.y,t.z),(q.x,q.y,q.z,q.w)
                bad = (first.header.frame_id != expected["frame_id"] or len(first.ranges) != expected["count"]
                    or abs(first.angle_min-expected["angle_min"])>1e-6
                    or abs(first.angle_increment-expected["angle_increment"])>1e-7
                    or math.dist(actual_t,expected["translation"])>.01
                    or min(math.dist(actual_q,expected["rotation"]),math.dist([-v for v in actual_q],expected["rotation"]))>.01)
                if bad:
                    self.failed = True
                    self.set_state("failed: laser geometry or calibration differs from the saved graph")
                    self.get_logger().error("Laser geometry/calibration mismatch; no mapping scans were released")
                    return
        except Exception:
            # Discard only BEFORE choosing a first scan, while TF discovery
            # catches up. Never drop the anchor scan once loading has begun.
            self.pending = self.pending[-10:]
            return
        seed = tuple(self.p[key] for key in ("start_x", "start_y", "start_yaw"))
        anchor = tuple(self.p[key] for key in ("anchor_odom_x", "anchor_odom_y", "anchor_odom_yaw"))
        pose = advance(seed, anchor, odom)
        if not all(math.isfinite(v) for v in pose):
            self.failed = True
            self.set_state("failed: invalid start pose")
            return
        req = DeserializePoseGraph.Request(filename=self.p["map_file_name"])
        req.match_type = req.START_AT_GIVEN_POSE
        req.initial_pose.x, req.initial_pose.y, req.initial_pose.theta = pose
        self.set_state("loading")
        self.future = self.client.call_async(req)
        self.future.add_done_callback(self.loaded)

    def loaded(self, future):
        try:
            future.result()
            if self.failed:
                return
            # Service response has no success field in 2.6.10. The source graph
            # is cold-validated before launch; post-handover laser verification
            # remains mandatory and failure tears down this entire launch.
            # Protect the anchor scan from a burst overflowing the engine's
            # input queue. Wait for its exact native pose acknowledgment,
            # then drain queued scans at 50 Hz while live scans keep arriving.
            self.anchor_scan=self.pending.pop(0)
            self.priming=True
            self.set_state("priming")
            self.try_prime()
        except Exception as exc:
            self.failed = True
            self.set_state("failed: "+str(exc))


def main():
    rclpy.init()
    node = ScanGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
