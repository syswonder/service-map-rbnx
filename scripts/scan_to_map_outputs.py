#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Back the pointcloud and odom contracts for engines that only publish a grid.

slam_toolbox owns `/map` and the `map → odom` transform, and nothing else: no 3D
cloud, no corrected-odometry topic. The mapping service's exported surface
promises both, so this adapter fills them from what is already on the wire:

  pointcloud  the current LaserScan lifted into the map frame as a PointCloud2,
              accumulated in a voxel set so consumers see the walls built so far
              rather than one sweep (scene uses it as an occlusion layer)
  odom        the chassis odometry with the SLAM correction applied, i.e. the
              robot's map-frame pose republished as nav_msgs/Odometry, which is
              what `service/map/odom` means for every other engine

Both are cheap: one transform lookup per scan, a dict of voxel keys, no GPU.
"""
from __future__ import annotations

import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped  # noqa: F401  (tf2 typing)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener


def _cloud_msg(stamp, frame_id: str, points: np.ndarray) -> PointCloud2:
    """Pack an (N, 3) float32 array as an unordered PointCloud2."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()
    return msg


class ScanToMapOutputs(Node):
    """Republish scans as a map-frame cloud and the SLAM pose as odometry."""

    def __init__(self, args) -> None:
        super().__init__("scan_to_map_outputs")
        self.map_frame, self.base_frame = args.map_frame, args.base_frame
        self.voxel = max(1e-3, float(args.voxel_size))
        self.max_points = int(args.max_points)
        self._voxels: dict[tuple[int, int, int], None] = {}
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, args.cloud_topic, latched)
        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 10)
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan,
                                 QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1.0 / max(1.0, float(args.odom_rate_hz)), self.publish_odom)
        self.get_logger().info(
            f"scan={args.scan_topic} -> cloud={args.cloud_topic} (voxel {self.voxel} m), "
            f"{self.map_frame}->{self.base_frame} -> odom={args.odom_topic}")

    def _lookup(self, target: str, source: str):
        """Latest transform, or None when tf does not have it yet."""
        try:
            return self._buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:  # noqa: BLE001  (tf2 raises several unrelated types)
            return None

    def on_scan(self, msg: LaserScan) -> None:
        """Lift one scan into the map frame and add it to the voxel set."""
        tf = self._lookup(self.map_frame, msg.header.frame_id)
        if tf is None:
            return
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
        good = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        if not good.any():
            return
        r, a = ranges[good], angles[good]
        local = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
        t, q = tf.transform.translation, tf.transform.rotation
        # Yaw-only rotation: a 2D scan carries no roll/pitch information anyway.
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.stack([
            local[:, 0] * c - local[:, 1] * s + t.x,
            local[:, 0] * s + local[:, 1] * c + t.y,
            local[:, 2] + t.z,
        ], axis=1)
        for key in map(tuple, np.floor(world / self.voxel).astype(np.int64)):
            self._voxels[key] = None
        if len(self._voxels) > self.max_points:  # oldest-first, dict keeps insertion order
            for key in list(self._voxels)[: len(self._voxels) - self.max_points]:
                self._voxels.pop(key, None)
        pts = (np.asarray(list(self._voxels), dtype=np.float64) + 0.5) * self.voxel
        self.cloud_pub.publish(_cloud_msg(msg.header.stamp, self.map_frame, pts))

    def publish_odom(self) -> None:
        """Publish the SLAM-corrected pose as odometry in the map frame."""
        tf = self._lookup(self.map_frame, self.base_frame)
        if tf is None:
            return
        msg = Odometry()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        self.odom_pub.publish(msg)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--cloud-topic", default="/robonix/map/cloud")
    ap.add_argument("--odom-topic", default="/robonix/map/odom")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--voxel-size", type=float, default=0.05)
    ap.add_argument("--max-points", type=int, default=400000)
    ap.add_argument("--odom-rate-hz", type=float, default=10.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = ScanToMapOutputs(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
