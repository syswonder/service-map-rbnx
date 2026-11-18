#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Run one isolated ROS replay; requires sourced ROS Humble and an unused domain.

Native pose messages measure corrections. TF republishing is never counted as
a new laser update. Latency is publisher-to-result software latency, not lidar
hardware latency. Truth is used only here for initialization and scoring.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
import yaml

ROOT = Path(__file__).resolve().parents[2]


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Replay(Node):
    def __init__(self, a, meta, frames):
        super().__init__('maintenance_replay')
        self.a, self.meta, self.frames = a, meta, frames
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.clock_pub = self.create_publisher(Clock, '/clock', 20)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'base_link', 'laser'
        mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = meta['sensor_translation']
        q = meta['sensor_quaternion_wxyz']
        mount.transform.rotation.w, mount.transform.rotation.x, mount.transform.rotation.y, mount.transform.rotation.z = q
        self.static_tf.sendTransform(mount)
        topic = '/pose' if a.backend == 'slam_toolbox' else '/amcl_pose'
        self.sub = self.create_subscription(PoseWithCovarianceStamped, topic, self.result, 100)
        self.sent = {}
        self.results = {}
        self.last_result = None
        self.records = []
        self.start_wall = None
        self.process = None
        self.resources = []
        self.odom_inputs = {}
        self.authorities = set()
        self.authority_nodes = set()
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.check_authority, 100)

    def check_authority(self, msg):
        if any(t.header.frame_id == 'map' and t.child_frame_id == 'odom' for t in msg.transforms):
            # Humble rclpy does not expose MessageInfo in subscriptions.
            # In this isolated domain our injector publishes only odom->base;
            # require exactly one other /tf endpoint and actual map->odom data.
            for endpoint in self.get_publishers_info_by_topic('/tf'):
                if endpoint.node_name != self.get_name():
                    self.authorities.add(bytes(endpoint.endpoint_gid).hex())
                    self.authority_nodes.add(endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name)

    def resource_sample(self, roots):
        # Aggregate backend, map server and lifecycle manager process tree.
        ticks, rss = 0, 0
        todo = list(roots)
        while todo:
            pid = todo.pop()
            try:
                raw = Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()
                ticks += int(raw[11])+int(raw[12])
                rss += int(raw[21])*os.sysconf('SC_PAGE_SIZE')
                todo.extend(int(v) for v in Path(f'/proc/{pid}/task/{pid}/children').read_text().split())
            except (FileNotFoundError,ProcessLookupError):
                pass
        self.resources.append(dict(wall=time.monotonic(),cpu_seconds=ticks/os.sysconf('SC_CLK_TCK'),rss_mb=rss/1024**2))

    def result(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        index = round((stamp-1000)*10)
        if index not in self.sent or index in self.results:
            return
        p = msg.pose.pose
        pose = [p.position.x,p.position.y,yaw(p.orientation)]
        result = dict(index=index, t=self.frames[index]['t'], estimate=pose,
                      latency_ms=1000*(time.monotonic()-self.sent[index]))
        self.results[index] = result
        if self.last_result is None or index >= self.last_result['index']:
            self.last_result = result

    def publish(self, index, scan=True):
        f = self.frames[index]
        at = 1000+f['t']
        stamp = Clock().clock
        stamp.sec, stamp.nanosec = int(at), round((at-int(at))*1e9)
        self.clock_pub.publish(Clock(clock=stamp))
        x,y,heading = f['truth']
        if self.a.odom_reset:
            x0,y0,theta0=self.frames[0]['truth']
            dx,dy=x-x0,y-y0
            x,y=math.cos(theta0)*dx+math.sin(theta0)*dy,-math.sin(theta0)*dx+math.cos(theta0)*dy
            heading -= theta0
        drift = 0.0
        if self.a.fault == 'drift':
            drift = math.radians(50)/960*f['t']
        if self.a.fault == 'jump' and f['t'] >= self.a.fault_at:
            drift = math.radians(getattr(self.a,'jump_yaw_deg',20.))
            x += getattr(self.a,'jump_x',.5)
        # Same perturbed motion in /odom and TF; truth is untouched.
        heading += drift
        self.odom_inputs[index] = [x,y,heading]
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp,'odom','base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x,y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        self.odom_pub.publish(odom)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, 'base_link'
        tf.transform.translation.x,tf.transform.translation.y = x,y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)
        if scan:
            msg = LaserScan()
            msg.header.stamp, msg.header.frame_id = stamp,'laser'
            msg.angle_min, msg.angle_increment = -math.pi, 2*math.pi/len(f['ranges'])
            msg.angle_max = math.pi-msg.angle_increment
            msg.range_min, msg.range_max, msg.scan_time = .1,40.,.1
            msg.time_increment = 0.0  # upstream rays are simultaneous
            msg.ranges = [float(v) if v is not None else math.inf for v in f['ranges']]
            self.sent[index] = time.monotonic()
            self.scan_pub.publish(msg)

    def spin_for(self, seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=min(.01, max(0,until-time.monotonic())))

    def service(self, kind, name, request):
        c = self.create_client(kind, name)
        try:
            if not c.wait_for_service(timeout_sec=15):
                raise RuntimeError(f'{name} unavailable')
            future = c.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30)
            if not future.done():
                raise RuntimeError(f'{name} timeout')
            return future.result()
        finally:
            self.destroy_client(c)
