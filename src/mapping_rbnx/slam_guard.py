# SPDX-License-Identifier: MulanPSL-2.0
"""Conservative mapping guard: require review after a large odom innovation.

A large loop correction may be correct. This guard withdraws mapping rather
than automatically deciding which hypothesis is right or accepting a bad map.
"""
import math
import os
from .slam_motion import advance


class PoseJumpGuard:
    def __init__(self, max_translation=.5, max_yaw=math.radians(15)):
        self.max_translation, self.max_yaw = max_translation, max_yaw
        self.previous = None

    def update(self, stamp, pose, odom):
        if not all(math.isfinite(float(v)) for v in (*pose,*odom)):
            return "nonfinite mapping pose or odometry"
        previous = self.previous
        if previous and stamp <= previous[0]:
            return None
        self.previous = (stamp,pose,odom)
        if previous is None:
            return None
        expected = advance(previous[1],previous[2],odom)
        distance = math.hypot(pose[0]-expected[0],pose[1]-expected[1])
        yaw = abs(math.atan2(math.sin(pose[2]-expected[2]),math.cos(pose[2]-expected[2])))
        if distance > self.max_translation or yaw > self.max_yaw:
            return f"mapping correction exceeds review threshold ({distance:.2f} m, {math.degrees(yaw):.1f} deg)"
        return None


class OdomJumpGuard:
    def __init__(self,max_speed=2.,max_yaw_rate=3.):
        if not all(math.isfinite(v) and v>0 for v in (max_speed,max_yaw_rate)):
            raise ValueError("odometry speed limits must be finite and positive")
        self.max_speed,self.max_yaw_rate=max_speed,max_yaw_rate
        self.previous=None

    def update(self,stamp,pose):
        if not all(math.isfinite(v) for v in (stamp,*pose)):
            return "nonfinite odometry during mapping"
        previous=self.previous
        self.previous=(stamp,pose)
        if previous is None or stamp==previous[0]:return None
        dt=stamp-previous[0]
        if dt<0:return "odometry time moved backwards during mapping"
        speed=math.hypot(pose[0]-previous[1][0],pose[1]-previous[1][1])/dt
        yaw=abs(math.atan2(math.sin(pose[2]-previous[1][2]),math.cos(pose[2]-previous[1][2])))/dt
        if speed>self.max_speed or yaw>self.max_yaw_rate:
            return f"odometry reset or implausible motion ({speed:.2f} m/s, {yaw:.2f} rad/s)"
        return None


class RuntimeGuard:
    """Observe native SLAM poses, pairing each with odom at its scan time."""
    def __init__(self, node, session, ops):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        from rclpy.duration import Duration
        self.buffer = Buffer(cache_time=Duration(seconds=60))
        self.listener = TransformListener(self.buffer, node, spin_thread=False)
        self.session, self.ops = session, ops
        self.latest = None
        self.latest_scan = None
        self.session_id = None
        self.guard = PoseJumpGuard()
        self.odom_guard = self.new_odom_guard()
        self.odom_fault = None
        self.odom_session_id = None
        self.subscription = node.create_subscription(PoseWithCovarianceStamped, "/pose", self.observe, 20)
        self.scan_subscription = node.create_subscription(LaserScan, ops._scan_topic,
            lambda msg: setattr(self,"latest_scan",msg), qos_profile_sensor_data)
        self.odom_subscription = node.create_subscription(Odometry,
            os.environ.get("MAPPING_ODOM_TOPIC","/odom"),self.observe_odom,50)

    @staticmethod
    def new_odom_guard():
        return OdomJumpGuard(float(os.environ.get("MAPPING_MAX_ODOM_SPEED_MPS","2")),
                             float(os.environ.get("MAPPING_MAX_ODOM_YAW_RATE","3")))

    def observe_odom(self,msg):
        state = self.session.status()
        if state["state"] not in {"mapping","saving"}:
            self.odom_guard = self.new_odom_guard()
            self.odom_fault = None
            return
        if state["session_id"] != self.odom_session_id:
            self.odom_session_id = state["session_id"]
            self.odom_guard = self.new_odom_guard()
            self.odom_fault = None
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        fault = self.odom_guard.update(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,pose)
        self.odom_fault = self.odom_fault or fault

    def sensor_metadata(self):
        from rclpy.time import Time
        scan = self.latest_scan
        if scan is None:
            raise RuntimeError("laser metadata unavailable")
        transform = self.buffer.lookup_transform(self.ops._base_frame,scan.header.frame_id,Time())
        t,q = transform.transform.translation,transform.transform.rotation
        return {"frame_id": scan.header.frame_id, "count": len(scan.ranges),
                "angle_min": scan.angle_min, "angle_increment": scan.angle_increment,
                "range_min": scan.range_min, "range_max": scan.range_max,
                "translation": [t.x,t.y,t.z], "rotation": [q.x,q.y,q.z,q.w]}

    def observe(self, msg):
        import time
        self.latest = (time.time(), msg)

    def __call__(self):
        from rclpy.time import Time
        from .slam_motion import tuple_from_transform
        state = self.session.status()
        if self.odom_fault and self.odom_session_id == state["session_id"]:
            return self.odom_fault
        if state["session_id"] != self.session_id:
            self.session_id, self.guard = state["session_id"], PoseJumpGuard()
            anchor = self.session._metadata.get("guard_anchor")
            if anchor:
                self.guard.update(anchor["stamp"],anchor["pose"],anchor["odom"])
        if self.latest is None or self.latest[0] < state.get("started_at", 0):
            return None
        msg = self.latest[1]
        stamp = Time.from_msg(msg.header.stamp)
        if not self.buffer.can_transform(self.ops._odom_frame, self.ops._base_frame, stamp):
            return None
        odom = tuple_from_transform(self.buffer.lookup_transform(self.ops._odom_frame, self.ops._base_frame, stamp))
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        return self.guard.update(stamp.nanoseconds,pose,odom)
