#!/usr/bin/env python3
# Voxel-accumulate world_cloud into a persistent downsampled point cloud.
# Publishes /robonix/map/cloud_accumulated at 1 Hz.
import numpy as np, rclpy, struct
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class CloudAcc(Node):
    def __init__(self):
        super().__init__('cloud_accumulator')
        self.voxel = 0.1          # m
        self.frame = 'lidar'
        self.keys = set()          # set of (i,j,k) voxel keys
        self.points = []           # list of (x,y,z) averaged per voxel (just first-hit sample)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PointCloud2, '/robonix/map/cloud_accumulated', latched)
        self.create_subscription(PointCloud2, '/fastlio2/world_cloud', self.cb, 10)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'CloudAcc: voxel={self.voxel}m, publish 1Hz')

    def cb(self, msg):
        offs = {f.name: f.offset for f in msg.fields}
        ox, oy, oz = offs['x'], offs['y'], offs['z']; ps = msg.point_step
        n_pts = msg.width * msg.height
        if n_pts == 0: return
        arr = np.frombuffer(msg.data, dtype=np.uint8)[:n_pts*ps].reshape(n_pts, ps)
        x = arr[:, ox:ox+4].copy().view(np.float32).ravel()
        y = arr[:, oy:oy+4].copy().view(np.float32).ravel()
        z = arr[:, oz:oz+4].copy().view(np.float32).ravel()
        mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x = x[mask]; y = y[mask]; z = z[mask]
        # voxel index
        ix = np.floor(x / self.voxel).astype(np.int32)
        iy = np.floor(y / self.voxel).astype(np.int32)
        iz = np.floor(z / self.voxel).astype(np.int32)
        for i in range(len(x)):
            k = (int(ix[i]), int(iy[i]), int(iz[i]))
            if k in self.keys: continue
            self.keys.add(k)
            self.points.append((float(x[i]), float(y[i]), float(z[i])))
        # cap at 300k voxels
        if len(self.points) > 300000:
            self.points = self.points[-300000:]

    def publish(self):
        if not self.points: return
        pts = np.array(self.points, dtype=np.float32)
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = pts.tobytes()
        msg.is_dense = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    n = CloudAcc()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        try: n.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__': main()
