#!/usr/bin/env python3
# Log-odds 2D OccupancyGrid from fastlio2 world_cloud + lio_odom.
# Each observation updates p(occ) per-cell so transients recover.
import numpy as np, os, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry


L_OCC = 0.85          # hit log-odds increment
L_FREE = -0.40        # miss (ray pass-through) log-odds decrement
L_MIN, L_MAX = -5.0, 5.0    # clamp (allow recovery)
L_OCC_THR = 0.70      # > threshold => occupied (100)
L_FREE_THR = -0.40    # < threshold => free (0)


class PC2Grid(Node):
    def __init__(self):
        super().__init__('pointcloud_to_grid')
        self.res = float(os.environ.get("PC2GRID_RES", "0.1"))
        self.size_m = float(os.environ.get("PC2GRID_SIZE_M", "200.0"))
        self.zmin = -0.2
        self.zmax = 0.8
        self.min_range = 0.3    # robot footprint / lidar blind zone only
        self.frame = 'lidar'
        self.max_range_m = self.size_m / 2.0 - self.res
        self.n = int(self.size_m / self.res)
        self.half = self.size_m / 2.0
        self.log = np.zeros((self.n, self.n), dtype=np.float32)
        self.sensor_xy = np.array([0.0, 0.0], dtype=np.float32)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/robonix/map/occupancy_grid', latched)
        self.create_subscription(PointCloud2, '/fastlio2/world_cloud', self.cb, 10)
        self.create_subscription(Odometry, '/fastlio2/lio_odom', self.odom_cb, 50)
        self.create_timer(0.5, self.publish)
        self.get_logger().info(
            f'PC2Grid log-odds: {self.n}x{self.n} z=[{self.zmin},{self.zmax}] '
            f'L_OCC={L_OCC} L_FREE={L_FREE}')

    def odom_cb(self, m):
        self.sensor_xy = np.array([m.pose.pose.position.x,
                                    m.pose.pose.position.y], dtype=np.float32)

    def cb(self, msg):
        offs = {f.name: f.offset for f in msg.fields}
        ox, oy, oz = offs['x'], offs['y'], offs['z']
        ps = msg.point_step
        n_pts = msg.width * msg.height
        if n_pts == 0: return
        arr = np.frombuffer(msg.data, dtype=np.uint8)[:n_pts*ps].reshape(n_pts, ps)
        x = arr[:, ox:ox+4].copy().view(np.float32).ravel()
        y = arr[:, oy:oy+4].copy().view(np.float32).ravel()
        z = arr[:, oz:oz+4].copy().view(np.float32).ravel()
        m = (z >= self.zmin) & (z <= self.zmax) & np.isfinite(x) & np.isfinite(y)
        x = x[m]; y = y[m]
        if x.size == 0: return

        sx, sy = float(self.sensor_xy[0]), float(self.sensor_xy[1])
        # drop points too close to sensor (operator/robot body)
        dxr = x - sx; dyr = y - sy
        rmask = np.hypot(dxr, dyr) >= self.min_range
        x = x[rmask]; y = y[rmask]
        if x.size == 0: return
        # world -> cell indices
        def w2c(wx, wy):
            cx = np.floor((wx + self.half) / self.res).astype(np.int32)
            cy = np.floor((wy + self.half) / self.res).astype(np.int32)
            return cx, cy

        hx, hy = w2c(x, y)
        ok = (hx >= 0) & (hx < self.n) & (hy >= 0) & (hy < self.n)
        hx = hx[ok]; hy = hy[ok]
        xw = x[ok]; yw = y[ok]
        if hx.size == 0: return

        # --- MISS pass: raytrace from sensor to each hit (stopping BEFORE the hit cell) ---
        # Sample each ray at resolution step; we cap max_samples to keep budget bounded.
        dx = xw - sx; dy = yw - sy
        rng = np.hypot(dx, dy)
        # for each ray, number of miss samples = int(rng/res) - 1 (exclude endpoint)
        max_steps = max(1, int(np.ceil(rng.max() / self.res)) - 1)
        # vectorize: t in [res/rng, (rng-res)/rng] → sample positions
        ts = (np.arange(max_steps, dtype=np.float32) + 1.0) * self.res
        # For each ray, mark cells at distance ts[k] < rng[i]
        # Compute all sample positions: (N, max_steps)
        sample_x = sx + (dx[:, None] / np.maximum(rng[:, None], 1e-6)) * ts[None, :]
        sample_y = sy + (dy[:, None] / np.maximum(rng[:, None], 1e-6)) * ts[None, :]
        valid = ts[None, :] < (rng[:, None] - self.res)    # exclude endpoint cell
        mx, my = w2c(sample_x, sample_y)
        vmask = valid & (mx >= 0) & (mx < self.n) & (my >= 0) & (my < self.n)
        miss_cells_idx = my[vmask] * self.n + mx[vmask]
        # Dedup miss cells per frame (avoid over-weighting when many rays share a cell)
        miss_cells_idx = np.unique(miss_cells_idx)
        np.add.at(self.log.ravel(), miss_cells_idx, L_FREE)

        # --- HIT pass: mark every endpoint cell as occupied ---
        hit_idx = np.unique(hy * self.n + hx)
        np.add.at(self.log.ravel(), hit_idx, L_OCC)

        # clamp
        np.clip(self.log, L_MIN, L_MAX, out=self.log)

    def publish(self):
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = self.frame
        g.info.resolution = float(self.res)
        g.info.width = self.n
        g.info.height = self.n
        g.info.origin.position.x = -self.half
        g.info.origin.position.y = -self.half
        g.info.origin.orientation.w = 1.0
        occ = np.full(self.log.shape, -1, dtype=np.int8)
        occ[self.log <= L_FREE_THR] = 0
        occ[self.log >= L_OCC_THR] = 100
        g.data = occ.flatten().tolist()
        self.pub.publish(g)


def main():
    rclpy.init()
    node = PC2Grid()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        try: node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__': main()
