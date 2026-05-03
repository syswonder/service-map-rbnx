#!/usr/bin/env python3
# Snapshot /robonix/map/cloud_accumulated (latched) + /robonix/map/occupancy_grid
# (latched). Writes:
#   <out>.pcd   — binary PCD of accumulated cloud (frame = whatever publisher
#                 tagged it; we do NOT transform)
#   <out>.pgm   — nav2 convention (0=occ, 254=free, 205=unknown)
#   <out>.yaml  — nav2 map_server yaml
#   <out>.png   — same image as pgm but png (for papers/reports)
#
# Poll both topics for up to DEADLINE_S seconds then exit explicitly.
# Using spin_once+sys.exit instead of spin() because spin+shutdown-in-cb
# hangs rclpy in Humble.
import os, sys, time, numpy as np, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

DEADLINE_S = 8.0

def write_pcd(path, xyz):
    n = xyz.shape[0]
    hdr = (
        b"# .PCD v0.7 - Point Cloud Data file format\n"
        b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        b"WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
        % (n, n)
    )
    with open(path, "wb") as f:
        f.write(hdr)
        f.write(xyz.astype("<f4").tobytes())

def write_grid(prefix, msg):
    w, h = msg.info.width, msg.info.height
    data = np.array(msg.data, dtype=np.int16).reshape(h, w)
    img = np.full_like(data, 205, dtype=np.uint8)
    img[data == 0] = 254
    img[data >= 50] = 0
    img = np.flipud(img)
    with open(prefix + ".pgm", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.tobytes())
    base = os.path.basename(prefix)
    with open(prefix + ".yaml", "w") as f:
        f.write(
            f"image: {base}.pgm\n"
            f"resolution: {msg.info.resolution}\n"
            f"origin: [{msg.info.origin.position.x}, {msg.info.origin.position.y}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
    try:
        from PIL import Image
        Image.fromarray(img, mode="L").save(prefix + ".png")
    except ImportError:
        pass

def main():
    if len(sys.argv) < 2:
        print("usage: save_map.py <out_prefix>", file=sys.stderr); sys.exit(2)
    prefix = sys.argv[1]
    rclpy.init()
    n = rclpy.create_node("map_snapshotter")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    got = {"cloud": None, "grid": None}
    n.create_subscription(PointCloud2, "/robonix/map/cloud_accumulated",
                          lambda m: got.__setitem__("cloud", m), latched)
    n.create_subscription(OccupancyGrid, "/robonix/map/occupancy_grid",
                          lambda m: got.__setitem__("grid", m), latched)
    t0 = time.time()
    while time.time() - t0 < DEADLINE_S:
        rclpy.spin_once(n, timeout_sec=0.1)
        if got["cloud"] is not None and got["grid"] is not None:
            break
    n.destroy_node()
    try: rclpy.shutdown()
    except Exception: pass

    rc = 0
    if got["cloud"] is not None:
        m = got["cloud"]; ps = m.point_step; npts = m.width * m.height
        offs = {f.name: f.offset for f in m.fields}
        if npts > 0 and "x" in offs:
            arr = np.frombuffer(m.data, dtype=np.uint8)[:npts*ps].reshape(npts, ps)
            x = arr[:, offs["x"]:offs["x"]+4].copy().view(np.float32).ravel()
            y = arr[:, offs["y"]:offs["y"]+4].copy().view(np.float32).ravel()
            z = arr[:, offs["z"]:offs["z"]+4].copy().view(np.float32).ravel()
            write_pcd(prefix + ".pcd", np.stack([x, y, z], axis=1))
            print(f"[save_map] wrote {prefix}.pcd ({npts} pts, frame={m.header.frame_id})")
        else:
            print(f"[save_map] cloud empty (npts={npts})"); rc = 1
    else:
        print("[save_map] no cloud received"); rc = 1

    if got["grid"] is not None:
        write_grid(prefix, got["grid"])
        g = got["grid"]
        print(f"[save_map] wrote {prefix}.pgm/.yaml/.png "
              f"({g.info.width}x{g.info.height} @ {g.info.resolution}m, "
              f"frame={g.header.frame_id})")
    else:
        print("[save_map] no grid received"); rc = 1

    sys.exit(rc)

if __name__ == "__main__":
    main()
