#!/usr/bin/env bash
# Snapshot current /robonix/map/* into ~/wheatfox/maps/<tag>/.
# Folder layout encodes time + algo so you can compare runs later:
#   ~/wheatfox/maps/YYYYMMDD_HHMMSS_<algo>_<sensor>[_<note>]/
#     cloud.pcd           — accumulated cloud (binary PCD)
#     grid.pgm/yaml/png   — occupancy grid (nav2 convention + png render)
#     dlio_params.yaml    — copy of DLIO params at save time (provenance)
#     meta.json           — timestamp, pose, point count, accumulator env
#
# Usage:   save_map.sh [note]        (note is an optional free-form suffix)
# Example: save_map.sh lab_floor1
set -o pipefail

NOTE="${1:-}"
TS=$(date +%Y%m%d_%H%M%S)
ALGO=dlio
SENSOR=mid360
TAG="${TS}_${ALGO}_${SENSOR}"
[[ -n "$NOTE" ]] && TAG="${TAG}_${NOTE}"

OUT="$HOME/wheatfox/maps/$TAG"
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY CYCLONEDDS_URI

# Grab latched snapshots
timeout 15 python3 /home/syswonder/wheatfox/packages/mapping_rbnx/scripts/save_map.py "$OUT/cloud"
# save_map.py writes <prefix>.pcd (cloud) and <prefix>.{pgm,yaml,png} (grid).
# Rename grid files: cloud.{pgm,yaml,png} -> grid.*
for ext in pgm yaml png; do
  [[ -f "$OUT/cloud.$ext" ]] && mv "$OUT/cloud.$ext" "$OUT/grid.$ext"
done
[[ -f "$OUT/grid.yaml" ]] && sed -i "s|^image: cloud.pgm|image: grid.pgm|" "$OUT/grid.yaml"

cp /home/syswonder/wheatfox/packages/slam_alt_ws/install/direct_lidar_inertial_odometry/share/direct_lidar_inertial_odometry/cfg/params.yaml \
   "$OUT/dlio_params.yaml" 2>/dev/null || true

python3 - "$OUT" "$ALGO" "$SENSOR" "$NOTE" << "PY"
import json, os, sys, time, subprocess
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

def accumulator_env():
    # Read env of live cloud_accumulator.py — pgrep -f matches parent
    # shells too, so scan all candidates and pick the one that actually
    # exports the var.
    try:
        pids = subprocess.check_output(
            ["pgrep", "-f", "cloud_accumulator.py"]).decode().split()
    except Exception:
        return {}
    wanted = {"CLOUD_ACC_VOXEL", "CLOUD_ACC_MIN_HITS", "CLOUD_ACC_DECAY_FRAMES",
              "CLOUD_ACC_DECAY_RANGE_M", "CLOUD_ACC_Z_MIN", "CLOUD_ACC_Z_MAX",
              "MAPPING_CLOUD_TOPIC", "MAPPING_ODOM_TOPIC", "MAPPING_OUTPUT_FRAME"}
    out = {}
    for pid in pids:
        try:
            with open(f"/proc/{pid}/environ", "rb") as f:
                for kv in f.read().split(b"\0"):
                    if not kv: continue
                    k, _, v = kv.decode(errors="replace").partition("=")
                    if k in wanted and k not in out:
                        out[k] = v
        except Exception:
            continue
    return out

out, algo, sensor, note = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]

rclpy.init()
n = rclpy.create_node("meta_grabber")
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
state = {"pose": None, "cloud_n": None}
def odom_cb(m):
    p = m.pose.pose.position; q = m.pose.pose.orientation
    state["pose"] = {"x": p.x, "y": p.y, "z": p.z,
                     "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
def cloud_cb(m):
    state["cloud_n"] = m.width * m.height
n.create_subscription(Odometry, "/dlio/odom_node/odom", odom_cb, 10)
n.create_subscription(PointCloud2, "/robonix/map/cloud_accumulated", cloud_cb, latched)
t0 = time.time()
while time.time() - t0 < 3.0 and (state["pose"] is None or state["cloud_n"] is None):
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node()
try: rclpy.shutdown()
except Exception: pass

meta = {
  "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
  "algorithm": algo,
  "algorithm_version": "DLIO v1.1.1 (enkerewpo/feature/ros2 fork)",
  "sensor": sensor,
  "sensor_detail": "Livox MID360 (lidar + built-in IMU)",
  "imu_topic_used": "/livox/imu_si (G->m/s^2 shim from /livox/imu)",
  "input_cloud": "/dlio/odom_node/pointcloud/deskewed",
  "note": note,
  "pose_at_save": state["pose"],
  "accumulated_points": state["cloud_n"],
  "accumulator_env": accumulator_env(),
}
with open(os.path.join(out, "meta.json"), "w") as f:
    json.dump(meta, f, indent=2)
print(f"[save_map] wrote {out}/meta.json")
PY

echo
echo "saved -> $OUT"
ls -la "$OUT"
