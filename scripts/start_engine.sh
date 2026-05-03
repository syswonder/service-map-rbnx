#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# In-container SLAM engine launcher. Picks the algo from MAPPING_ALGO
# (set by atlas_bridge after parsing config) and execs the
# corresponding ros2 launch.
#
# Supported algos:
#   rtabmap   — 2D lidar + RGBD fusion (default; webots and real robots
#               with 2D scan + RGBD camera)
#   dlio      — Direct LiDAR-Inertial Odometry (3D Livox; real robot)
#   fastlio2  — [BROKEN: drift] kept for repro/debug only
#
# Pre-conditions: atlas_bridge has already written
# /tmp/<algo>_resolved.yaml with discovered topic names.
set -eo pipefail

ALGO="${MAPPING_ALGO:-rtabmap}"
RESOLVED="/tmp/${ALGO}_resolved.yaml"

source /opt/ros/humble/setup.bash

read_y() {
    # Never fail under `set -e`: grep returns 1 on no match, which
    # would otherwise abort the script before we even get to the
    # case branch. Default-empty is what every caller wants anyway.
    { grep -E "^$1:" "$RESOLVED" 2>/dev/null || true; } | head -1 | awk '{print $2}' || true
}

case "$ALGO" in
    rtabmap)
        # 2D lidar + RGBD fusion. EVERY input topic comes from the
        # atlas-resolved config; this script does not hardcode any
        # downstream topic name. atlas_bridge writes the resolved
        # endpoints into /tmp/<algo>_resolved.yaml — keys produced by
        # atlas_bridge:_SENSOR_CONTRACTS:
        #   scan_topic   ← robonix/primitive/lidar/lidar
        #   odom_topic   ← robonix/primitive/chassis/odom
        #   rgb_topic    ← robonix/primitive/camera/rgb (optional)
        #   depth_topic  ← robonix/primitive/camera/depth (optional)
        # When camera resolution fails (no rgbd in this deploy), the
        # launch falls back to lidar-only mode automatically via the
        # `<none>` sentinel — no code change needed for headless
        # deploys vs full RGBD deploys.
        SCAN_TOPIC=$(read_y scan_topic)
        ODOM_TOPIC=$(read_y odom_topic)
        RGB_TOPIC=$(read_y rgb_topic)
        DEPTH_TOPIC=$(read_y depth_topic)
        SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
        ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
        RGB_TOPIC="${RGB_TOPIC:-<none>}"
        DEPTH_TOPIC="${DEPTH_TOPIC:-<none>}"
        USE_SIM_TIME="${MAPPING_USE_SIM_TIME:-true}"
        ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"
        echo "[start_engine] rtabmap scan=$SCAN_TOPIC odom=$ODOM_TOPIC rgb=$RGB_TOPIC depth=$DEPTH_TOPIC viz=$ENABLE_VIZ"
        exec ros2 launch /mapping/launch/rtabmap_2d.launch.py \
            scan_topic:="$SCAN_TOPIC" \
            odom_topic:="$ODOM_TOPIC" \
            rgb_topic:="$RGB_TOPIC" \
            depth_topic:="$DEPTH_TOPIC" \
            use_sim_time:="$USE_SIM_TIME" \
            enable_viz:="$ENABLE_VIZ"
        ;;

    dlio)
        # Direct LiDAR-Inertial Odometry — real-robot 3D livox path.
        # Requires the `dlio` ros2 package mounted/installed in the
        # image; for desktop deploys bind-mount the slam_alt_ws colcon
        # build at /ws/install.
        if [ -f /ws/install/setup.bash ]; then
            source /ws/install/setup.bash
        else
            echo "[start_engine] ERR: dlio needs /ws/install (slam_alt_ws colcon build)" >&2
            exit 2
        fi
        echo "[start_engine] dlio (3D Livox + IMU)"
        exec ros2 launch dlio dlio.launch.py
        ;;

    fastlio2)
        # [BROKEN: drift] kept only so the repro path stays reachable.
        # Image must include /ws/install built from FASTLIO2_ROS2
        # (the heavier Dockerfile.fastlio2_full path). Do NOT pick
        # fastlio2 for any production deploy until the global drift
        # issue is fixed.
        if [ -f /ws/install/setup.bash ]; then
            source /ws/install/setup.bash
        fi
        echo "[start_engine] WARN fastlio2 has known drift; use for repro only"
        exec ros2 launch /mapping/launch/slam_mapping_native.launch.py
        ;;

    *)
        echo "[start_engine] unknown algo: $ALGO (supported: rtabmap | dlio | fastlio2)" >&2
        exit 2
        ;;
esac
