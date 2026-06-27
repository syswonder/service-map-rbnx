# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx — RTAB-Map SLAM service image.
#
# Same packaging pattern as system/scene: own image so the package can
# pick its RMW + SLAM toolchain without polluting the sim image.
# --network host + --ipc=host at runtime so DDS/SHM line up with
# whatever sim/robot container is publishing the sensor topics.
#
# Algorithms shipped (selected via manifest config.algo):
#   rtabmap   — default. 2D lidar + RGBD fusion. Fully apt-installable
#               (ros-humble-rtabmap-ros), no extra colcon build.
#   dlio      — Direct LiDAR-Inertial Odometry. Real-robot 3D Livox
#               path; expects the dlio package mounted at /ws/install
#               (separate slam_alt_ws colcon build), so this image
#               stays light.
#   fastlio2  — [BROKEN: drift] same /ws/install path as dlio.
#
# slam_toolbox + cartographer were dropped after rtabmap consistently
# beat them in head-to-head tests on webots tiago (curve-motion
# stability, RGBD fusion, loop closure).
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# ── TUNA mirrors for domestic (CN) networks ─────────────────────────────
# Swap apt sources (Ubuntu/ports + ROS 2) to TUNA. Use `find` + sed
# --follow-symlinks because /etc/apt/sources.list.d/ros2.sources is
# a symlink into /usr/share/ros-apt-source/ in the upstream ros image.
RUN set -eux; \
    find /etc/apt/sources.list.d/ -maxdepth 1 \( -name '*.list' -o -name '*.sources' \) -print0 \
      | xargs -0 -r sed -i --follow-symlinks \
          -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
          -e 's|^Types: deb deb-src$|Types: deb|'; \
    if [ -f /etc/apt/sources.list ]; then \
        sed -i --follow-symlinks \
            -e 's|http://archive\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://security\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://ports\.ubuntu\.com/ubuntu-ports/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/|g' \
            /etc/apt/sources.list; \
    fi
ENV PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    UV_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    HF_ENDPOINT=https://hf-mirror.com

# Runtime deps:
#   rtabmap-ros: SLAM core + rtabmap_viz GUI for dev inspection
#   tf2 / tf-transformations: pose lookups in atlas_bridge
#   pointcloud_to_laserscan: depth → 2D scan adapter (used by dlio
#     fallback paths and any future depth-only deploys)
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-numpy \
        python3-yaml \
        ros-humble-rtabmap-ros \
        ros-humble-tf2-ros \
        ros-humble-tf-transformations \
        ros-humble-pointcloud-to-laserscan \
 && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
        grpcio>=1.60.0 \
        protobuf>=4.25.0 \
        pyyaml>=6.0 \
        numpy \
        mcp>=1.28.1

WORKDIR /mapping
COPY entrypoint.sh /entrypoint.sh
COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml
RUN chmod +x /entrypoint.sh

# Force FastRTPS to skip SHM transport — UDP-only is the only path
# that's actually data-flowing across containers in this layout.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastrtps_no_shm.xml

# Stick with FastRTPS (sim's default RMW). With --network host +
# --ipc=host the SHM transport shares cleanly between sim and mapping,
# and FastRTPS UDP multicast discovery covers the rest. We tried
# Zenoh first but `zenoh_bridge_dds` cannot see FastRTPS-only SHM
# publishers reliably across containers, so this is the path that
# actually delivers sensor topics → rtabmap in webots.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENTRYPOINT ["/entrypoint.sh"]
