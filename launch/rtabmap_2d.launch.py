# SPDX-License-Identifier: MulanPSL-2.0
"""RTAB-Map 2D — lidar + RGBD fusion with loop closure.

Every input topic is taken as a launch argument so atlas_bridge's
resolved.yaml drives the wiring. The launch file itself contains no
hardcoded sensor topic names — that's the only way one mapping package
serves both webots (`/head_front_camera/...`) and a real robot
(`/realsense/...`) without a code change. start_engine.sh reads each
contract from /tmp/<algo>_resolved.yaml and passes its endpoint here.

Launch args (with sentinels for "not provided"):
  scan_topic       LaserScan (lidar primitive's lidar/lidar contract)
  rgb_topic        Image — sentinel `<none>` disables RGB subscription
  rgb_info_topic   CameraInfo — paired with rgb_topic
  depth_topic      Image (depth) — sentinel `<none>` disables depth
  odom_topic       Odometry (chassis odom contract)
  use_sim_time, enable_viz: standard

When rgb_topic or depth_topic is `<none>` the corresponding
subscription is turned off automatically (lidar-only mode is the
fallback when the deploy has no RGBD camera).

Outputs (declared on atlas by atlas_bridge — see _ALGO_TOPIC_BINDINGS):
  /map                 nav_msgs/OccupancyGrid (2D, lidar + depth proj)
  /rtabmap/cloud_map   sensor_msgs/PointCloud2 (3D fused cloud)
  /tf                  map→odom transform
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_NONE = "<none>"  # sentinel for "no such topic in this deploy"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    scan_topic = LaunchConfiguration("scan_topic", default="/scan")
    odom_topic = LaunchConfiguration("odom_topic", default="/odom")
    rgb_topic = LaunchConfiguration("rgb_topic", default=_NONE)
    rgb_info_topic = LaunchConfiguration("rgb_info_topic", default=_NONE)
    depth_topic = LaunchConfiguration("depth_topic", default=_NONE)
    enable_viz = LaunchConfiguration("enable_viz", default="false")

    # Resolve substitutions to plain strings so we can branch on them
    # at launch-description build time. We have to do this lazily via
    # OpaqueFunction because LaunchConfiguration is not a string until
    # the launch system evaluates it.
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("rgb_topic", default_value=_NONE),
        DeclareLaunchArgument("rgb_info_topic", default_value=_NONE),
        DeclareLaunchArgument("depth_topic", default_value=_NONE),
        DeclareLaunchArgument("enable_viz", default_value="false"),
        OpaqueFunction(function=_make_nodes),
    ])


def _make_nodes(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    rgb_topic = LaunchConfiguration("rgb_topic").perform(context)
    rgb_info_topic = LaunchConfiguration("rgb_info_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    enable_viz = LaunchConfiguration("enable_viz").perform(context).lower() == "true"

    have_rgb = rgb_topic and rgb_topic != _NONE
    have_depth = depth_topic and depth_topic != _NONE
    have_rgbd = have_rgb and have_depth

    rtabmap_params = {
        "use_sim_time": use_sim_time.lower() == "true",
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "publish_tf": True,
        # Subscribe to whichever sensors the deploy actually has. RGBD
        # fusion gives table-top occupancy that lidar alone misses;
        # lidar-only mode still produces a valid 2D map (just no
        # below-plane obstacles).
        "subscribe_scan": True,
        "subscribe_rgbd": False,
        "subscribe_rgb": have_rgbd,
        "subscribe_depth": have_rgbd,
        "approx_sync": True,        # webots topics not perfectly synced
        "queue_size": 30,
        # webots emits image stamps slightly ahead of the dynamic TF
        # for the camera chain (head_2_link → Astra → ...), causing
        # "extrapolation into the future" + "TF of received image is
        # not set" errors when wait_for_transform is short. 1.5s gives
        # the TF buffer plenty of room to catch up.
        "wait_for_transform": 1.5,
        # Build occupancy grid from BOTH scan and depth — depth fills in
        # obstacles below the lidar plane (tables, chairs) that the 2D
        # scan misses entirely. RTAB-Map 0.21+: scan goes into grid via
        # subscribe_scan, depth via Grid/FromDepth + Grid/Sensor=1.
        "Grid/Sensor": "1",
        "Grid/FromDepth": "true",
        "Grid/RangeMax": "8.0",
        "Grid/CellSize": "0.05",
        "Grid/RayTracing": "true",
        "Grid/3D": "false",
        "Grid/NormalsSegmentation": "false",
        "Grid/MaxObstacleHeight": "1.5",
        "Grid/MaxGroundHeight": "0.05",
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "false",
        "Reg/Strategy": "1",        # 0=Visual, 1=ICP, 2=Visual+ICP
        "Reg/Force3DoF": "true",
        "Optimizer/Strategy": "1",  # g2o
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "Vis/MinInliers": "12",
        # Default DetectionRate is 1Hz — WAY too slow for tiago at
        # 0.4m/s. Between updates the robot drifts ~40cm uncorrected on
        # webots wheel odom (which has small but accumulating slip).
        # 5Hz cuts inter-frame drift to ~8cm, well within ICP capture
        # range, so scan-matching keeps the map locked.
        "Rtabmap/DetectionRate": "5.0",
        # Loosen scan-matching tolerance: with 5Hz frames the relative
        # motion is small enough that 20cm correspondence is generous,
        # not noisy. Default 0.1 was tuned for 1Hz.
        "Icp/MaxCorrespondenceDistance": "0.2",
        # When scan ICP and odom disagree, prefer the scan correction
        # over odom (default lets odom dominate when ICP is weak).
        "Icp/MaxTranslation": "0.5",
        "Icp/MaxRotation": "0.78",  # ~45°
    }

    rtabmap_remappings = [
        ("scan", scan_topic),
        ("odom", odom_topic),
        # rviz "2D Pose Estimate" → /initialpose: rtabmap defaults to
        # the node-relative ~initialpose, remap to global so the rviz
        # tool reaches us without rviz config gymnastics.
        ("initialpose", "/initialpose"),
    ]
    if have_rgbd:
        rtabmap_remappings += [
            ("rgb/image", rgb_topic),
            ("rgb/camera_info", rgb_info_topic if rgb_info_topic != _NONE
                                else _derive_camera_info(rgb_topic)),
            ("depth/image", depth_topic),
        ]

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_params],
        arguments=["--delete_db_on_start"],
        remappings=rtabmap_remappings,
    )

    nodes = [rtabmap_node]

    if enable_viz:
        # rtabmap_viz: GUI subscribes to scan + odom only (RGBD belongs
        # to the main node). Spawned only when enable_viz=true.
        viz = Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            name="rtabmap_viz",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time.lower() == "true",
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "subscribe_scan": True,
                "subscribe_rgb": False,
                "subscribe_depth": False,
                "approx_sync": True,
                "queue_size": 30,
                "wait_for_transform": 1.5,
            }],
            remappings=[("scan", scan_topic), ("odom", odom_topic)],
        )
        nodes.append(viz)

    return nodes


def _derive_camera_info(rgb_topic: str) -> str:
    """When the deploy doesn't tell us a camera_info topic explicitly,
    derive it by ROS convention: replace the leaf with `camera_info`.
    e.g. /head_front_camera/rgb/image_raw → /head_front_camera/rgb/camera_info.
    """
    parts = rgb_topic.rstrip("/").split("/")
    if len(parts) >= 2:
        parts[-1] = "camera_info"
        return "/".join(parts)
    return rgb_topic + "/camera_info"
