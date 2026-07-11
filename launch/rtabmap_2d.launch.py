# SPDX-License-Identifier: MulanPSL-2.0
"""RTAB-Map launch — sensor-agnostic, deploy-driven.

The launch file does not assume any sensor combination. It branches on
which input topics the deploy actually wired up via atlas_bridge's
resolved.yaml:

    sensors.lidar2d=true   → subscribe_scan          (LaserScan)
    sensors.lidar3d=true   → subscribe_scan_cloud    (PointCloud2)
    sensors.rgb + .depth   → subscribe_rgb + _depth  (RGB-D fusion)
    sensors.odom=true      → external odom (else rtabmap odometry node)

Webots tiago = lidar2d + rgb + depth + odom (LaserScan + Astra + diff-drive).
Real robot  = lidar3d + rgb + depth + odom + imu (Mid360 + RealSense).

start_engine.sh reads `/tmp/<algo>_resolved.yaml` and passes each topic
as a launch arg. Sentinel `<none>` means "this sensor is not in the
deploy" — the corresponding subscription is disabled.

Launch args:
    scan_topic       LaserScan      (lidar2d)         | <none> = disabled
    scan_cloud_topic PointCloud2    (lidar3d)         | <none> = disabled
    rgb_topic        Image          (camera/rgb)      | <none> = disabled
    rgb_info_topic   CameraInfo     (paired w/ rgb)   | <none> = derive
    depth_topic      Image          (camera/depth)    | <none> = disabled
    odom_topic       Odometry       (chassis/odom)    | <none> = rtabmap
                                                         runs its own
                                                         odometry node
    use_sim_time, enable_viz: standard

Outputs (declared on atlas by atlas_bridge — see _ALGO_TOPIC_BINDINGS):
    /map                 nav_msgs/OccupancyGrid (2D, lidar + depth proj)
    /rtabmap/cloud_map   sensor_msgs/PointCloud2 (3D fused cloud)
    /tf                  map→odom transform
"""
import json
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_NONE = "<none>"  # sentinel for "no such topic in this deploy"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value=_NONE),
        DeclareLaunchArgument("scan_cloud_topic", default_value=_NONE),
        DeclareLaunchArgument("odom_topic", default_value=_NONE),
        DeclareLaunchArgument("rgb_topic", default_value=_NONE),
        DeclareLaunchArgument("rgb_info_topic", default_value=_NONE),
        DeclareLaunchArgument("depth_topic", default_value=_NONE),
        DeclareLaunchArgument("imu_topic", default_value=_NONE),
        DeclareLaunchArgument("deskew_lidar", default_value="false"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("enable_viz", default_value="false"),
        # Map persistence (set by atlas_bridge from the deploy's map_id /
        # map_mode config; empty database_path = ephemeral, the legacy
        # behaviour).
        #   map_mode=mapping       build/extend a map at database_path.
        #   map_mode=localization  load database_path read-only; the map
        #                          frame re-anchors to the saved map so it
        #                          is STABLE across restarts (what scene's
        #                          per-map_id semantic store needs).
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("map_mode", default_value="mapping"),
        DeclareLaunchArgument("reset_map", default_value="false"),
        DeclareLaunchArgument("rtabmap_overrides_file", default_value=""),
        OpaqueFunction(function=_make_nodes),
    ])


def _make_nodes(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    rgb_topic = LaunchConfiguration("rgb_topic").perform(context)
    rgb_info_topic = LaunchConfiguration("rgb_info_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    deskew_lidar = LaunchConfiguration("deskew_lidar").perform(context).lower() == "true"
    base_frame = LaunchConfiguration("base_frame").perform(context)
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    enable_viz = LaunchConfiguration("enable_viz").perform(context).lower() == "true"
    use_sim_time = use_sim_time_str.lower() == "true"
    database_path = LaunchConfiguration("database_path").perform(context).strip()
    map_mode = LaunchConfiguration("map_mode").perform(context).strip().lower()
    reset_map = LaunchConfiguration("reset_map").perform(context).lower() == "true"
    overrides_file = LaunchConfiguration("rtabmap_overrides_file").perform(context).strip()
    localization = bool(database_path) and map_mode == "localization"

    have_scan = bool(scan_topic) and scan_topic != _NONE
    have_scan_cloud = bool(scan_cloud_topic) and scan_cloud_topic != _NONE
    have_rgb = bool(rgb_topic) and rgb_topic != _NONE
    have_depth = bool(depth_topic) and depth_topic != _NONE
    have_rgbd = have_rgb and have_depth
    have_odom = bool(odom_topic) and odom_topic != _NONE
    have_imu = bool(imu_topic) and imu_topic != _NONE

    if deskew_lidar and not have_scan_cloud:
        raise RuntimeError("deskew_lidar requires a lidar3d PointCloud2 input")

    if not (have_scan or have_scan_cloud or have_rgbd):
        # rtabmap with neither lidar nor RGBD has nothing to map. Bail
        # loudly so the operator notices (instead of rtabmap silently
        # idling waiting for topics that will never arrive).
        raise RuntimeError(
            "rtabmap launch: no sensor inputs enabled. Set at least one "
            "of sensors.lidar2d / sensors.lidar3d / sensors.rgbd in the "
            "deploy manifest."
        )

    # Occupancy-grid source must auto-adapt to the sensors the deploy
    # actually wired up (via atlas_bridge's resolved.yaml), the same way
    # the subscriptions below do. Grid/Sensor: 0=laser scan(s) only,
    # 1=depth only, 2=both. A hardcoded "2" assumed a depth camera was
    # always present; on a lidar-only deploy (no RGBD) the depth half has
    # no input, so the projected grid stays empty (/map never populates).
    if have_rgbd and (have_scan or have_scan_cloud):
        grid_sensor = "2"
    elif have_rgbd:
        grid_sensor = "1"
    else:
        grid_sensor = "0"

    rtabmap_params = {
        "use_sim_time": use_sim_time,
        "frame_id": base_frame,
        "odom_frame_id": odom_frame,
        "map_frame_id": "map",
        "publish_tf": True,
        # Sensor subscriptions branch on what the deploy actually has.
        # rtabmap accepts EITHER 2D scan OR 3D scan_cloud (or both); the
        # 3D path is what real-robot Mid360 deployments use.
        "subscribe_scan": have_scan,
        "subscribe_scan_cloud": have_scan_cloud,
        "subscribe_rgbd": False,
        "subscribe_rgb": have_rgbd,
        "subscribe_depth": have_rgbd,
        "subscribe_odom_info": False,
        "odom_sensor_sync": have_odom,
        "approx_sync": True,
        "queue_size": 30,
        # webots emits image stamps slightly ahead of the dynamic TF
        # for the camera chain (head_2_link → Astra → ...), causing
        # "extrapolation into the future" + "TF of received image is
        # not set" errors when wait_for_transform is short. 1.5s gives
        # the TF buffer plenty of room to catch up.
        "wait_for_transform": 1.5,
        # Build the occupancy grid from whatever the deploy has. RTAB-Map
        # 0.21+ Grid/Sensor: 0=laser scan only, 1=depth only, 2=both.
        # `grid_sensor` (derived above from the present sensors) picks the
        # value automatically: a lidar-only robot projects its 3D cloud
        # (0), and a camera+lidar robot fuses both (2) so depth fills the
        # obstacles below the lidar plane (tables, chairs) the scan misses.
        "Grid/Sensor": grid_sensor,
        "Grid/FromDepth": "true" if have_rgbd else "false",
        # Persist per-node occupancy grids at insertion time. Saved maps must
        # reload into a usable /map later; RTAB-Map explicitly warns that nodes
        # inserted without this cache can make publish_map return an empty grid,
        # which breaks save/load and scene room overlays.
        "RGBD/CreateOccupancyGrid": "true",
        "Grid/RangeMax": "6.0",
        "Grid/CellSize": "0.05",
        "Grid/RayTracing": "true",
        # 3D pointcloud → 2D grid: when the only lidar is 3D, rtabmap
        # projects it to the planar grid via Grid/FromObstacles using
        # the same height clamp as the depth path.
        "Grid/3D": "false",
        "Grid/NormalsSegmentation": "false",
        # Height clamps tuned for the floor unevenness of the real
        # deploy environment (4F corridor): a stricter 0.05 m ground
        # cutoff misclassified slope/cable bumps as obstacles, while
        # the original 1.5 m obstacle cap clipped door frames + tall
        # shelves. Raise both bounds.
        "Grid/MaxObstacleHeight": "1.0",
        "Grid/MaxGroundHeight": "0.1",
        # Memory mode follows map_mode. Mapping: incremental (add nodes,
        # grow the graph). Localization: frozen graph (IncrementalMemory
        # off) initialised with all saved nodes, so rtabmap relocalises
        # against the loaded map and re-publishes the SAME map frame each
        # boot — the stable-origin property scene's per-map_id store needs.
        "Mem/IncrementalMemory": "false" if localization else "true",
        "Mem/InitWMWithAllNodes": "true" if localization else "false",
        "Reg/Strategy": "1",        # 0=Visual, 1=ICP, 2=Visual+ICP
        "Reg/Force3DoF": "true",
        "Optimizer/Strategy": "1",  # g2o
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        # Conservative defaults work for physical robots. Fast simulators may
        # opt into a denser profile through config.rtabmap_params.
        "RGBD/AngularUpdate": "0.1",
        "RGBD/LinearUpdate": "0.1",
        "Vis/MinInliers": "12",
        "Rtabmap/DetectionRate": "1.0",
        "Icp/MaxCorrespondenceDistance": "0.2",
        "Icp/MaxTranslation": "0.5",
        "Icp/MaxRotation": "0.78",  # ~45°
    }

    if overrides_file:
        try:
            with open(overrides_file, encoding="utf-8") as f:
                overrides = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"cannot read rtabmap_overrides_file={overrides_file!r}: {exc}") from exc
        if not isinstance(overrides, dict) or any(
            not isinstance(key, str) or not key or isinstance(value, (dict, list)) or value is None
            for key, value in overrides.items()
        ):
            raise RuntimeError("rtabmap_overrides_file must contain a JSON object of scalar parameters")
        # The ROS wrapper forwards RTAB-Map's slash-named parameters as
        # strings. Preserve that established type boundary even though JSON
        # decoded booleans/numbers have native Python types.
        rtabmap_params.update({
            key: str(value).lower() if isinstance(value, bool) else str(value)
            for key, value in overrides.items()
        })
        print(f"[rtabmap.launch] applied {len(overrides)} deploy override(s) from {overrides_file}")

    rtabmap_remappings = [
        # rviz "2D Pose Estimate" → /initialpose: rtabmap defaults to
        # the node-relative ~initialpose, remap to global so the rviz
        # tool reaches us without rviz config gymnastics.
        ("initialpose", "/initialpose"),
    ]
    if have_scan:
        rtabmap_remappings.append(("scan", scan_topic))
    deskewed_cloud_topic = "/rtabmap/scan_cloud_deskewed"
    if have_scan_cloud:
        rtabmap_remappings.append((
            "scan_cloud", deskewed_cloud_topic if deskew_lidar and have_odom else scan_cloud_topic
        ))
    if have_odom:
        rtabmap_remappings.append(("odom", odom_topic))
    elif have_scan or have_scan_cloud:
        rtabmap_remappings.append(("odom", "/rtabmap/odom"))
    if have_rgbd:
        rtabmap_remappings += [
            ("rgb/image", rgb_topic),
            ("rgb/camera_info", rgb_info_topic if rgb_info_topic != _NONE
                                else _derive_camera_info(rgb_topic)),
            ("depth/image", depth_topic),
        ]

    # Persist the graph at the deploy-chosen path when a named map is used;
    # otherwise rtabmap falls back to its default ~/.ros/rtabmap.db (the
    # legacy ephemeral path).
    if database_path:
        rtabmap_params["database_path"] = database_path

    # --delete_db_on_start wipes the db. Ephemeral (no named map) always
    # wipes — legacy temp-db behaviour. With a named map, wipe ONLY for an
    # explicit fresh start (mapping + reset_map); a normal mapping run
    # extends the existing db, and localization must never wipe.
    if not database_path or (map_mode == "mapping" and reset_map):
        rtabmap_args = ["--delete_db_on_start"]
    else:
        rtabmap_args = []
    print(f"[rtabmap.launch] map_mode={map_mode or 'ephemeral'} "
          f"db={database_path or '(default temp)'} "
          f"localization={localization} delete_db={bool(rtabmap_args)}")

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_params],
        arguments=rtabmap_args,
        remappings=rtabmap_remappings,
    )

    nodes = []

    filtered_imu_topic = "/rtabmap/imu/data"
    if have_imu and not have_odom:
        # Livox publishes angular velocity and acceleration, but leaves the
        # Imu orientation quaternion unset. RTAB-Map's wait_imu_to_init needs
        # a real attitude estimate, so never feed /livox/imu directly.
        nodes.append(Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="mapping_imu_filter",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "use_mag": False,
                "publish_tf": False,
                "world_frame": "enu",
            }],
            remappings=[
                ("imu/data_raw", imu_topic),
                ("imu/data", filtered_imu_topic),
            ],
        ))

    # A 100 ms Mid360 frame is visibly distorted while a skid-steer robot
    # rotates. With external odometry, compensate every point against the
    # odom TF before SLAM consumes the cloud. This requires a timestamp field
    # (Livox xfer_format=0); it is opt-in so generic PointXYZI providers fail
    # neither silently nor unexpectedly.
    if deskew_lidar and have_odom:
        nodes.append(Node(
            package="rtabmap_util",
            executable="lidar_deskewing",
            name="mapping_lidar_deskewing",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "fixed_frame_id": odom_frame,
                "wait_for_transform": 0.2,
                "slerp": True,
            }],
            remappings=[
                ("input_cloud", scan_cloud_topic),
                (f"{scan_cloud_topic}/deskewed", deskewed_cloud_topic),
            ],
        ))

    nodes.append(rtabmap_node)

    # When the deploy didn't supply external odom, run rtabmap's own
    # ICP-odometry node off whichever lidar source we have. icp_odometry
    # consumes either /scan (LaserScan) or /scan_cloud (PointCloud2);
    # we pick based on what's wired up.
    if not have_odom and (have_scan or have_scan_cloud):
        icp_odom_remappings = [("odom", "/rtabmap/odom")]
        if have_scan_cloud:
            icp_odom_remappings.append(("scan_cloud", scan_cloud_topic))
        elif have_scan:
            icp_odom_remappings.append(("scan", scan_topic))
        icp_odom_params = {
            "use_sim_time": use_sim_time,
            "frame_id": base_frame,
            "odom_frame_id": odom_frame,
            "publish_tf": True,
            "approx_sync": True,
            "wait_for_transform": 1.5,
            "deskewing": deskew_lidar,
            "deskewing_slerp": True,
            "Reg/Force3DoF": "true",
            "Icp/VoxelSize": "0.1",
            "Icp/PointToPlane": "true",
            "Icp/MaxCorrespondenceDistance": "1.0",
            "Odom/ScanKeyFrameThr": "0.4",
        }
        if have_imu:
            icp_odom_remappings.append(("imu", filtered_imu_topic))
            icp_odom_params["wait_imu_to_init"] = True
        icp_odom = Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            name="icp_odometry",
            output="screen",
            parameters=[icp_odom_params],
            remappings=icp_odom_remappings,
        )
        nodes.append(icp_odom)

    if enable_viz:
        viz_params = {
            "use_sim_time": use_sim_time,
            "frame_id": base_frame,
            "odom_frame_id": odom_frame,
            "subscribe_scan": have_scan,
            "subscribe_scan_cloud": have_scan_cloud,
            "subscribe_rgb": False,
            "subscribe_depth": False,
            "approx_sync": True,
            "queue_size": 30,
            "wait_for_transform": 1.5,
        }
        viz_remappings = []
        if have_scan:
            viz_remappings.append(("scan", scan_topic))
        if have_scan_cloud:
            viz_remappings.append(("scan_cloud", scan_cloud_topic))
        if have_odom:
            viz_remappings.append(("odom", odom_topic))
        viz = Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            name="rtabmap_viz",
            output="screen",
            parameters=[viz_params],
            remappings=viz_remappings,
        )
        nodes.append(viz)

    # tf2 → /robonix/map/pose adapter. rtabmap in mapping mode does
    # NOT publish /localization_pose; the SLAM-corrected pose is
    # only on the tf2 chain. The robonix `service/map/pose` contract
    # promises a topic-out PoseWithCovarianceStamped, so we run a
    # small adapter that polls tf2 and republishes. Without this
    # scene's self-tracker silently fell back to chassis /odom and
    # the web UI's robot dot drifted from rviz once SLAM corrected.
    #
    # ExecuteProcess (not launch_ros.Node) because the script is a
    # standalone Python file under scripts/, not a ros2 entrypoint
    # registered in a setup.py — there's no `package + executable`
    # to look up.
    pkg_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    tf_adapter = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(pkg_root, "scripts", "tf_to_pose.py"),
            "--ros-args",
            "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            "-p", "map_frame:=map",
            "-p", f"base_frame:={base_frame}",
            "-p", "publish_rate_hz:=10.0",
            "-p", "topic:=/robonix/map/pose",
        ],
        name="tf_to_pose",
        output="screen",
    )
    nodes.append(tf_adapter)

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
