# SPDX-License-Identifier: MulanPSL-2.0
"""2D SLAM with slam_toolbox, wired to the mapping service's contract surface.

slam_toolbox does scan matching against a pose graph and nothing else: it owns
`/map` and the `map → odom` transform, has no visual loop closure to reject, no
database, and no GPU. That is the point of having it next to RTAB-Map — on a
planar indoor robot with a 2D lidar it is the cheap, predictable option, and its
pose graph serializes to two files instead of a sqlite database.

Two adapters keep the exported surface complete:
  tf_to_pose.py            map → base_link  →  `service/map/pose`
  scan_to_map_outputs.py   scans → map-frame cloud, SLAM pose → `service/map/odom`

Everything deployment-specific (scan topic, frames, mode) arrives as launch
arguments; `start_engine.sh` fills them from the resolved contract topics.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _launch_setup(context, *args, **kwargs):
    """Build the node list. Side effects: none."""
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    map_frame = LaunchConfiguration("map_frame").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    mode = LaunchConfiguration("map_mode").perform(context).strip().lower()
    resolution = float(LaunchConfiguration("resolution").perform(context))
    max_laser_range = float(LaunchConfiguration("max_laser_range").perform(context))

    common = {"use_sim_time": use_sim_time}
    slam_params = {
        **common,
        "odom_frame": odom_frame,
        "map_frame": map_frame,
        "base_frame": base_frame,
        "scan_topic": scan_topic,
        # `mapping` builds a new graph; `localization` needs a serialized graph,
        # which map_ops supplies through the deserialize service after start.
        "mode": "localization" if mode == "localization" else "mapping",
        "resolution": resolution,
        "max_laser_range": max_laser_range,
        "transform_publish_period": 0.02,
        "map_update_interval": 1.0,
        # Scan matching: the defaults are tuned for a robot that drives, and this
        # is the part that decides whether the map stays rigid on curves.
        "minimum_travel_distance": 0.2,
        "minimum_travel_heading": 0.2,
        "scan_buffer_size": 20,
        "do_loop_closing": True,
        "loop_search_maximum_distance": 3.0,
        "loop_match_minimum_response_fine": 0.45,
        "minimum_time_interval": 0.2,
        "stack_size_to_use": 40000000,
    }
    sim_time = "true" if use_sim_time else "false"
    # The two adapters are standalone scripts under scripts/, not ros2
    # entrypoints registered in a setup.py, so they are run the same way
    # rtabmap_2d.launch.py runs tf_to_pose: ExecuteProcess with python3.
    return [
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", output="screen", parameters=[slam_params]),
        ExecuteProcess(
            cmd=["python3", os.path.join(_PKG_DIR, "scripts", "tf_to_pose.py"),
                 "--ros-args",
                 "-p", f"use_sim_time:={sim_time}",
                 "-p", f"map_frame:={map_frame}",
                 "-p", f"base_frame:={base_frame}",
                 "-p", "publish_rate_hz:=10.0",
                 "-p", "topic:=/robonix/map/pose"],
            name="tf_to_pose", output="screen"),
        ExecuteProcess(
            cmd=["python3", os.path.join(_PKG_DIR, "scripts", "scan_to_map_outputs.py"),
                 "--scan-topic", scan_topic, "--map-frame", map_frame,
                 "--base-frame", base_frame,
                 "--ros-args", "-p", f"use_sim_time:={sim_time}"],
            name="scan_to_map_outputs", output="screen"),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments start_engine.sh passes and build the stack."""
    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_mode", default_value="mapping"),
        DeclareLaunchArgument("resolution", default_value="0.05"),
        DeclareLaunchArgument("max_laser_range", default_value="12.0"),
        OpaqueFunction(function=_launch_setup),
    ])
