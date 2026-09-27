# SPDX-License-Identifier: MulanPSL-2.0
"""slam_toolbox localizing on a saved pose graph, without editing it.

The asynchronous node this deployment maps with has no localization-only
behaviour: it folds every scan into whatever graph it holds, so loading a saved
map into it merges that map with the session already in memory and then keeps
editing the result as the robot drives. slam_toolbox ships a separate
executable for the other half of the job, and which one runs is decided when the
process starts -- hence a second launch file rather than a parameter.

Started with the pose a relocalization already recovered, so it does not begin
by guessing. While it runs it is the only publisher of `map -> odom` and the
only publisher of `/map`; the particle filter that found the robot is stopped
before this starts.

Every tuning value comes from the same function the mapping launch uses. A
hand-typed subset once left the loop-closure gates at slam_toolbox's stock
values, and a wrong loop closure wrenched a freshly relocalized robot two metres
sideways within seconds of the handover.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_LAUNCH_DIR)


def _mapping_launch_module():
    """Load the sibling `slam_toolbox_2d.launch.py`; its dotted filename keeps
    it off the normal import path."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "slam_toolbox_2d_launch", os.path.join(_LAUNCH_DIR, "slam_toolbox_2d.launch.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


slam_toolbox_params = _mapping_launch_module().slam_toolbox_params


def _nodes(context, *args, **kwargs):
    """Build the node list with the arguments resolved to plain Python values."""
    cfg = {k: LaunchConfiguration(k).perform(context) for k in (
        "map_file_name", "scan_topic", "base_frame", "odom_frame", "map_frame",
        "use_sim_time", "resolution", "max_laser_range", "minimum_travel_distance",
        "minimum_travel_heading", "scan_buffer_size", "loop_search_maximum_distance",
        "start_x", "start_y", "start_yaw")}
    use_sim_time = cfg["use_sim_time"].lower() == "true"

    params = slam_toolbox_params(
        scan_topic=cfg["scan_topic"], base_frame=cfg["base_frame"],
        odom_frame=cfg["odom_frame"], map_frame=cfg["map_frame"],
        use_sim_time=use_sim_time, mode="localization",
        resolution=float(cfg["resolution"]), max_laser_range=float(cfg["max_laser_range"]),
        travel_distance=float(cfg["minimum_travel_distance"]),
        travel_heading=float(cfg["minimum_travel_heading"]),
        scan_buffer=int(float(cfg["scan_buffer_size"])),
        loop_distance=float(cfg["loop_search_maximum_distance"]))
    params.update({
        # Loop closure exists to fix drift accumulated in a graph being built.
        # Against a map that is already fixed it has nothing to correct and
        # everything to break: a closure fired mid-drive and wrenched a
        # correctly localized robot three metres sideways in one step, heading
        # untouched, then let it crawl back. Localizing does not need it.
        "do_loop_closing": False,
        "map_file_name": cfg["map_file_name"],
        # Where the robot is on that graph. The relocalization that ran before
        # this found it; without a pose the node would place the robot at the
        # graph's first node, which is only right if the robot is back at the
        # spot the recording started from.
        "map_start_pose": [float(cfg["start_x"]), float(cfg["start_y"]), float(cfg["start_yaw"])],
        "map_start_at_dock": False,
    })

    # The two adapters that keep the exported surface complete come with the
    # engine, not with the mode: swapping to this node without them left
    # `service/map/pose` with no publisher, so every consumer -- the status
    # page included -- kept reporting the last pose the mapping node had sent,
    # which looks exactly like a robot frozen at the map origin.
    sim_time = "true" if use_sim_time else "false"
    return [
        Node(package="slam_toolbox", executable="localization_slam_toolbox_node",
             name="slam_toolbox", output="screen", parameters=[params]),
        ExecuteProcess(
            cmd=["python3", os.path.join(_PKG_DIR, "scripts", "tf_to_pose.py"),
                 "--ros-args",
                 "-p", f"use_sim_time:={sim_time}",
                 "-p", f"map_frame:={cfg['map_frame']}",
                 "-p", f"base_frame:={cfg['base_frame']}",
                 "-p", "publish_rate_hz:=10.0",
                 "-p", "topic:=/robonix/map/pose"],
            name="tf_to_pose", output="screen"),
        ExecuteProcess(
            cmd=["python3", os.path.join(_PKG_DIR, "scripts", "scan_to_map_outputs.py"),
                 "--scan-topic", cfg["scan_topic"], "--map-frame", cfg["map_frame"],
                 "--base-frame", cfg["base_frame"],
                 "--ros-args", "-p", f"use_sim_time:={sim_time}"],
            name="scan_to_map_outputs", output="screen"),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments `engines.py` and `start_engine.sh` fill in.

    The tuning arguments and their defaults are the mapping launch's, so a
    deployment's overrides apply to both nodes alike.
    """
    return LaunchDescription([
        DeclareLaunchArgument("map_file_name"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("resolution", default_value="0.05"),
        DeclareLaunchArgument("max_laser_range", default_value="12.0"),
        DeclareLaunchArgument("minimum_travel_distance", default_value="0.1"),
        DeclareLaunchArgument("minimum_travel_heading", default_value="0.1"),
        DeclareLaunchArgument("scan_buffer_size", default_value="30"),
        DeclareLaunchArgument("loop_search_maximum_distance", default_value="2.5"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        OpaqueFunction(function=_nodes),
    ])
