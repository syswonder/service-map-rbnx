# SPDX-License-Identifier: MulanPSL-2.0
"""Localization on a saved 2D map: map_server + a particle-filter localizer.

Brought up by `load_map` when the deployment sets `localizer:` to something
other than `none`. It replaces the SLAM engine's own localization mode with the
standard ROS 2 stack, which is what makes automatic global relocalization
possible: the localizer exposes `reinitialize_global_localization`, which
scatters particles over the free space of the map so the robot converges
without anyone typing a pose or clicking "2D Pose Estimate".

Nodes:
  map_server          serves `<map_dir>/occupancy.yaml` on /map (latched)
  <localizer>         nav2_amcl or beluga_amcl (interface-compatible):
                      map → odom TF, /amcl_pose, global localization service
  lifecycle_manager   configures + activates both, autostart

The SLAM engine keeps running in mapping mode or is left stopped by the caller;
this launch never touches it. Frames, topics and the map path all come in as
launch arguments so nothing here is deployment-specific.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LOCALIZERS = {"amcl": "nav2_amcl", "beluga": "beluga_amcl"}


def _launch_setup(context, *args, **kwargs):
    """Build the node list from the resolved arguments.

    Side effects: none. Raises RuntimeError for an unknown localizer so the
    failure names the accepted values instead of a missing-executable error
    from ros2 launch.
    """
    localizer = LaunchConfiguration("localizer").perform(context).strip().lower()
    if localizer not in _LOCALIZERS:
        raise RuntimeError(
            f"unknown localizer {localizer!r}; expected one of {', '.join(sorted(_LOCALIZERS))}"
        )
    package = _LOCALIZERS[localizer]
    map_yaml = LaunchConfiguration("map_yaml").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    global_frame = LaunchConfiguration("global_frame").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    min_particles = int(LaunchConfiguration("min_particles").perform(context))
    max_particles = int(LaunchConfiguration("max_particles").perform(context))

    common = {"use_sim_time": use_sim_time}
    # Particle counts are the whole resource story for MCL: 500-2000 covers a
    # room-scale map in a few tens of MB of RAM and a few percent of one core.
    localizer_params = {
        **common,
        "base_frame_id": base_frame,
        "odom_frame_id": odom_frame,
        "global_frame_id": global_frame,
        "scan_topic": scan_topic,
        "min_particles": min_particles,
        "max_particles": max_particles,
        # Recovery: without these the filter cannot inject random particles when
        # the estimate goes bad, which is half of "it never re-converges".
        "recovery_alpha_slow": 0.001,
        "recovery_alpha_fast": 0.1,
        "update_min_d": 0.15,
        "update_min_a": 0.15,
        "laser_model_type": "likelihood_field",
        "set_initial_pose": False,
        "always_reset_initial_pose": False,
        "tf_broadcast": True,
        "transform_tolerance": 1.0,
    }
    nodes = [
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            output="screen", parameters=[{**common, "yaml_filename": map_yaml, "frame_id": global_frame}],
        ),
        Node(
            package=package, executable=package.replace("_", "-") if package == "beluga_amcl" else "amcl",
            name="amcl", output="screen", parameters=[localizer_params],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_localization", output="screen",
            parameters=[{**common, "autostart": True, "node_names": ["map_server", "amcl"]}],
        ),
    ]
    return nodes


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments `localizers.py` fills in and build the stack."""
    return LaunchDescription([
        DeclareLaunchArgument("localizer", default_value="amcl",
                              description="amcl (nav2) or beluga (beluga_amcl, drop-in)"),
        DeclareLaunchArgument("map_yaml", description="path to the saved occupancy.yaml"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("global_frame", default_value="map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("min_particles", default_value="500"),
        DeclareLaunchArgument("max_particles", default_value="2000"),
        OpaqueFunction(function=_launch_setup),
    ])
