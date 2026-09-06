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
                      /amcl_pose + the global localization service. It does NOT
                      publish map → odom: the SLAM engine owns that frame, and
                      load_map hands it the recovered pose.
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
    tf_broadcast = LaunchConfiguration("tf_broadcast").perform(context).lower() == "true"
    initial_pose = [float(LaunchConfiguration(k).perform(context))
                    for k in ("initial_x", "initial_y", "initial_yaw")]
    has_initial_pose = any(v != 0.0 for v in initial_pose)

    # The saved grid goes on a topic of its own. Published on /map it collides
    # with the SLAM engine's live grid: two publishers, two different origins,
    # and RViz redraws a different map every frame while the robot appears to
    # jump between them. /map stays the engine's; the filter reads this one.
    localizer_map_topic = LaunchConfiguration("map_topic").perform(context)

    common = {"use_sim_time": use_sim_time}
    # Particle counts are the whole resource story for MCL, and global
    # localization is what sets the floor: the particles are scattered over
    # every free cell and every heading at once, so too few of them means no
    # particle starts near the truth and the filter collapses confidently onto
    # whichever wrong hypothesis it did cover. A room-scale map wants thousands.
    localizer_params = {
        **common,
        "base_frame_id": base_frame,
        "odom_frame_id": odom_frame,
        "global_frame_id": global_frame,
        "scan_topic": scan_topic,
        "map_topic": localizer_map_topic,
        "min_particles": min_particles,
        "max_particles": max_particles,
        # No random-particle injection. The localizer here runs only until a
        # global relocalization succeeds and is then stopped, so injection buys
        # no long-run recovery; what it did buy was a filter that kept adding
        # noise to a cloud that was trying to settle, leaving the heading
        # spread hovering above the convergence threshold. Recovering from a
        # confident wrong answer is handled where it can be judged: the caller
        # scores the fix against the map and re-scatters when it does not fit.
        "recovery_alpha_slow": 0.0,
        "recovery_alpha_fast": 0.0,
        "update_min_d": 0.15,
        "update_min_a": 0.15,
        "laser_model_type": "likelihood_field",
        # A seeded start is how the filter is handed back a pose it already
        # recovered, so a restart into pose-owning mode does not begin by
        # searching the whole map again.
        "set_initial_pose": has_initial_pose,
        "initial_pose.x": initial_pose[0],
        "initial_pose.y": initial_pose[1],
        "initial_pose.yaw": initial_pose[2],
        "always_reset_initial_pose": False,
        # Exactly one node publishes map -> odom, and which one depends on the
        # mode. While the robot is being relocalized the SLAM engine still owns
        # it and the filter must stay quiet (tf_broadcast false); once the
        # deployment is localizing on a saved map the engine is paused so the
        # map cannot be edited, and the filter owns the frame instead. Two
        # publishers overwrite each other and the robot teleports between their
        # answers, so this is never both.
        "tf_broadcast": tf_broadcast,
        "transform_tolerance": 1.0,
    }
    nodes = [
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            output="screen",
            parameters=[{**common, "yaml_filename": map_yaml, "frame_id": global_frame,
                         "topic_name": localizer_map_topic}],
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
        # /map while the SLAM engine is stopped -- the operator still has to see
        # the map they loaded -- and a private topic whenever the engine is the
        # one publishing there.
        DeclareLaunchArgument("map_topic", default_value="/localizer/map"),
        DeclareLaunchArgument("tf_broadcast", default_value="false"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("min_particles", default_value="800"),
        DeclareLaunchArgument("max_particles", default_value="8000"),
        OpaqueFunction(function=_launch_setup),
    ])
