import importlib.util
import os
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parent


def load_profiles():
    spec = importlib.util.spec_from_file_location(
        "rtabmap_profiles", ROOT / "src" / "mapping_rbnx" / "profiles.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RtabmapConfigurationTest(unittest.TestCase):
    def test_docker_start_mounts_manifest_directory_read_only(self):
        bash_major = int(
            subprocess.run(
                ["bash", "-c", "printf %s ${BASH_VERSINFO[0]}"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        )
        if bash_major < 4:
            self.skipTest("provider Docker wrapper requires Bash 4 or newer")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "package"
            deploy = root / "robot deploy"
            fake_bin = root / "bin"
            package.mkdir()
            deploy.mkdir()
            fake_bin.mkdir()
            runtime_proto = root / "runtime-proto"
            runtime_proto.mkdir()
            (runtime_proto / "atlas.proto").write_text(
                'syntax = "proto3";\n', encoding="utf-8"
            )
            proto_staging = package / "rbnx-build" / "proto-staging"
            proto_staging.mkdir(parents=True)
            (proto_staging / "mapping.proto").write_text(
                'syntax = "proto3";\n', encoding="utf-8"
            )
            docker_args = root / "docker.args"
            docker = fake_bin / "docker"
            docker.write_text(
                '#!/usr/bin/env bash\n'
                'if [[ "${1:-}" == run ]]; then\n'
                '  printf "%s\\n" "$@" > "$DOCKER_ARGS_FILE"\n'
                "fi\n",
                encoding="utf-8",
            )
            docker.chmod(0o755)
            rbnx = fake_bin / "rbnx"
            rbnx.write_text(
                f'#!/usr/bin/env bash\necho "{runtime_proto}"\n',
                encoding="utf-8",
            )
            rbnx.chmod(0o755)
            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{fake_bin}:/usr/bin:/bin",
                    "DOCKER_ARGS_FILE": str(docker_args),
                    "RBNX_PACKAGE_ROOT": str(package),
                    "RBNX_INVOCATION_CWD": str(deploy),
                    "ROBONIX_MAPPING_FORCE": "docker",
                    "DISPLAY": "",
                }
            )
            subprocess.run(
                ["bash", str(ROOT / "scripts" / "start.sh")],
                check=True,
                env=env,
                capture_output=True,
                text=True,
            )
            args = docker_args.read_text(encoding="utf-8").splitlines()
            self.assertIn(f"RBNX_INVOCATION_CWD={deploy}", args)
            self.assertIn(f"{deploy}:{deploy}:ro", args)

    def test_legacy_profile_remains_compatible(self):
        profiles = load_profiles()
        values = profiles.resolve_rtabmap_overrides({}, "ranger_mini_v3")
        self.assertEqual(values["Grid/FootprintLength"], 0.84)
        self.assertEqual(values["Rtabmap/DetectionRate"], 5.0)

    def test_unknown_legacy_profile_fails(self):
        profiles = load_profiles()
        with self.assertRaisesRegex(RuntimeError, "unknown legacy rtabmap_profile"):
            profiles.resolve_rtabmap_overrides({}, "unknown_robot")

    def test_single_provider_can_be_selected_implicitly(self):
        profiles = load_profiles()
        record = SimpleNamespace(provider_id="mid360_lidar")
        self.assertIs(
            profiles.choose_provider_record([record], "", "lidar3d"), record
        )

    def test_multiple_providers_require_explicit_id(self):
        profiles = load_profiles()
        records = [
            SimpleNamespace(provider_id="front_lidar"),
            SimpleNamespace(provider_id="rear_lidar"),
        ]
        with self.assertRaisesRegex(RuntimeError, "multiple Atlas providers"):
            profiles.choose_provider_record(records, "", "lidar3d")

    def test_provider_bindings_are_the_sensor_enablement_source(self):
        source = (ROOT / "src" / "mapping_rbnx" / "atlas_bridge.py").read_text()
        self.assertIn('providers = cfg.get("sensor_providers")', source)
        self.assertIn("return {key: key in providers", source)
        self.assertIn("config.sensors is deprecated", source)

    def test_deployment_overrides_are_preserved(self):
        profiles = load_profiles()
        values = profiles.resolve_rtabmap_overrides(
            {"Rtabmap/DetectionRate": 2.0, "Grid/RangeMin": 0.20},
        )
        self.assertEqual(values["Rtabmap/DetectionRate"], 2.0)
        self.assertEqual(values["Grid/RangeMin"], 0.20)

    def test_deploy_params_file_is_relative_to_manifest_and_inline_wins(self):
        profiles = load_profiles()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config" / "rtabmap.yaml"
            path.parent.mkdir()
            path.write_text(
                "Rtabmap/DetectionRate: 2.0\nGrid/RangeMin: 0.25\n",
                encoding="utf-8",
            )
            old = os.environ.get("RBNX_INVOCATION_CWD")
            os.environ["RBNX_INVOCATION_CWD"] = directory
            try:
                values = profiles.resolve_rtabmap_overrides(
                    {"Rtabmap/DetectionRate": 3.0},
                    params_file="config/rtabmap.yaml",
                )
            finally:
                if old is None:
                    os.environ.pop("RBNX_INVOCATION_CWD", None)
                else:
                    os.environ["RBNX_INVOCATION_CWD"] = old
        self.assertEqual(values["Rtabmap/DetectionRate"], 3.0)
        self.assertEqual(values["Grid/RangeMin"], 0.25)

    def test_missing_deploy_params_file_fails_loudly(self):
        profiles = load_profiles()
        with self.assertRaisesRegex(RuntimeError, "params_file not found"):
            profiles.resolve_rtabmap_overrides({}, params_file="missing.yaml")

    def test_nested_deployment_override_is_rejected(self):
        profiles = load_profiles()
        with self.assertRaisesRegex(RuntimeError, "must be a scalar"):
            profiles.resolve_rtabmap_overrides({"nested": {"value": 1}})

    def test_upstream_file_is_a_template_not_a_runtime_default(self):
        text = (ROOT / "config" / "rtabmap_params.template.yaml").read_text()
        self.assertIn("Rtabmap/DetectionRate: 1.0", text)
        self.assertNotIn("Grid/FootprintLength", text)
        self.assertNotIn("Grid/FootprintWidth", text)
        launch = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        self.assertNotIn("rtabmap_params.template.yaml", launch)
        self.assertNotIn("_load_default_rtabmap_params", launch)

    def test_config_spec_documents_deploy_owned_file(self):
        text = (ROOT / "config.spec").read_text()
        self.assertIn("config/rtabmap_params.template.yaml", text)
        self.assertIn("never loaded at runtime", text)
        self.assertIn("sensor_providers:", text)
        self.assertIn("Icp/* and Odom/* values", text)
        self.assertIn("dense_scan_2d:", text)
        self.assertIn("dense_scan_refine_neighbors:", text)
        self.assertIn("default: false", text)

    def test_occupancy_source_is_policy_not_sensor_inference(self):
        profiles = load_profiles()
        values = profiles.resolve_occupancy_sources(
            ["lidar"], {"lidar", "depth"}
        )
        self.assertEqual(values["Grid/Sensor"], 0)
        self.assertIs(values["Grid/FromDepth"], False)

    def test_explicit_lidar_and_depth_fusion(self):
        profiles = load_profiles()
        values = profiles.resolve_occupancy_sources(
            ["lidar", "depth"], {"lidar", "depth"}
        )
        self.assertEqual(values["Grid/Sensor"], 2)
        self.assertIs(values["Grid/FromDepth"], True)

    def test_missing_requested_source_fails_loudly(self):
        profiles = load_profiles()
        with self.assertRaisesRegex(RuntimeError, "not resolved from Atlas"):
            profiles.resolve_occupancy_sources(["depth"], {"lidar"})

    def test_explicit_inputs_drop_rgbd_without_hiding_atlas_capabilities(self):
        profiles = load_profiles()
        resolved = {
            "lidar_topic": "/scanner/cloud",
            "rgb_topic": "/camera/color",
            "depth_topic": "/camera/depth",
            "odom_topic": "/odom",
            "imu_topic": "/imu",
        }
        selected = profiles.select_rtabmap_inputs(["lidar", "odom"], resolved)
        self.assertEqual(
            selected,
            {"lidar_topic": "/scanner/cloud", "odom_topic": "/odom"},
        )

    def test_explicit_imu_input_is_preserved(self):
        profiles = load_profiles()
        selected = profiles.select_rtabmap_inputs(
            ["lidar", "imu"],
            {"lidar_topic": "/scanner/cloud", "imu_topic": "/livox/imu"},
        )
        self.assertEqual(
            selected,
            {"lidar_topic": "/scanner/cloud", "imu_topic": "/livox/imu"},
        )

    def test_visual_fusion_keeps_lidar_rgbd_and_external_odom(self):
        profiles = load_profiles()
        selected = profiles.select_rtabmap_inputs(
            ["lidar", "rgbd", "odom"],
            {
                "lidar_topic": "/scanner/cloud",
                "rgb_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/aligned_depth/image_raw",
                "odom_topic": "/odom",
            },
        )
        self.assertEqual(
            selected,
            {
                "lidar_topic": "/scanner/cloud",
                "rgb_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/aligned_depth/image_raw",
                "odom_topic": "/odom",
            },
        )

    def test_external_odom_keeps_its_original_capability_owner(self):
        source = (ROOT / "src" / "mapping_rbnx" / "atlas_bridge.py").read_text()
        self.assertIn(
            'contract_id == "robonix/service/map/odom" and resolved.get("odom_topic")',
            source,
        )
        self.assertIn("external odom remains owned by its provider", source)

    def test_occupancy_grid_declares_transient_local_qos(self):
        source = (ROOT / "src" / "mapping_rbnx" / "atlas_bridge.py").read_text()
        self.assertIn(
            'contract_id == "robonix/service/map/occupancy_grid"',
            source,
        )
        self.assertIn('mapping.declare_ros2_topic(contract_id, topic, qos=output_qos)', source)
        self.assertIn('"transient_local"', source)

    def test_external_odom_uses_canonical_tf_mode(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        self.assertIn(
            'rtabmap_odom_frame = "" if navigation_odom_bridge else odom_frame',
            source,
        )
        self.assertEqual(
            source.count('"odom_frame_id": rtabmap_odom_frame'), 2
        )
        self.assertIn('"odom_sensor_sync": False', source)
        self.assertIn('rtabmap_remappings.append(("odom", odom_topic))', source)

    def test_raw_livox_imu_is_filtered_before_icp(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        self.assertIn('package="imu_filter_madgwick"', source)
        self.assertIn('(\"imu/data_raw\", imu_topic)', source)
        self.assertIn('(\"imu\", filtered_imu_topic)', source)

    def test_icp_motion_limits_are_forwarded_to_internal_odometry(self):
        profiles = load_profiles()
        selected = profiles.select_icp_odometry_overrides(
            {
                "Icp/MaxTranslation": "0.25",
                "Icp/MaxRotation": "0.20",
                "Grid/RangeMax": "10.0",
            }
        )
        self.assertEqual(
            selected,
            {
                "Icp/MaxTranslation": "0.25",
                "Icp/MaxRotation": "0.20",
            },
        )

    def test_navigation_odom_bridge_is_opt_in_and_keeps_legacy_defaults(self):
        launch = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        self.assertIn(
            'DeclareLaunchArgument("navigation_odom_bridge", default_value="false")',
            launch,
        )
        self.assertIn(
            '"/rtabmap/odom_icp" if navigation_odom_bridge else "/rtabmap/odom"',
            launch,
        )
        self.assertIn('"publish_tf": not navigation_odom_bridge', launch)
        self.assertIn(
            'rtabmap_odom_frame = "" if navigation_odom_bridge else odom_frame',
            launch,
        )
        self.assertIn('"publish_tf": True', launch)

    def test_navigation_odom_bridge_config_reaches_launch(self):
        bridge = (ROOT / "src" / "mapping_rbnx" / "atlas_bridge.py").read_text()
        engine = (ROOT / "scripts" / "start_engine.sh").read_text()
        self.assertIn('"navigation_odom_bridge"', bridge)
        self.assertIn('navigation_odom_bridge:="$NAV_ODOM_BRIDGE"', engine)
        self.assertIn('navigation_odom_topic:="$NAV_ODOM_TOPIC"', engine)
        self.assertIn('navigation_odom_frame:="$NAV_ODOM_FRAME"', engine)

    def test_rgbd_only_profile_starts_visual_odometry(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        self.assertIn("elif not have_odom and have_rgbd:", source)
        self.assertIn('executable="rgbd_odometry"', source)
        self.assertIn('(\"rgb/image\", rgb_topic)', source)
        self.assertIn('(\"depth/image\", depth_topic)', source)
        self.assertIn(
            "elif have_scan or have_scan_cloud or have_rgbd:", source
        )

    def test_lidar_consumers_use_sensor_data_best_effort_qos(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        self.assertIn("_SENSOR_DATA_QOS = 2", source)
        # rtabmap_slam and rtabmap_viz expose qos_scan, while
        # rtabmap_odom/icp_odometry exposes generic qos plus qos_imu.
        self.assertEqual(source.count('"qos_scan": _SENSOR_DATA_QOS'), 2)
        # icp_odometry, lidar_deskewing and the opt-in cloud assembler expose
        # generic `qos`.
        self.assertEqual(source.count('"qos": _SENSOR_DATA_QOS'), 3)
        self.assertEqual(source.count('"qos_imu": _SENSOR_DATA_QOS'), 1)

    def test_dense_scan_2d_is_explicit_and_disabled_by_default(self):
        bridge = (ROOT / "src" / "mapping_rbnx" / "atlas_bridge.py").read_text()
        engine = (ROOT / "scripts" / "start_engine.sh").read_text()
        launch = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()

        self.assertIn('"dense_scan_2d",', bridge)
        self.assertIn("DENSE_SCAN_2D=$(read_y dense_scan_2d)", engine)
        self.assertIn('DENSE_SCAN_2D="${DENSE_SCAN_2D:-false}"', engine)
        self.assertIn('dense_scan_2d:="$DENSE_SCAN_2D"', engine)
        self.assertIn(
            'DeclareLaunchArgument("dense_scan_2d", default_value="false")',
            launch,
        )
        self.assertIn("if dense_scan_2d:", launch)
        self.assertIn("dense_scan_2d requires external odometry", launch)
        self.assertIn("dense_scan_2d requires deskew_lidar=true", launch)

    def test_dense_scan_2d_chain_has_bounded_production_parameters(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()

        self.assertIn('executable="point_cloud_assembler"', source)
        self.assertIn('"fixed_frame_id": odom_frame', source)
        self.assertIn('"frame_id": base_frame', source)
        self.assertIn('"max_clouds": 4', source)
        self.assertIn('"assembling_time": 0.75', source)
        self.assertIn("max_clouds\n        # and assembling_time as an OR bound", source)
        self.assertIn('"circular_buffer": True', source)
        self.assertIn('"voxel_size": 0.035', source)
        self.assertIn('("cloud", deskewed_cloud_topic)', source)
        self.assertIn('("assembled_cloud", assembled_cloud_topic)', source)

        self.assertIn('executable="pointcloud_to_laserscan_node"', source)
        self.assertIn('"target_frame": base_frame', source)
        self.assertIn('"min_height": -0.25', source)
        self.assertIn('"max_height": 0.50', source)
        self.assertIn('"angle_increment": 0.008726646259971648', source)
        self.assertIn('"queue_size": 10', source)
        self.assertIn('"scan_time": 0.26', source)
        self.assertIn('"range_min": 0.492', source)
        self.assertIn('"range_max": 6.0', source)
        self.assertIn('"use_inf": True', source)
        self.assertIn('"inf_epsilon": 1.0', source)
        self.assertIn('("cloud_in", assembled_cloud_topic)', source)
        self.assertIn('("scan", dense_scan_topic)', source)

        self.assertIn('"subscribe_scan": rtabmap_have_scan', source)
        self.assertIn(
            '"subscribe_scan_cloud": rtabmap_have_scan_cloud',
            source,
        )
        self.assertIn(
            'rtabmap_params["Grid/Scan2dUnknownSpaceFilled"] = "false"',
            source,
        )

    def test_dense_neighbor_refinement_is_explicit_and_dense_only(self):
        bridge = (ROOT / "src" / "mapping_rbnx" / "atlas_bridge.py").read_text()
        engine = (ROOT / "scripts" / "start_engine.sh").read_text()
        launch = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()

        self.assertIn('"dense_scan_refine_neighbors",', bridge)
        self.assertIn(
            "DENSE_SCAN_REFINE_NEIGHBORS=$(read_y dense_scan_refine_neighbors)",
            engine,
        )
        self.assertIn(
            'DENSE_SCAN_REFINE_NEIGHBORS="${DENSE_SCAN_REFINE_NEIGHBORS:-false}"',
            engine,
        )
        self.assertIn(
            'dense_scan_refine_neighbors:="$DENSE_SCAN_REFINE_NEIGHBORS"',
            engine,
        )
        self.assertIn(
            '"dense_scan_refine_neighbors", default_value="false"',
            launch,
        )
        self.assertIn(
            "dense_scan_refine_neighbors requires dense_scan_2d=true",
            launch,
        )
        self.assertIn(
            "neighbor refinement is never enabled on the sparse raw",
            launch,
        )

    def test_dense_neighbor_refinement_preserves_deploy_precedence(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()

        applied = source.index("rtabmap_params.update(deployment_overrides)")
        neighbor_default = source.index(
            'rtabmap_params.setdefault("RGBD/NeighborLinkRefining", "true")'
        )
        self.assertLess(applied, neighbor_default)
        self.assertIn(
            'rtabmap_params.setdefault("Reg/Strategy", "1")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Reg/Force3DoF", "true")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("RGBD/ProximityBySpace", "false")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("RGBD/ProximityPathMaxNeighbors", "0")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/PointToPlane", "true")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/PointToPlaneK", "5")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/PointToPlaneMinComplexity", "0.02")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/PointToPlaneLowComplexityStrategy", "1")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/CorrespondenceRatio", "0.20")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/MaxCorrespondenceDistance", "0.15")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/MaxTranslation", "0.10")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/MaxRotation", "0.10")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/RangeMin", "0.492")',
            source,
        )
        self.assertIn(
            'rtabmap_params.setdefault("Icp/RangeMax", "6.0")',
            source,
        )
        self.assertIn(
            '"RGBD/NeighborLinkRefining" in deployment_overrides',
            source,
        )
        self.assertIn("the explicit deploy value takes precedence", source)
        self.assertIn("dense_scan_refine_neighbors requires ", source)
        self.assertIn("RGBD/ProximityBySpace=false", source)
        self.assertIn(
            "dense_scan_refine_neighbors requires Reg/Strategy=1",
            source,
        )

    def test_dense_neighbor_inline_false_overrides_params_file_true(self):
        profiles = load_profiles()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rtabmap.yaml"
            path.write_text(
                "RGBD/NeighborLinkRefining: true\n"
                "RGBD/ProximityBySpace: false\n",
                encoding="utf-8",
            )
            old = os.environ.get("RBNX_INVOCATION_CWD")
            os.environ["RBNX_INVOCATION_CWD"] = directory
            try:
                values = profiles.resolve_rtabmap_overrides(
                    {"RGBD/NeighborLinkRefining": False},
                    params_file="rtabmap.yaml",
                )
            finally:
                if old is None:
                    os.environ.pop("RBNX_INVOCATION_CWD", None)
                else:
                    os.environ["RBNX_INVOCATION_CWD"] = old

        self.assertIs(values["RGBD/NeighborLinkRefining"], False)
        self.assertIs(values["RGBD/ProximityBySpace"], False)

    def test_rgbd_profile_uses_sensor_data_image_qos_only_when_enabled(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        base_params, rgbd_and_later = source.split(
            "\n    if have_rgbd:\n"
            "        # RealSense and other camera drivers commonly publish",
            1,
        )
        rgbd_params, _ = rgbd_and_later.split("\n\n    deployment_overrides", 1)

        # Lidar-only keeps the established RTAB-Map parameter baseline.
        self.assertNotIn('"qos_image"', base_params)
        self.assertNotIn('"qos_camera_info"', base_params)

        # RGB-D readers accept the sensor-data (best-effort) publishers.
        self.assertIn('"qos_image": _SENSOR_DATA_QOS', rgbd_params)
        self.assertIn('"qos_camera_info": _SENSOR_DATA_QOS', rgbd_params)
        self.assertEqual(source.count('"qos_image": _SENSOR_DATA_QOS'), 1)
        self.assertEqual(source.count('"qos_camera_info": _SENSOR_DATA_QOS'), 1)

    def test_icp_odometry_receives_only_deploy_icp_and_odom_overrides(self):
        profiles = load_profiles()
        selected = profiles.select_icp_odometry_overrides(
            {
                "Icp/MaxCorrespondenceDistance": "0.2",
                "Odom/GuessMotion": "false",
                "Odom/ResetCountdown": "1",
                "Grid/RangeMax": "10.0",
                "Rtabmap/DetectionRate": "2.0",
            }
        )
        self.assertEqual(
            selected,
            {
                "Icp/MaxCorrespondenceDistance": "0.2",
                "Odom/GuessMotion": "false",
                "Odom/ResetCountdown": "1",
            },
        )

    def test_icp_deploy_overrides_are_applied_after_internal_defaults(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        defaults = source.index('"Icp/MaxCorrespondenceDistance": "1.0"')
        forwarded = source.index(
            "select_icp_odometry_overrides(deployment_overrides)"
        )
        self.assertLess(defaults, forwarded)

    def test_requested_rtabmap_input_must_resolve(self):
        profiles = load_profiles()
        with self.assertRaisesRegex(RuntimeError, "rgbd input.*not resolved"):
            profiles.select_rtabmap_inputs(
                ["lidar", "rgbd"], {"lidar_topic": "/scanner/cloud"}
            )

    def test_rtabmap_exit_terminates_the_mapping_launch(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        self.assertIn("OnProcessExit(", source)
        self.assertIn("target_action=rtabmap_node", source)
        self.assertIn('Shutdown(reason="RTAB-Map engine exited")', source)


if __name__ == "__main__":
    unittest.main()
