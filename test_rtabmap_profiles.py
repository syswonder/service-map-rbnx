import importlib.util
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


class RtabmapProfileTest(unittest.TestCase):
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

    def test_ranger_profile_preserves_v01_baseline_and_filters_self_points(self):
        profiles = load_profiles()
        values = profiles.resolve_rtabmap_overrides("ranger_mini_v3", None)
        self.assertEqual(values["Rtabmap/DetectionRate"], 5.0)
        self.assertEqual(values["RGBD/LinearUpdate"], 0.05)
        self.assertEqual(values["RGBD/AngularUpdate"], 0.05)
        self.assertIs(values["RGBD/CreateOccupancyGrid"], True)
        self.assertIs(values["Mem/NotLinkedNodesKept"], True)
        self.assertEqual(values["Grid/FootprintLength"], 0.84)
        self.assertEqual(values["Grid/FootprintWidth"], 0.60)
        self.assertEqual(values["Grid/RangeMin"], 0.10)

        # Regression for the 2026-07-13 temporary map: the invalid point was
        # emitted at the lidar viewpoint (0.18, 0.0), inside the Ranger body.
        self.assertLess(0.18, values["Grid/FootprintLength"] / 2.0)
        self.assertLess(0.0, values["Grid/FootprintWidth"] / 2.0)

    def test_explicit_values_override_profile(self):
        profiles = load_profiles()
        values = profiles.resolve_rtabmap_overrides(
            "ranger_mini_v3",
            {"Rtabmap/DetectionRate": 2.0, "Grid/RangeMin": 0.20},
        )
        self.assertEqual(values["Rtabmap/DetectionRate"], 2.0)
        self.assertEqual(values["Grid/RangeMin"], 0.20)

    def test_webots_profile_preserves_external_odom_during_rotation(self):
        profiles = load_profiles()
        values = profiles.resolve_rtabmap_overrides("webots_tiago", None)

        self.assertEqual(values["Rtabmap/DetectionRate"], 5.0)
        self.assertEqual(values["RGBD/LinearUpdate"], 0.03)
        self.assertEqual(values["RGBD/AngularUpdate"], 0.03)
        self.assertIs(values["RGBD/NeighborLinkRefining"], False)
        self.assertIs(values["RGBD/ProximityBySpace"], False)
        self.assertEqual(values["Reg/Strategy"], 0)

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

    def test_ranger_inputs_drop_rgbd_without_hiding_atlas_capabilities(self):
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

    def test_ranger_visual_fusion_keeps_lidar_rgbd_and_external_odom(self):
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

    def test_raw_livox_imu_is_filtered_before_icp(self):
        source = (ROOT / "launch" / "rtabmap_2d.launch.py").read_text()
        self.assertIn('package="imu_filter_madgwick"', source)
        self.assertIn('(\"imu/data_raw\", imu_topic)', source)
        self.assertIn('(\"imu\", filtered_imu_topic)', source)

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
