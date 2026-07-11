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

    def test_ranger_profile_matches_v01_database_parameters(self):
        profiles = load_profiles()
        values = profiles.resolve_rtabmap_overrides("ranger_mini_v3", None)
        self.assertEqual(values["Rtabmap/DetectionRate"], 5.0)
        self.assertEqual(values["RGBD/LinearUpdate"], 0.05)
        self.assertEqual(values["RGBD/AngularUpdate"], 0.05)
        self.assertIs(values["RGBD/CreateOccupancyGrid"], True)
        self.assertIs(values["Mem/NotLinkedNodesKept"], True)

    def test_explicit_values_override_profile(self):
        profiles = load_profiles()
        values = profiles.resolve_rtabmap_overrides(
            "ranger_mini_v3", {"Rtabmap/DetectionRate": 2.0}
        )
        self.assertEqual(values["Rtabmap/DetectionRate"], 2.0)

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

    def test_requested_rtabmap_input_must_resolve(self):
        profiles = load_profiles()
        with self.assertRaisesRegex(RuntimeError, "rgbd input.*not resolved"):
            profiles.select_rtabmap_inputs(
                ["lidar", "rgbd"], {"lidar_topic": "/scanner/cloud"}
            )


if __name__ == "__main__":
    unittest.main()
