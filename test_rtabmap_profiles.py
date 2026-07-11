import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def load_profiles():
    spec = importlib.util.spec_from_file_location(
        "rtabmap_profiles", ROOT / "src" / "mapping_rbnx" / "profiles.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RtabmapProfileTest(unittest.TestCase):
    def test_ranger_profile_matches_v01_database_parameters(self):
        profiles = load_profiles()
        values = profiles.resolve_rtabmap_overrides("ranger_mini_v3", None)
        self.assertEqual(values["Grid/Sensor"], 0)
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


if __name__ == "__main__":
    unittest.main()
