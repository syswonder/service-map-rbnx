# SPDX-License-Identifier: MulanPSL-2.0
"""Localizer slot: configuration validation and the launch-argument contract.

Everything here runs without ROS: the pieces that need a live node (starting the
stack, the global-localization service call) are exercised in the Webots
benchmark, not in unit tests.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from mapping_rbnx import localizers  # noqa: E402


class LocalizerConfigTests(unittest.TestCase):
    def tearDown(self):
        localizers.configure({})

    def test_default_is_off_so_existing_deployments_do_not_change(self):
        self.assertEqual(localizers.configure({}), "none")
        self.assertFalse(localizers.enabled())

    def test_named_localizers_enable_the_slot(self):
        for name in ("amcl", "beluga", " AMCL "):
            self.assertEqual(localizers.configure({"localizer": name}), name.strip().lower())
            self.assertTrue(localizers.enabled())

    def test_unknown_localizer_fails_at_configuration(self):
        with self.assertRaises(ValueError) as ctx:
            localizers.configure({"localizer": "amcl2"})
        self.assertIn("none, amcl, beluga", str(ctx.exception))

    def test_scan_topic_and_frames_reach_the_launch(self):
        localizers.configure({
            "localizer": "amcl", "scan_topic": "/tiago/scan",
            "base_frame": "base_footprint", "odom_frame": "odom_combined",
            "use_sim_time": True, "min_particles": 300, "max_particles": 900,
        })
        self.assertEqual(localizers._CONFIG["scan_topic"], "/tiago/scan")
        self.assertEqual(localizers._CONFIG["base_frame"], "base_footprint")
        self.assertEqual(localizers._CONFIG["odom_frame"], "odom_combined")
        self.assertTrue(localizers._CONFIG["use_sim_time"])
        self.assertEqual((localizers._CONFIG["min_particles"], localizers._CONFIG["max_particles"]), (300, 900))

    def test_start_without_a_saved_grid_is_refused_by_name(self):
        localizers.configure({"localizer": "amcl"})
        ok, detail = localizers.start("/nonexistent/map/dir", "nowhere")
        self.assertFalse(ok)
        self.assertIn("occupancy.yaml", detail)

    def test_start_is_refused_when_the_slot_is_off(self):
        localizers.configure({"localizer": "none"})
        ok, detail = localizers.start("/tmp", "any")
        self.assertFalse(ok)
        self.assertIn("localizer: none", detail)


class LaunchFileTests(unittest.TestCase):
    def test_launch_file_ships_with_the_package(self):
        self.assertTrue(os.path.isfile(os.path.join(os.path.dirname(__file__),
                                                    "launch", "localization_2d.launch.py")))

    def test_launch_declares_every_argument_the_module_passes(self):
        src = open(os.path.join(os.path.dirname(__file__), "launch",
                                "localization_2d.launch.py"), encoding="utf-8").read()
        for arg in ("localizer", "map_yaml", "scan_topic", "base_frame", "odom_frame",
                    "global_frame", "use_sim_time", "min_particles", "max_particles"):
            self.assertIn(f'DeclareLaunchArgument("{arg}"', src, f"{arg} is passed but not declared")


if __name__ == "__main__":
    unittest.main()
