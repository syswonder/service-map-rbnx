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


class MapFrameOwnershipTests(unittest.TestCase):
    """Only one node may publish map -> odom."""

    def test_every_engine_can_hand_the_frame_over(self):
        # RTAB-Map's implementations are injected by map_ops at import time.
        from mapping_rbnx import engines, map_ops  # noqa: F401
        for algo in ("slam_toolbox", "rtabmap"):
            ops = engines.engine_for(algo)
            self.assertIsNotNone(ops, algo)
            self.assertTrue(callable(getattr(ops, "yield_map_frame", None)), algo)

    def test_rtabmap_keeps_the_frame_because_it_localizes_itself(self):
        from mapping_rbnx import engines, map_ops  # noqa: F401
        ok, detail = engines.engine_for("rtabmap").yield_map_frame(None, 1.0)
        self.assertTrue(ok)
        self.assertIn("localization mode", detail)

    def test_no_slam_toolbox_process_means_the_frame_is_already_free(self):
        from mapping_rbnx import engines
        ok, detail = engines.engine_for("slam_toolbox").yield_map_frame(None, 1.0)
        self.assertTrue(ok)                       # nothing is running here
        self.assertIn("was not running", detail)

    def test_a_process_that_will_not_die_is_reported_rather_than_assumed_gone(self):
        from unittest import mock
        from mapping_rbnx import engines
        ops = engines.engine_for("slam_toolbox")
        found = mock.Mock(stdout="4242\n")
        with mock.patch("subprocess.run", return_value=found), \
             mock.patch("os.kill"), \
             mock.patch.object(engines, "_pid_alive", return_value=True):
            ok, detail = ops.yield_map_frame(None, 1.0)
        self.assertFalse(ok)
        self.assertIn("4242", detail)
        self.assertIn("fight", detail)


class ConvergenceTests(unittest.TestCase):
    """The spread the UI reports, read straight off AMCL's covariance."""

    @staticmethod
    def _msg(var_x, var_y, var_yaw):
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = var_x, var_y, var_yaw

        class Msg:
            class pose:
                covariance = cov
        return Msg()

    def test_spread_reads_position_and_heading_from_the_covariance(self):
        pos, yaw = localizers.spread(self._msg(0.09, 0.16, 0.04))
        self.assertAlmostEqual(pos, 0.5)        # sqrt(0.09 + 0.16)
        self.assertAlmostEqual(yaw, 0.2)

    def test_a_filter_spread_over_the_map_is_not_converged(self):
        pos, yaw = localizers.spread(self._msg(9.0, 9.0, 1.0))
        self.assertEqual(localizers.convergence_state(pos, yaw), "converging")

    def test_a_tight_filter_is_converged(self):
        pos, yaw = localizers.spread(self._msg(0.01, 0.01, 0.004))
        self.assertEqual(localizers.convergence_state(pos, yaw), "converged")

    def test_heading_alone_can_hold_it_back(self):
        # Position is tight but the robot does not know which way it faces —
        # driving on that is how you end up mapping into a wall.
        self.assertEqual(localizers.convergence_state(0.05, 0.9), "converging")

    def test_negative_variance_does_not_raise(self):
        pos, yaw = localizers.spread(self._msg(-1.0, 0.04, -1.0))
        self.assertAlmostEqual(pos, 0.2)
        self.assertAlmostEqual(yaw, 0.0)


class LaunchFileTests(unittest.TestCase):
    def test_launch_file_ships_with_the_package(self):
        self.assertTrue(os.path.isfile(os.path.join(os.path.dirname(__file__),
                                                    "launch", "localization_2d.launch.py")))

    def test_launch_declares_every_argument_the_module_passes(self):
        with open(os.path.join(os.path.dirname(__file__), "launch",
                               "localization_2d.launch.py"), encoding="utf-8") as fh:
            src = fh.read()
        for arg in ("localizer", "map_yaml", "scan_topic", "base_frame", "odom_frame",
                    "global_frame", "use_sim_time", "min_particles", "max_particles"):
            self.assertIn(f'DeclareLaunchArgument("{arg}"', src, f"{arg} is passed but not declared")



class EngineRegistryTests(unittest.TestCase):
    """Every engine that can persist maps must name its artifacts."""

    def test_slam_toolbox_declares_its_graph_files(self):
        from mapping_rbnx import engines
        ops = engines.engine_for("slam_toolbox")
        self.assertIsNotNone(ops)
        self.assertEqual(set(ops.graph_files), {"posegraph.posegraph", "posegraph.data"})

    def test_missing_graph_is_reported_by_name(self):
        from mapping_rbnx import engines
        ok, detail = engines.engine_for("slam_toolbox").graph_ready("/nonexistent")
        self.assertFalse(ok)
        self.assertIn("posegraph", detail)

    def test_engines_without_persistence_are_absent_rather_than_broken(self):
        from mapping_rbnx import engines
        self.assertIsNone(engines.engine_for("dlio"))
        self.assertEqual(engines.graph_files_for("dlio"), ())

class MapPersistenceTests(unittest.TestCase):
    """Backend tagging: a saved map names its engine, and a map from another
    engine is refused by name instead of failing inside the load."""

    def setUp(self):
        import tempfile
        from mapping_rbnx import map_ops
        self.map_ops = map_ops
        self.tmp = tempfile.mkdtemp()
        self._saved_maps_dir = map_ops.MAPS_DIR
        map_ops.MAPS_DIR = self.tmp
        self._saved_algo = os.environ.get("MAPPING_ALGO")

    def tearDown(self):
        import shutil
        self.map_ops.MAPS_DIR = self._saved_maps_dir
        if self._saved_algo is None:
            os.environ.pop("MAPPING_ALGO", None)
        else:
            os.environ["MAPPING_ALGO"] = self._saved_algo
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _map(self, name, files, meta=""):
        d = os.path.join(self.tmp, name)
        os.makedirs(d, exist_ok=True)
        for f in files:
            with open(os.path.join(d, f), "wb") as fh:
                fh.write(b"x" * 16)
        if meta:
            with open(os.path.join(d, "meta.yaml"), "w", encoding="utf-8") as fh:
                fh.write(meta)
        return d

    def test_meta_records_the_engine_and_keeps_the_other_fields(self):
        d = self._map("m", (), "map_id: m\nnote: hallway\n")
        self.map_ops._write_meta_fields(d, {"engine": "slam_toolbox"})
        meta = self.map_ops.read_meta(d)
        self.assertEqual(meta["engine"], "slam_toolbox")
        self.assertEqual((meta["map_id"], meta["note"]), ("m", "hallway"))

    def test_maps_saved_before_the_field_existed_read_as_rtabmap(self):
        d = self._map("old", ("rtabmap.db",), "map_id: old\n")
        self.assertEqual(self.map_ops.map_engine(d), "rtabmap")
        self.assertEqual(self.map_ops.map_engine(self._map("empty", ())), "")

    def test_load_refuses_a_map_built_by_another_engine(self):
        self._map("stmap", ("posegraph.posegraph", "posegraph.data"),
                  "map_id: stmap\nengine: slam_toolbox\n")
        os.environ["MAPPING_ALGO"] = "rtabmap"
        out = self.map_ops.load_map_impl("stmap")
        self.assertFalse(out["ok"])
        self.assertIn("slam_toolbox", out["detail"])
        self.assertIn("rtabmap", out["detail"])

    def test_listing_names_the_engine_and_which_maps_load_here(self):
        self._map("stmap", ("posegraph.posegraph", "posegraph.data"),
                  "map_id: stmap\nengine: slam_toolbox\n")
        self._map("rtmap", ("rtabmap.db",), "map_id: rtmap\nengine: rtabmap\n")
        os.environ["MAPPING_ALGO"] = "slam_toolbox"
        import json
        rows = {r["map_id"]: r for r in json.loads(self.map_ops.list_maps_impl()["maps_json"])}
        self.assertEqual(rows["stmap"]["engine"], "slam_toolbox")
        self.assertTrue(rows["stmap"]["loadable_here"])
        self.assertEqual(rows["stmap"]["artifact_size"], 32)   # both graph files
        self.assertFalse(rows["rtmap"]["loadable_here"])
        self.assertIn("this deployment runs slam_toolbox", rows["rtmap"]["artifact_detail"])

    def test_missing_graph_file_is_not_advertised_as_loadable(self):
        self._map("half", ("posegraph.posegraph",), "map_id: half\nengine: slam_toolbox\n")
        os.environ["MAPPING_ALGO"] = "slam_toolbox"
        import json
        row = json.loads(self.map_ops.list_maps_impl()["maps_json"])[0]
        self.assertFalse(row["has_spatial_artifact"])
        self.assertFalse(row["loadable_here"])


class SlamToolboxParamTests(unittest.TestCase):
    """A mistyped scan-matching knob must fail at init, not at map time."""

    def setUp(self):
        from mapping_rbnx import profiles
        self.bridge = profiles

    def test_unset_params_keep_the_launch_defaults(self):
        self.assertEqual(self.bridge.resolve_slam_toolbox_overrides(None), {})
        self.assertEqual(self.bridge.resolve_slam_toolbox_overrides({}), {})

    def test_known_keys_reach_the_resolved_file(self):
        out = self.bridge.resolve_slam_toolbox_overrides({"min_travel_m": 0.15, "scan_buffer": 40})
        self.assertEqual(out, {"slam_toolbox_min_travel_m": "0.15",
                               "slam_toolbox_scan_buffer": "40.0"})

    def test_typo_and_bad_values_are_refused_by_name(self):
        for bad, expect in (({"min_travel": 0.1}, "unknown slam_toolbox_params"),
                            ({"scan_buffer": "many"}, "is not a number"),
                            ({"min_travel_m": 0}, "must be positive"),
                            ([("min_travel_m", 1)], "must be a mapping")):
            with self.assertRaises(RuntimeError) as ctx:
                self.bridge.resolve_slam_toolbox_overrides(bad)
            self.assertIn(expect, str(ctx.exception))

    def test_start_engine_forwards_every_key_the_bridge_writes(self):
        with open(os.path.join(os.path.dirname(__file__), "scripts", "start_engine.sh"),
                  encoding="utf-8") as fh:
            src = fh.read()
        for key in self.bridge.SLAM_TOOLBOX_KEYS.values():
            self.assertIn(key, src, f"{key} is written but start_engine.sh never reads it")


if __name__ == "__main__":
    unittest.main()
