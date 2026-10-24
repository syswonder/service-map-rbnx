# SPDX-License-Identifier: MulanPSL-2.0
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

try:
    from launch import LaunchContext
    from launch.actions import ExecuteProcess
    from launch.utilities import perform_substitutions
    from rclpy.parameter import parameter_value_to_python
    from ros2param.api import get_parameter_value
except ImportError:
    LaunchContext = None


ROOT = Path(__file__).resolve().parent


@unittest.skipIf(LaunchContext is None, "requires ROS 2 launch_ros")
class RtabmapLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            "mapping_rtabmap_launch", ROOT / "launch" / "rtabmap_2d.launch.py"
        )
        cls.module = importlib.util.module_from_spec(spec)
        with patch.object(sys, "path", [str(ROOT / "src"), *sys.path]):
            spec.loader.exec_module(cls.module)

    def configure(self, **overrides):
        context = LaunchContext()
        for argument in self.module.generate_launch_description().get_launch_arguments():
            context.launch_configurations[argument.name] = perform_substitutions(
                context, argument.default_value
            )
        context.launch_configurations.update(
            scan_topic="/scanner", odom_topic="/odom", enable_viz="false"
        )
        context.launch_configurations.update(overrides)
        with patch.object(self.module, "ExecuteProcess", wraps=ExecuteProcess) as factory:
            actions = self.module._make_nodes(context)
        container = next(
            action for action in actions
            if isinstance(action, self.module.Node)
            and action.node_executable == "component_container"
        )
        command = next(
            call.kwargs["cmd"] for call in factory.call_args_list
            if "rtabmap_slam::CoreWrapper" in call.kwargs["cmd"]
        )
        params = {
            command[index + 1].split(":=", 1)[0]:
            parameter_value_to_python(get_parameter_value(
                string_value=command[index + 1].split(":=", 1)[1]
            ))
            for index, argument in enumerate(command) if argument == "-p"
        }
        return container, command, params

    def test_engine_uses_single_threaded_component_with_sensor_remappings(self):
        container, command, params = self.configure()
        self.assertEqual(container.node_executable, "component_container")
        self.assertIn("rtabmap_slam::CoreWrapper", command)
        self.assertEqual(command[:3], ["timeout", "--kill-after=5s", "120s"])
        self.assertEqual(command[command.index("--node-name") + 1], "rtabmap")
        self.assertIn("scan:=/scanner", command)
        self.assertIn("odom:=/odom", command)
        self.assertTrue(params["delete_db_on_start"])

    def test_named_mapping_preserves_database_unless_reset_requested(self):
        for reset, expected in (("false", False), ("true", True)):
            with self.subTest(reset=reset):
                _, _, params = self.configure(
                    database_path="/tmp/mapping-test.db", reset_map=reset
                )
                self.assertEqual(params["database_path"], "/tmp/mapping-test.db")
                self.assertIs(params["delete_db_on_start"], expected)
                self.assertEqual(params["Mem/IncrementalMemory"], "true")

    def test_localization_preserves_database_even_with_reset_requested(self):
        _, _, params = self.configure(
            database_path="/tmp/mapping-test.db", map_mode="localization", reset_map="true"
        )
        self.assertFalse(params["delete_db_on_start"])
        self.assertEqual(params["Mem/IncrementalMemory"], "false")
        self.assertEqual(params["Mem/InitWMWithAllNodes"], "true")

    def test_native_deployment_parameters_keep_precedence_and_types(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "overrides.json"
            source.write_text(json.dumps({
                "delete_db_on_start": False, "qos_scan": 2, "Grid/RayTracing": True,
                "wait_for_transform": 1e-6,
            }))
            _, _, params = self.configure(
                database_path="/tmp/mapping-test.db", rtabmap_overrides_file=str(source)
            )
        self.assertIs(params["delete_db_on_start"], False)
        self.assertEqual(params["qos_scan"], 2)
        self.assertEqual(params["Grid/RayTracing"], "true")
        self.assertIsInstance(params["wait_for_transform"], float)
        self.assertEqual(params["wait_for_transform"], 1e-6)

    def test_forced_reset_takes_precedence_over_false_native_override(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "overrides.json"
            source.write_text(json.dumps({"delete_db_on_start": False}))
            for extra in ({}, {"database_path": "/tmp/mapping-test.db", "reset_map": "true"}):
                with self.subTest(extra=extra):
                    _, _, params = self.configure(
                        rtabmap_overrides_file=str(source), **extra
                    )
                    self.assertTrue(params["delete_db_on_start"])

    def test_failed_component_load_shuts_down_but_success_keeps_engine_running(self):
        self.assertEqual(
            self.module._on_component_load_exit(SimpleNamespace(returncode=0), None), []
        )
        actions = self.module._on_component_load_exit(SimpleNamespace(returncode=1), None)
        self.assertEqual(len(actions), 1)
        self.assertIn("component failed to load", actions[0].event.reason)


if __name__ == "__main__":
    unittest.main()
