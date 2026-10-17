# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import os
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from mapping_rbnx import map_ops


class LoadMapTransactionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        map_dir = os.path.join(self.tmp.name, "target")
        os.makedirs(map_dir)
        self.saved_db = os.path.join(map_dir, "rtabmap.db")
        open(self.saved_db, "wb").close()

    def test_localization_precedes_database_load_and_publish_is_verified(self):
        order = []
        barrier = {"subscription": object()}

        with (
            patch.object(map_ops, "MAPS_DIR", self.tmp.name),
            patch.object(map_ops, "_sqlite_quick_check", return_value=(True, "ok")),
            patch.object(map_ops, "_get_node", return_value=object()),
            patch.object(
                map_ops,
                "_runtime_db_copy",
                side_effect=lambda *_: order.append("copy") or "/runtime/target.db",
            ),
            patch.object(
                map_ops, "_set_mode", side_effect=lambda *_: order.append("mode") or (True, "ok")
            ),
            patch.object(map_ops.lifecycle, "set_mode",
                         side_effect=lambda *_a, **_k: None),
            patch.object(
                map_ops,
                "_load_database",
                side_effect=lambda *_: order.append("load") or (True, "ok"),
            ),
            patch.object(
                map_ops,
                "_publish_full_map",
                side_effect=lambda *_args, **_kwargs: order.append("publish")
                or (True, "published optimized global map"),
            ),
            patch.object(
                map_ops,
                "_begin_target_map_wait",
                side_effect=lambda *_: order.append("subscribe") or barrier,
            ),
            patch.object(
                map_ops,
                "_finish_target_map_wait",
                side_effect=lambda *_: order.append("verify")
                or (True, "verified fresh occupancy"),
            ),
            patch.object(
                map_ops.lifecycle,
                "set_state",
                side_effect=lambda *_args, **_kwargs: order.append("identity"),
            ),
        ):
            result = map_ops.load_map_impl("target")

        self.assertTrue(result["ok"], result)
        self.assertEqual(
            order,
            ["copy", "mode", "load", "identity", "subscribe", "publish", "verify"],
        )
        self.assertIn("verified fresh occupancy", result["detail"])

    def test_mode_switch_failure_never_loads_database(self):
        with (
            patch.object(map_ops, "MAPS_DIR", self.tmp.name),
            patch.object(map_ops, "_sqlite_quick_check", return_value=(True, "ok")),
            patch.object(map_ops, "_get_node", return_value=object()),
            patch.object(map_ops, "_runtime_db_copy", return_value="/runtime/target.db"),
            patch.object(map_ops, "_set_mode", return_value=(False, "mode failed")),
            patch.object(map_ops, "_load_database") as load,
        ):
            result = map_ops.load_map_impl("target")

        self.assertFalse(result["ok"])
        self.assertIn("before load", result["detail"])
        load.assert_not_called()

    @staticmethod
    def _occupancy(width, height, resolution, data):
        return SimpleNamespace(
            info=SimpleNamespace(
                width=width,
                height=height,
                resolution=resolution,
                origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
            ),
            data=data,
        )

    def test_full_map_publish_can_deliver_first_grid_after_discovery(self):
        callback = []
        closed = []

        class Node:
            def create_subscription(self, kind, topic, receive, qos):
                callback.append(receive)
                return object()

            def destroy_subscription(self, subscription):
                closed.append(subscription)

        node = Node()
        barrier = map_ops._begin_target_map_wait(node)
        self.assertFalse(barrier["event"].is_set())

        def publish(*_args, **_kwargs):
            callback[0](self._occupancy(2, 2, .05, [0, 100, 0, 0]))
            return True, "published after discovery"

        with (patch.object(map_ops, "_publish_full_map", side_effect=publish) as requested,
              patch.object(map_ops.time, "monotonic", side_effect=[0., 6., 6.])):
            ready, detail = map_ops._finish_target_map_wait(node, barrier, 30.)
        self.assertTrue(ready, detail)
        requested.assert_called_once()
        self.assertEqual(len(closed), 1)

    def test_occupancy_readiness_accepts_optimized_non_empty_map(self):
        msg = self._occupancy(3, 2, 0.05, [-1, 0, 0, 100, -1, 0])

        ready, summary = map_ops._occupancy_sample_ready(msg)

        self.assertTrue(ready)
        self.assertIn("3x2@0.050000", summary)
        self.assertIn("known=4", summary)

    def test_occupancy_readiness_rejects_unknown_or_malformed_grid(self):
        unknown = self._occupancy(3, 2, 0.05, [-1] * 6)
        malformed = self._occupancy(3, 2, 0.05, [0] * 5)
        no_resolution = self._occupancy(3, 2, 0.0, [0] * 6)

        self.assertFalse(map_ops._occupancy_sample_ready(unknown)[0])
        self.assertFalse(map_ops._occupancy_sample_ready(malformed)[0])
        self.assertFalse(map_ops._occupancy_sample_ready(no_resolution)[0])


class SlamLoadTransactionTest(unittest.TestCase):
    setUp = LoadMapTransactionTest.setUp

    def test_reference_reset_during_engine_stop_cannot_publish_map_identity(self):
        sample = [(0, 1, 1)]

        def stop():
            sample[0] = (0, 2, 1)
            return True, "stopped"

        ops = SimpleNamespace(graph_ready=lambda _path: (True, "ok"), stop_engine=stop)
        with (patch.object(map_ops.reference_time, "_clock", SimpleNamespace(read=lambda: sample[0])),
              patch.object(map_ops, "MAPS_DIR", self.tmp.name),
              patch.object(map_ops, "active_algo", return_value="slam_toolbox"),
              patch.object(map_ops, "map_engine", return_value="slam_toolbox"),
              patch.object(map_ops, "_engine", return_value=ops),
              patch.object(map_ops, "_get_node", return_value=object()),
              patch.object(map_ops.localizers, "enabled", return_value=True),
              patch.object(map_ops.lifecycle, "set_state") as publish,
              patch.object(map_ops.lifecycle, "mark_reset") as reset):
            with self.assertRaises(RuntimeError):
                map_ops.load_map_impl("target")
        publish.assert_not_called()
        reset.assert_not_called()



if __name__ == "__main__":
    unittest.main()
