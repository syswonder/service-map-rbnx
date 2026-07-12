# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import os
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

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
            patch.object(map_ops, "set_current_mode"),
            patch.object(
                map_ops,
                "_load_database",
                side_effect=lambda *_: order.append("load") or (True, "ok"),
            ),
            patch.object(
                map_ops,
                "_begin_target_map_wait",
                side_effect=lambda *_: order.append("subscribe") or barrier,
            ),
            patch.object(
                map_ops,
                "_publish_full_map",
                side_effect=lambda *_args, **_kwargs: order.append("publish")
                or (True, "published optimized global map"),
            ),
            patch.object(
                map_ops,
                "_finish_target_map_wait",
                side_effect=lambda *_: order.append("verify")
                or (True, "verified target occupancy"),
            ),
        ):
            result = map_ops.load_map_impl("target")

        self.assertTrue(result["ok"], result)
        self.assertEqual(order, ["copy", "mode", "load", "subscribe", "publish", "verify"])
        self.assertIn("verified target occupancy", result["detail"])

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

    def test_occupancy_similarity_accepts_small_edge_rebuild(self):
        expected = np.full((100, 100), 205, dtype=np.uint8)
        expected[20:40, 20:40] = 0
        expected[50:80, 50:80] = 254
        observed = expected.copy()
        observed.flat[:1] = 254

        agreement, occupied_iou, free_iou = map_ops._occupancy_similarity(
            expected, observed
        )

        self.assertGreaterEqual(agreement, 0.9999)
        self.assertGreaterEqual(occupied_iou, 0.995)
        self.assertGreaterEqual(free_iou, 0.995)

    def test_occupancy_similarity_rejects_different_same_size_map(self):
        expected = np.full((100, 100), 205, dtype=np.uint8)
        expected[20:40, 20:40] = 0
        observed = np.full((100, 100), 205, dtype=np.uint8)
        observed[60:80, 60:80] = 0

        agreement, occupied_iou, free_iou = map_ops._occupancy_similarity(
            expected, observed
        )

        self.assertLess(agreement, 0.9999)
        self.assertLess(occupied_iou, 0.995)
        self.assertEqual(free_iou, 1.0)


if __name__ == "__main__":
    unittest.main()
