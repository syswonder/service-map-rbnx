# SPDX-License-Identifier: MulanPSL-2.0
import threading
import time
import unittest
from unittest.mock import Mock, patch

from mapping_rbnx import lifecycle, map_ops, reference_time


class LifecycleShutdownTest(unittest.TestCase):
    def test_shutdown_cancels_retry_and_prevents_ros_node_recreation(self):
        with patch.object(lifecycle, "_stopping", threading.Event()), \
                patch.object(lifecycle, "_retry_pending", False), \
                patch.object(lifecycle, "_retry_attempts", 0), \
                patch.object(lifecycle, "_retry_timer", None), \
                patch.object(lifecycle, "_disabled", False), \
                patch.object(lifecycle, "_RETRY_PERIOD_S", .05), \
                patch.object(lifecycle, "_publish") as publish:
            lifecycle._schedule_retry()
            timer = lifecycle._retry_timer
            lifecycle._stop()
            time.sleep(.07)
            self.assertFalse(timer.is_alive())
            publish.assert_not_called()
            lifecycle._schedule_retry()
            self.assertIsNone(lifecycle._retry_timer)
        stopped = threading.Event()
        stopped.set()
        with patch.object(reference_time, "_stopping", stopped), \
                patch.object(map_ops, "_node", None):
            self.assertIsNone(map_ops._get_node())
            self.assertIsNone(map_ops._node)


if __name__ == "__main__":
    unittest.main()
