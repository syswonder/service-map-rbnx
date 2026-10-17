# SPDX-License-Identifier: MulanPSL-2.0
import asyncio
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from mapping_rbnx import reference_time as clock


class ReferenceWaitTests(unittest.TestCase):
    def setUp(self):
        self.sample = [(0, 1, 1)]
        self.fake = SimpleNamespace(read=lambda: self.sample[0])
        self.patch = patch.object(clock, "_clock", self.fake)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def test_frozen_zero_does_not_expire_with_real_time(self):
        deadline = clock.Deadline(.01)
        time.sleep(.03)
        self.assertFalse(deadline.expired())
        self.sample[0] = (10_000_000, 1, 1)
        self.assertTrue(deadline.expired())

    def test_wait_completes_on_evidence_without_time_advancing(self):
        event = threading.Event()
        timer = threading.Timer(.03, event.set)
        timer.start()
        self.addCleanup(timer.join)
        self.assertTrue(clock.wait(event, .01))

    def test_reset_reconnect_and_unavailable_fail_closed(self):
        for sample in ((0, 2, 1), (0, 1, 2), None):
            self.sample[0] = (100, 1, 1)
            deadline = clock.Deadline(1.)
            self.sample[0] = sample
            with self.assertRaises(RuntimeError):
                deadline.expired()

    def test_engine_grid_wait_survives_its_wall_time_budget(self):
        try:
            from nav_msgs.msg import OccupancyGrid
        except ImportError:
            self.skipTest("ROS message bindings unavailable")
        from mapping_rbnx import engines
        message = OccupancyGrid()
        message.info.width, message.info.height = 10, 12
        closed = []
        timers = []

        class Node:
            def create_subscription(self, kind, topic, callback, qos):
                timer = threading.Timer(.03, lambda: callback(message))
                timers.append(timer)
                timer.start()
                return timer

            def destroy_subscription(self, subscription):
                closed.append(subscription)

        try:
            found, shape, stamp = engines._await_map_shape(Node(), "/map", (10, 12), .01)
        finally:
            for timer in timers:
                timer.join()
        self.assertTrue(found)
        self.assertEqual(shape, (10, 12))
        self.assertEqual(stamp, 0)
        self.assertEqual(closed, timers)

    def test_request_delivery_at_zero_is_retained_and_cached_replay_is_excluded(self):
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy
        from mapping_rbnx import engines

        message = OccupancyGrid()
        message.info.width, message.info.height = 10, 12
        subscribed = []
        closed = []

        class Node:
            def create_subscription(self, kind, topic, callback, qos):
                subscribed.append((callback, qos))
                callback(message)
                return object()

            def destroy_subscription(self, subscription):
                closed.append(subscription)

        def request():
            self.assertEqual(subscribed[0][1].durability, DurabilityPolicy.TRANSIENT_LOCAL)
            subscribed[0][0](message)
            return True, "published during request"

        found, shape, stamp = engines._await_map_shape(
            Node(), "/map", (10, 12), .01, request=request)
        self.assertEqual((found, shape, stamp), (True, (10, 12), 0))
        self.assertEqual(len(closed), 1)

        def no_delivery():
            timer = threading.Timer(.03, lambda: self.sample.__setitem__(0, (20_000_000, 1, 1)))
            timer.start()
            self.addCleanup(timer.join)
            return True, "no new grid"

        found, shape, stamp = engines._await_map_shape(
            Node(), "/map", (10, 12), .01, request=no_delivery)
        self.assertFalse(found)
        self.assertIsNone(shape)
        self.assertIsNone(stamp)

    def test_canceled_grpc_pose_wait_releases_worker_while_reference_is_frozen(self):
        import grpc
        from concurrent.futures import ThreadPoolExecutor
        import map_pb2
        import robonix_contracts_pb2_grpc as contracts
        from mapping_rbnx import atlas_bridge, map_ops

        subscribed, destroyed = threading.Event(), threading.Event()

        class Node:
            def create_subscription(self, *_args):
                subscribed.set()
                return object()

            def destroy_subscription(self, _sub):
                destroyed.set()

        executor = ThreadPoolExecutor(max_workers=2)
        server = grpc.server(executor)
        contracts.add_RobonixServiceMapGetPoseServicer_to_server(
            atlas_bridge._GetPoseServicer(), server)
        port = server.add_insecure_port("127.0.0.1:0")
        server.start()
        try:
            with patch.object(map_ops, "_get_node", return_value=Node()):
                with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
                    method = contracts.RobonixServiceMapGetPoseStub(channel).GetPose
                    stale = method(map_pb2.GetPose_Request(), timeout=2., metadata=(
                        ("robonix-reference-generation", "2"),))
                    self.assertFalse(stale.ok)
                    self.assertFalse(subscribed.is_set())
                    call = method.future(map_pb2.GetPose_Request())
                    self.assertTrue(subscribed.wait(2.0))
                    call.cancel()
                    self.assertTrue(destroyed.wait(1.0))
                    self.assertEqual(self.sample[0], (0, 1, 1))
        finally:
            server.stop(0).wait()
            executor.shutdown()

    def test_initialization_reports_deferred_while_work_continues(self):
        from robonix_api import Deferred, Ok
        from mapping_rbnx import atlas_bridge

        release = threading.Event()
        with (patch.object(atlas_bridge, "_INIT_THREAD", None),
              patch.object(atlas_bridge, "_INIT_RESULT", None),
              patch.object(atlas_bridge, "_INIT_CONFIG", None),
              patch.object(clock, "configure", return_value=True),
              patch.object(atlas_bridge, "_initialize", side_effect=lambda _cfg: release.wait(2.) and Ok())):
            try:
                self.assertIsInstance(atlas_bridge.init({}), Deferred)
                self.assertIsInstance(atlas_bridge.init({}), Deferred)
            finally:
                release.set()
                atlas_bridge._INIT_THREAD.join(2.)
            self.assertIsInstance(atlas_bridge.init({}), Ok)

    def test_async_cancellation_joins_a_frozen_reference_worker(self):
        entered, exited = threading.Event(), threading.Event()

        def worker():
            entered.set()
            try:
                return clock.wait(threading.Event(), .01)
            finally:
                exited.set()

        async def exercise():
            task = asyncio.create_task(clock.run(worker))
            self.assertTrue(await asyncio.to_thread(entered.wait, 1.))
            await asyncio.sleep(.03)
            self.assertFalse(task.done())
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await task
            self.assertTrue(exited.is_set())

        asyncio.run(exercise())

    def test_cached_pose_at_zero_remains_readable_during_pause(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from mapping_rbnx import map_ops, webui
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = 2.
        with (patch.object(webui, "_latest", dict.fromkeys(webui._latest)),
              patch.object(webui, "_reference", None),
              patch.object(webui, "_map_frame", None),
              patch.object(map_ops, "_get_node", return_value=object())):
            webui._receive("pose", message)
            result = map_ops.get_pose_impl()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["x"], 2.)
        self.assertEqual(self.sample[0], (0, 1, 1))

    def test_scan_computation_keeps_a_real_budget_while_reference_is_frozen(self):
        import numpy as np
        from mapping_rbnx import global_match

        with patch.object(global_match.time, "monotonic", side_effect=[0., 61.]):
            with self.assertRaises(TimeoutError):
                global_match.distance_field(np.zeros((2, 2), dtype=bool), .05)
        self.assertEqual(self.sample[0], (0, 1, 1))

    def test_reset_during_pose_settle_releases_publisher_without_publishing(self):
        from builtin_interfaces.msg import Time
        from mapping_rbnx import map_ops
        published, closed = [], []
        publisher = SimpleNamespace(publish=published.append)
        node = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
            create_publisher=lambda *_args: publisher,
            destroy_publisher=closed.append,
        )
        with (patch.object(map_ops, "_get_node", return_value=node),
              patch.object(map_ops.time, "sleep", side_effect=lambda _s: self.sample.__setitem__(0, (0, 2, 1)))):
            with self.assertRaises(RuntimeError):
                map_ops.pose_estimate_impl(0., 0., 0.)
        self.assertEqual(published, [])
        self.assertEqual(closed, [publisher])

    def test_canceled_save_cannot_publish_staged_directory(self):
        import tempfile
        from pathlib import Path
        from mapping_rbnx import map_ops

        with tempfile.TemporaryDirectory() as root:
            staged = Path(root) / "staged"
            staged.mkdir()
            target = Path(root) / "target"
            active = [True]
            with self.assertRaises(RuntimeError):
                with clock.operation(lambda: active[0]):
                    active[0] = False
                    map_ops._atomic_publish_map_dir(str(staged), str(target))
            self.assertTrue(staged.exists())
            self.assertFalse(target.exists())

    def test_operation_detects_reset_between_individual_waits(self):
        with self.assertRaises(RuntimeError):
            with clock.operation():
                self.sample[0] = (0, 2, 1)

    def test_cancellation_restores_the_previous_context(self):
        active = [True]
        with self.assertRaises(RuntimeError):
            with clock.operation(lambda: active[0]):
                active[0] = False
                clock.reading()
        self.assertEqual(clock.reading(), (0, 1, 1))



class RuntimeCleanupTest(unittest.TestCase):
    def test_map_ros_executor_is_stopped_joined_and_destroyed(self):
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        from mapping_rbnx import map_ops
        stop = threading.Event()
        thread = threading.Thread(target=stop.wait)
        thread.start()
        node = SimpleNamespace(_robonix_executor=Mock(shutdown=lambda **_: stop.set()),
                               _robonix_spin=thread, destroy_node=Mock())
        try:
            with patch.object(map_ops, "_node", node):
                map_ops._close_runtime()
                self.assertIsNone(map_ops._node)
                self.assertFalse(thread.is_alive())
                node.destroy_node.assert_called_once()
        finally:
            stop.set()
            thread.join(1)

if __name__ == "__main__":
    unittest.main()
