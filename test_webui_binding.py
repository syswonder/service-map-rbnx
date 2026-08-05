import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock
from urllib import error, request


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

from mapping_rbnx import webui  # noqa: E402


class WebUiBindingTest(unittest.TestCase):
    def test_container_forwards_loopback_safe_webui_host(self):
        source = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        self.assertIn(
            '-e MAPPING_WEBUI_HOST="${MAPPING_WEBUI_HOST:-127.0.0.1}"',
            source,
        )

    def test_webui_default_is_loopback(self):
        source = (ROOT / "src" / "mapping_rbnx" / "webui.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'os.environ.get("MAPPING_WEBUI_HOST", "127.0.0.1")', source
        )
        self.assertNotIn(
            'os.environ.get("MAPPING_WEBUI_HOST", "0.0.0.0")', source
        )

    def test_driver_config_propagates_webui_host(self):
        source = (ROOT / "src" / "mapping_rbnx" / "atlas_bridge.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('cfg.get(\n        "webui_host"', source)
        self.assertIn('os.environ["MAPPING_WEBUI_HOST"] = _webui_host', source)
        self.assertIn('os.environ.pop("MAPPING_WEBUI_PORT", None)', source)

    def test_config_spec_declares_loopback_safe_webui_host(self):
        source = (ROOT / "config.spec").read_text(encoding="utf-8")
        self.assertIn("  webui_host:\n", source)
        self.assertIn("    default: 127.0.0.1\n", source)


class WebUiLiveStateTest(unittest.TestCase):
    def setUp(self):
        webui._clear_latest()

    def tearDown(self):
        webui._clear_latest()

    def test_receipt_ages_drive_independent_map_and_pose_staleness(self):
        grid = object()
        pose = object()
        webui._record_latest("grid", grid, received_monotonic=100.0)
        webui._record_latest("pose", pose, received_monotonic=100.0)

        fresh = webui._latest_snapshot(now_monotonic=101.0)
        self.assertIs(fresh["grid"], grid)
        self.assertIs(fresh["pose"], pose)
        self.assertEqual(fresh["map_age_s"], 1.0)
        self.assertEqual(fresh["pose_age_s"], 1.0)
        self.assertFalse(fresh["map_stale"])
        self.assertFalse(fresh["pose_stale"])

        stale = webui._latest_snapshot(now_monotonic=106.0)
        self.assertEqual(stale["map_age_s"], 6.0)
        self.assertEqual(stale["pose_age_s"], 6.0)
        self.assertTrue(stale["map_stale"])
        self.assertTrue(stale["pose_stale"])

        webui._record_latest("pose", pose, received_monotonic=105.0)
        cached_map_live_pose = webui._latest_snapshot(now_monotonic=106.0)
        self.assertTrue(cached_map_live_pose["map_stale"])
        self.assertFalse(cached_map_live_pose["pose_stale"])

    def test_successful_reset_clears_grid_pose_and_receipt_times(self):
        webui._record_latest("grid", object(), received_monotonic=10.0)
        webui._record_latest("pose", object(), received_monotonic=10.0)
        with mock.patch.object(
            webui.map_ops, "reset_map_impl", return_value={"ok": True, "detail": "reset"}
        ):
            out = webui._reset_map_from_webui()

        self.assertTrue(out["ok"])
        live = webui._latest_snapshot(now_monotonic=11.0)
        self.assertIsNone(live["grid"])
        self.assertIsNone(live["pose"])
        self.assertIsNone(live["map_age_s"])
        self.assertIsNone(live["pose_age_s"])
        self.assertTrue(live["map_stale"])
        self.assertTrue(live["pose_stale"])

    def test_failed_reset_preserves_last_live_samples(self):
        grid = object()
        pose = object()
        webui._record_latest("grid", grid, received_monotonic=10.0)
        webui._record_latest("pose", pose, received_monotonic=10.0)
        with mock.patch.object(
            webui.map_ops, "reset_map_impl", return_value={"ok": False, "detail": "failed"}
        ):
            out = webui._reset_map_from_webui()

        self.assertFalse(out["ok"])
        live = webui._latest_snapshot(now_monotonic=11.0)
        self.assertIs(live["grid"], grid)
        self.assertIs(live["pose"], pose)

    def test_state_and_map_responses_are_no_store(self):
        grid = SimpleNamespace(
            info=SimpleNamespace(
                width=1,
                height=1,
                resolution=0.05,
                origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
            ),
            data=[0],
        )
        webui._record_latest("grid", grid)
        server = webui.ThreadingHTTPServer(("127.0.0.1", 0), webui._Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with mock.patch.object(webui, "_ensure_subscriptions", return_value=None):
                with request.urlopen(base + "/api/state", timeout=2.0) as response:
                    state = json.loads(response.read())
                    self.assertIn("no-store", response.headers["Cache-Control"])
                    self.assertIsNotNone(state["map_age_s"])
                    self.assertFalse(state["map_stale"])
                    self.assertTrue(state["pose_stale"])

                with request.urlopen(base + "/api/map.png", timeout=2.0) as response:
                    self.assertEqual(response.headers["Content-Type"], "image/png")
                    self.assertIn("no-store", response.headers["Cache-Control"])
                    self.assertTrue(response.read().startswith(b"\x89PNG"))

                # OccupancyGrid is commonly transient-local and may not be
                # republished while stationary. Its age is metadata, not a
                # reason to reject an otherwise valid persistent base map.
                webui._record_latest("grid", grid, received_monotonic=0.0)
                with request.urlopen(base + "/api/map.png", timeout=2.0) as response:
                    self.assertEqual(response.status, 200)
                    self.assertIn("no-store", response.headers["Cache-Control"])

                webui._clear_latest()
                with self.assertRaises(error.HTTPError) as caught:
                    request.urlopen(base + "/api/map.png", timeout=2.0)
                self.assertEqual(caught.exception.code, 503)
                self.assertIn("no-store", caught.exception.headers["Cache-Control"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    def test_frontend_uses_pose_freshness_and_retains_cached_base_map(self):
        self.assertIn("STALE —", webui._PAGE)
        self.assertIn("DISCONNECTED —", webui._PAGE)
        self.assertIn("base map retained, old robot pose hidden", webui._PAGE)
        self.assertIn("(cached base map)", webui._PAGE)
        self.assertIn("if(mapImg&&MI.resolution>0)", webui._PAGE)
        self.assertIn("if(MI.pose&&!MI.pose_stale)", webui._PAGE)
        self.assertNotIn("MI.map_stale||!MI.has_map", webui._PAGE)
        self.assertIn("fetch('/api/state',{cache:'no-store'})", webui._PAGE)


if __name__ == "__main__":
    unittest.main()
