# SPDX-License-Identifier: MulanPSL-2.0
"""The web UI's HTTP surface, exercised against stub impls.

The page is not a second implementation of the map operations -- it calls the
same map_ops functions the gRPC servicers and the MCP handlers call. The bug
this guards against is the page diverging from them: carrying its own copy of
the session state, or passing arguments the other entry points do not. Each
test asserts the handler forwards to map_ops and reports back what map_ops (or
lifecycle) says, rather than anything the page remembered.
"""
from __future__ import annotations

import json
import re
import threading
import unittest
from types import SimpleNamespace
import urllib.request
from http.server import ThreadingHTTPServer
from unittest.mock import patch

from mapping_rbnx import webui


class WebUiShutdownTest(unittest.TestCase):
    def test_shutdown_joins_frozen_diagnostic_and_bounds_incomplete_http(self):
        import socket
        import time
        with patch.dict(webui.os.environ, {"MAPPING_WEBUI_PORT": "0"}), \
                patch.object(webui, "_ensure_subscriptions"), \
                patch.object(webui.reference_time, "reading", return_value=(0, 1, 1)):
            webui.start_supervisor()
            webui.maybe_start()
            port = webui._server.server_address[1]
            serving, supervising = webui._server_thread, webui._supervisor
            peers = [socket.create_connection(("127.0.0.1", port), timeout=2) for _ in range(2)]
            sending = threading.Event()

            def trickle():
                while not sending.wait(.1):
                    try:
                        peers[1].sendall(b"x")
                    except OSError:
                        return

            sender = threading.Thread(target=trickle)
            try:
                for peer in peers:
                    peer.sendall(b"GET / HTTP/1.1\r\nX-Test: ")
                sender.start()
                deadline = time.monotonic() + 2
                while getattr(webui._server._threads, "__len__", lambda: 0)() < 2 and time.monotonic() < deadline:
                    time.sleep(.01)
                handlers = list(webui._server._threads)
                self.assertEqual(len(handlers), 2)
                webui._track_convergence(0, 0, 0, settle_s=.01)
                diagnostic = webui._convergence_worker
                self.assertTrue(diagnostic.is_alive())
                start = time.monotonic()
                webui.stop()
                self.assertLess(time.monotonic() - start, 3)
                self.assertFalse(any(t.is_alive() for t in (serving, supervising, diagnostic, *handlers)))
                with socket.socket() as probe:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    probe.bind(("127.0.0.1", port))
            finally:
                sending.set()
                if sender.ident is not None:
                    sender.join(1)
                for peer in peers:
                    peer.close()
                webui.stop()
                webui._stop_evt.clear()


class WebUiApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), webui._Handler)
        cls.base = "http://127.0.0.1:%d" % cls.srv.server_address[1]
        cls.thread = threading.Thread(target=cls.srv.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def call(self, path, body=None):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(
            self.base + path, data=data,
            headers={"Content-Type": "application/json"},
            method="POST" if data is not None else "GET")
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read() or b"{}")

    def test_save_passes_no_session_state_of_its_own(self):
        # A page that supplied its own live-database hint here is exactly how
        # the two entry points drifted apart. It must call the impl the same
        # way the servicers do: id and note, nothing else.
        with patch.object(webui.map_ops, "save_map_impl",
                          return_value={"ok": True, "detail": "saved"}) as save:
            out = self.call("/api/save", {"map_id": "lab", "note": "n"})
        self.assertTrue(out["ok"])
        save.assert_called_once_with("lab", "n")

    def test_pose_candidates_forwards_to_shared_impl(self):
        expected = {"ok": True, "detail": "ambiguous", "map_id": "lab",
                    "generation": 3, "needs_selection": True,
                    "candidates_json": "[]"}
        with patch.object(webui.map_ops, "get_pose_candidates_impl",
                          return_value=expected) as query:
            out = self.call("/api/pose_candidates")
        self.assertEqual(out, expected)
        query.assert_called_once_with()

    def test_load_forwards_the_pose_seed(self):
        with patch.object(webui.map_ops, "load_map_impl",
                          return_value={"ok": True, "detail": "loaded"}) as load:
            self.call("/api/load", {"map_id": "lab", "mode": "localization",
                                    "has_initial_pose": True,
                                    "x": 1.5, "y": -2.0, "theta": 0.25})
        load.assert_called_once_with("lab", "localization", True, 1.5, -2.0, 0.25)

    def test_state_reports_the_service_mode_not_a_local_copy(self):
        with (
            patch.object(webui.map_ops, "get_mode_impl",
                         return_value={"ok": True, "mode": "localization", "detail": ""}),
            patch.object(webui.lifecycle, "current",
                         return_value={"map_id": "lab_3f", "mode": "localization"}),
        ):
            out = self.call("/api/state")
        self.assertEqual(out["mode"], "localization")
        self.assertEqual(out["map_id"], "lab_3f")

    def test_state_mode_follows_a_change_the_page_never_made(self):
        # A load or a reset issued over MCP changes the mode with the page
        # uninvolved; the badge has to follow it.
        for mode in ("mapping", "localization"):
            with (
                patch.object(webui.map_ops, "get_mode_impl",
                             return_value={"ok": True, "mode": mode, "detail": ""}),
                patch.object(webui.lifecycle, "current",
                             return_value={"map_id": "", "mode": mode}),
            ):
                self.assertEqual(self.call("/api/state")["mode"], mode)

    def test_switch_and_reset_reach_the_impls(self):
        with patch.object(webui.map_ops, "switch_mode_impl",
                          return_value={"ok": True, "detail": "switched"}) as sw:
            self.call("/api/switch_mode", {"mode": "mapping"})
        sw.assert_called_once_with("mapping")
        with patch.object(webui.map_ops, "reset_map_impl",
                          return_value={"ok": True, "detail": "cleared"}) as rs:
            self.call("/api/reset", {})
        rs.assert_called_once_with()

    def test_a_failing_impl_is_reported_verbatim(self):
        # The page must not soften or invent a result: the operator needs the
        # service's own words to act on.
        with patch.object(webui.map_ops, "save_map_impl",
                          return_value={"ok": False, "detail": "no live rtabmap database"}):
            out = self.call("/api/save", {"map_id": "lab"})
        self.assertFalse(out["ok"])
        self.assertEqual(out["detail"], "no live rtabmap database")

    def test_every_action_is_recorded_in_the_activity_log(self):
        with (
            patch.object(webui.map_ops, "save_map_impl", return_value={"ok": True, "detail": "saved lab"}),
            patch.object(webui.map_ops, "reset_map_impl", return_value={"ok": True, "detail": "cleared"}),
        ):
            self.call("/api/save", {"map_id": "lab"})
            self.call("/api/reset", {})
        kinds = {e["kind"] for e in self.call("/api/log")}
        self.assertTrue({"save", "reset"} <= kinds, kinds)


class StaticAssetTest(unittest.TestCase):
    """The page is three files on disk, not a Python string.

    It used to be one string holding minified HTML, CSS and JavaScript. That
    made every change a merge hazard and silently ate escapes: a "\\n" written
    for a JS string literal became a real newline, split the literal across
    lines and stopped the entire script -- with nothing in any log, because no
    server ever parses the page it serves.
    """

    def test_the_assets_exist_and_carry_the_features_they_should(self):
        js = (webui.STATIC_DIR / "app.js").read_text()
        html = (webui.STATIC_DIR / "index.html").read_text()
        self.assertIn("askConfirm", js)
        self.assertIn("session id", js)          # the mode-switch explanation
        self.assertIn("MIN_POSE_DRAG_PX", js)    # the pose drag has a floor
        self.assertIn("runExclusive", js)        # map ops are single-flight
        self.assertIn('candidateTool.hidden = s.engine !== "slam_toolbox"', js)
        self.assertIn("tool-candidates", html)
        self.assertIn("posehint", html)
        self.assertIn("/static/app.js", html)
        # Styles are served from disk, never fetched: the robot has no internet.
        self.assertIn("/static/style.css", html)
        self.assertNotIn("http://", html)
        self.assertNotIn("https://", html.replace("https://getbootstrap.com", ""))

    def test_assets_are_served_with_their_own_content_types(self):
        for name, kind in (("index.html", "text/html"),
                           ("app.js", "application/javascript"),
                           ("style.css", "text/css"),
                           ("vendor/bootstrap.min.css", "text/css")):
            status, ctype, body = webui._static(name)
            self.assertEqual(status, 200, name)
            self.assertIn(kind, ctype)
            self.assertTrue(body)

    def test_only_the_pages_own_assets_are_readable(self):
        # A fixed set of names, so anything else is a mistake or an attack.
        for name in ("../webui.py", "/etc/passwd", "app.js.bak", ""):
            status, _, _ = webui._static(name)
            self.assertEqual(status, 404, name)


class RangeEndpointTest(unittest.TestCase):
    """A capability the deployment has not bound is absent from the payload,
    not present and empty: the page then has nothing to say about a sensor that
    does not exist, instead of reporting it as perpetually waiting."""

    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), webui._Handler)
        cls.base = "http://127.0.0.1:%d" % cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def get(self):
        with urllib.request.urlopen(self.base + "/api/range", timeout=10) as r:
            return json.loads(r.read())

    def test_scan_only_deployment_reports_only_scan(self):
        self.addCleanup(webui.set_sensor_topics, "", "")
        webui.set_sensor_topics(scan="/scanner_normalized")
        with patch.object(webui, "_overlay_in_map",
                          return_value={"pts": [[1.0, 2.0]], "frame": "l", "stale": False}):
            out = self.get()
        self.assertIn("scan", out)
        self.assertNotIn("cloud", out)
        self.assertEqual(out["bound"], {"scan": "/scanner_normalized"})

    def test_a_deployment_with_no_lidar_reports_nothing_to_draw(self):
        self.addCleanup(webui.set_sensor_topics, "", "")
        webui.set_sensor_topics()
        out = self.get()
        self.assertEqual(out, {"bound": {}})


class ReferenceTrustTest(unittest.TestCase):
    def setUp(self):
        from mapping_rbnx import reference_time, webui
        self.webui = webui
        self.sample = [(0, 1, 1)]
        self.patches = [
            patch.object(reference_time, "_clock", SimpleNamespace(read=lambda: self.sample[0])),
            patch.object(webui.map_ops, "active_algo", return_value="slam_toolbox"),
            patch.object(webui.map_ops._LOCALIZATION, "snapshot", return_value={"run_id": 0, "map_id": "", "state": "idle"}),
            patch.object(webui, "_reference", None),
            patch.object(webui, "_latest", dict.fromkeys(webui._latest)),
            patch.object(webui, "_trust", dict(grid_for="m", grid=object(), fit=None,
                         at=None, detail="", history=[], bad_since=None,
                         withdrawn=False, evidence=None)),
            patch.object(webui.lifecycle, "current", return_value={"mode": "localization", "map_id": "m"}),
            patch.object(webui, "_live_pose_xytheta", return_value=(0., 0., 0.)),
            patch.object(webui.localizers, "scan_fit_of", return_value=(.9, "fits")),
        ]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        webui._receive("pose", object())
        webui._receive("scan", {"pts": [], "frame": "laser"})
        webui._latest["scan_msg"] = object()

    def test_zero_receipts_stay_fresh_during_pause_without_recounting(self):
        import time
        first = self.webui._localization_trust()
        time.sleep(.03)
        second = self.webui._localization_trust()
        self.assertEqual(first["verdict"], "ok")
        self.assertEqual(second["samples"], 1)
        self.assertEqual(self.webui._stale_inputs(0.), "")
        self.sample[0] = (2_000_000_000, 1, 1)
        self.assertEqual(self.webui._localization_trust()["samples"], 1)

    def test_generation_reset_discards_pose_scan_and_trust_history(self):
        self.webui._localization_trust()
        self.sample[0] = (0, 2, 1)
        self.webui._reference_now()
        self.assertIsNone(self.webui._latest["pose"])
        self.assertIsNone(self.webui._latest["scan_msg"])
        self.assertEqual(self.webui._trust["history"], [])

    def test_bad_since_zero_does_not_trigger_recovery_while_paused(self):
        with patch.object(self.webui.map_ops, "relocalize_impl") as recover:
            self.webui._enforce_trust(False, "m", .1)
            self.webui._enforce_trust(False, "m", .1)
            self.assertEqual(self.webui._trust["bad_since"], 0.)
            recover.assert_not_called()


    def test_projection_from_previous_generation_is_discarded(self):
        baseline = self.sample[0]
        self.sample[0] = (0, 2, 1)
        self.assertFalse(self.webui._receive("scan", {"pts": [(1., 0.)]}, baseline))
        self.assertIsNone(self.webui._latest["scan"])
        self.assertIsNone(self.webui._latest["scan_msg"])
