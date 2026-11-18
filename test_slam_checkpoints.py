"""Checkpoint transactions, coverage invariants and concurrent worker behavior."""
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
from mapping_rbnx.slam_checkpoints import CheckpointStore, Coverage, MappingSession, GRAPH_FILES,validate_reference,MappingDiverged
from mapping_rbnx.slam_motion import advance
from mapping_rbnx.slam_guard import PoseJumpGuard,OdomJumpGuard


def grid(data, width, x=0., y=0., resolution=1., frame="map"):
    return NS(data=data, header=NS(frame_id=frame), info=NS(width=width,height=len(data)//width,
        resolution=resolution,origin=NS(position=NS(x=x,y=y),orientation=NS(x=0.,y=0.,z=0.,w=1.))))


def fake_capture(directory):
    for name in GRAPH_FILES:
        (directory/name).write_bytes(b"complete graph file")


def fake_inspect(directory, resolution):
    directory=Path(directory)
    for name in ("occupancy.pgm","occupancy.yaml","occupancy.png","graph.json","nodes.jsonl"):
        (directory/name).write_bytes(b"verified artifact")
    return {"nodes": 4,"edges":3,"components":1}


class CoverageTests(unittest.TestCase):
    def test_unknown_canvas_expansion_and_origin_shift_do_not_count_twice(self):
        c=Coverage()
        self.assertEqual(c.observe(grid([0,100],2)),2.)
        self.assertEqual(c.observe(grid([-1,0,100,-1],4,x=-1.)),2.)
        self.assertEqual(c.observe(grid([-1,0,100,0],4,x=-1.)),3.)

    def test_only_known_cells_count_and_frame_changes_are_rejected(self):
        c=Coverage()
        self.assertEqual(c.observe(grid([-1,0,-1,100],2,resolution=.5)),.5)
        with self.assertRaises(ValueError):c.observe(grid([0],1,frame="other"))
        with self.assertRaises(ValueError):c.observe(grid([0],1,resolution=.25))

    def test_pose_transport_handles_translation_and_rotation(self):
        import math
        result=advance((10.,5.,math.pi/2),(1.,0.,0.),(2.,0.,math.pi/2))
        self.assertAlmostEqual(result[0],10.)
        self.assertAlmostEqual(result[1],6.)
        self.assertAlmostEqual(abs(result[2]),math.pi)

    def test_guard_allows_odometry_motion_and_detects_map_frame_jump(self):
        guard=PoseJumpGuard()
        self.assertIsNone(guard.update(1,(10.,0.,0.),(0.,0.,0.)))
        self.assertIsNone(guard.update(2,(12.,0.,0.),(2.,0.,0.)))
        self.assertIn("review",guard.update(3,(14.,0.,0.),(2.1,0.,0.)))

    def test_guard_ignores_duplicate_stamps(self):
        guard=PoseJumpGuard()
        self.assertIsNone(guard.update(1,(0.,0.,0.),(0.,0.,0.)))
        self.assertIsNone(guard.update(1,(10.,0.,0.),(0.,0.,0.)))
        self.assertIsNone(guard.update(2,(.1,0.,0.),(.1,0.,0.)))

    def test_raw_odometry_reset_is_detected_even_if_scan_matcher_follows_it(self):
        guard=OdomJumpGuard()
        self.assertIsNone(guard.update(1.,(0.,0.,0.)))
        self.assertIsNone(guard.update(1.1,(.03,0.,0.)))
        self.assertIn("odometry reset",guard.update(1.2,(.03,0.,1.)))


class TransactionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.store=CheckpointStore(self.tmp.name)
        self.session="a"*32
        self.inspector=patch("mapping_rbnx.slam_checkpoints.inspect_graph",side_effect=fake_inspect)
        self.inspector.start();self.addCleanup(self.inspector.stop)

    def save(self,capture=fake_capture):
        return self.store.create(self.session,capture,metadata={},reason="manual")

    def test_partial_write_is_not_discoverable_or_latest(self):
        def interrupted(directory):
            (directory/GRAPH_FILES[0]).write_bytes(b"half")
            raise OSError("disk full")
        with self.assertRaises(OSError):self.save(interrupted)
        self.assertEqual(self.store.list(),[])
        self.assertFalse((Path(self.tmp.name)/self.session/"latest.json").exists())
        self.assertEqual(list((Path(self.tmp.name)/self.session).iterdir()),[])

    def test_capture_resolution_overrides_default_before_cold_render(self):
        def capture(directory):
            fake_capture(directory)
            return {"engine_parameters":{"resolution":.025}}
        with patch("mapping_rbnx.slam_checkpoints.inspect_graph",side_effect=fake_inspect) as inspector:
            saved=self.save(capture)
            self.assertEqual(inspector.call_args[0][1],.025)
            self.assertEqual(saved['metadata']['engine_parameters']['resolution'],.025)

    def test_fault_arriving_during_capture_cannot_promote_checkpoint(self):
        first=self.save()
        def fault():
            raise MappingDiverged('odometry fault arrived during rendering')
        with self.assertRaises(MappingDiverged):
            self.store.create(self.session,fake_capture,metadata={},reason='manual',before_commit=fault)
        self.assertEqual(self.store.list()[0]['checkpoint_id'],first['checkpoint_id'])
        self.assertEqual(json.loads((Path(self.tmp.name)/self.session/'latest.json').read_text())['checkpoint_id'],first['checkpoint_id'])
        self.assertEqual(len(self.store.list()),1)

    def test_corruption_is_detected_before_restore(self):
        saved=self.save();path,_=self.store.validate(self.session,saved["checkpoint_id"])
        (path/GRAPH_FILES[1]).write_bytes(b"corrupted")
        with self.assertRaisesRegex(ValueError,"integrity"):
            self.store.validate(self.session,saved["checkpoint_id"])

    def test_incomplete_manifest_and_path_traversal_are_rejected(self):
        saved=self.save();path=self.store.path(self.session,saved["checkpoint_id"])
        manifest=json.loads((path/"manifest.json").read_text());del manifest["files"][GRAPH_FILES[0]]
        (path/"manifest.json").write_text(json.dumps(manifest))
        with self.assertRaises(ValueError):self.store.validate(self.session,saved["checkpoint_id"])
        with self.assertRaises(ValueError):self.store.path("../","b"*32)

    def test_prior_checkpoint_survives_next_failed_capture(self):
        saved=self.save()
        with self.assertRaises(RuntimeError):
            self.save(lambda directory: (_ for _ in ()).throw(RuntimeError("engine lost")))
        self.assertEqual(self.store.list()[0]["checkpoint_id"],saved["checkpoint_id"])
        self.store.validate(self.session,saved["checkpoint_id"])

    def test_empty_individual_graph_file_is_rejected(self):
        from mapping_rbnx.engines import SlamToolboxOps
        directory=Path(self.tmp.name)
        (directory/GRAPH_FILES[0]).write_bytes(b"graph")
        (directory/GRAPH_FILES[1]).touch()
        self.assertFalse(SlamToolboxOps().graph_ready(str(directory))[0])

    def test_deformed_or_missing_baseline_nodes_cannot_be_accepted(self):
        path=Path(self.tmp.name)
        (path/'nodes.jsonl').write_text(json.dumps({'id':0,'pose':[1.,0.,0.]})+'\n')
        with self.assertRaises(MappingDiverged):
            validate_reference(path,{'reference_nodes':{'0':[0.,0.,0.]}})
        with self.assertRaises(MappingDiverged):
            validate_reference(path,{'reference_nodes':{'1':[1.,0.,0.]}})
        self.assertEqual(validate_reference(path,{'reference_nodes':{'0':[1.,0.,0.]}})['max_baseline_shift_m'],0.)


class ProcessLifecycleTests(unittest.TestCase):
    def test_slam_service_clients_are_released_on_success_and_timeout(self):
        from concurrent.futures import Future
        from unittest.mock import Mock
        from mapping_rbnx.engines import _call_service
        for complete in (True,False):
            node=Mock();client=node.create_client.return_value
            client.wait_for_service.return_value=True
            future=Future();client.call_async.return_value=future
            if complete:future.set_result('saved')
            ok,response=_call_service(node,object,'/test',object(),.01)
            self.assertEqual(ok,complete)
            node.destroy_client.assert_called_once_with(client)
            if not complete:self.assertTrue(future.cancelled())

    def test_stopping_an_engine_reaps_active_group_and_accepts_zombies(self):
        import subprocess,sys
        from mapping_rbnx.engines import _stop_process,_group_members
        process=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"],start_new_session=True)
        try:
            self.assertTrue(_stop_process(process.pid,timeout_s=.2))
            process.wait(timeout=2)
            self.assertEqual(_group_members(process.pid),[])
        finally:
            if process.poll() is None:process.kill();process.wait()


class WorkerTests(unittest.TestCase):
    def wait(self, condition):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            if condition():return
            time.sleep(.01)
        self.fail("checkpoint worker did not complete")

    def test_area_trigger_returns_control_while_capture_runs_and_bounds_history(self):
        with tempfile.TemporaryDirectory() as tmp, patch("mapping_rbnx.slam_checkpoints.inspect_graph",side_effect=fake_inspect):
            entered,release=threading.Event(),threading.Event()
            def slow(directory):
                entered.set();release.wait(2);fake_capture(directory)
            manager=MappingSession(tmp,slow,threading.RLock(),area_m2=2,min_interval_s=0,keep=1)
            try:
                manager.begin(baseline_grid=grid([0],1))
                manager.observe(grid([0,0,0],3))
                self.assertTrue(entered.wait(1))
                before=time.monotonic()
                manager.observe(grid([0,0,0,0],4))
                self.assertLess(time.monotonic()-before,.05)
                self.assertFalse(manager.request()["ok"])
                release.set()
                self.wait(lambda:manager.status()["last_checkpoint"] is not None)
                old=manager.status()["last_checkpoint"]["checkpoint_id"]
                self.assertTrue(manager.request()["ok"])
                self.wait(lambda: manager.status()["last_checkpoint"]["checkpoint_id"]!=old)
                self.wait(lambda:len(manager.store.list())==1)
            finally:
                release.set();manager.close()

    def test_failed_snapshot_does_not_advance_coverage_watermark(self):
        with tempfile.TemporaryDirectory() as tmp:
            def fail(directory):raise OSError("write failed")
            manager=MappingSession(tmp,fail,threading.RLock(),area_m2=1,min_interval_s=60)
            try:
                manager.begin();manager.observe(grid([0,0],2));manager.request()
                self.wait(lambda:bool(manager.status()["error"]))
                self.assertIsNone(manager.status()["last_checkpoint"])
                self.assertEqual(manager.status()["new_area_m2"],2.)
                self.assertEqual(manager.status()["state"],"mapping")
            finally:manager.close()

    def test_suspicious_mapping_is_withdrawn_and_cannot_checkpoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            withdrew=threading.Event()
            manager=MappingSession(tmp,fake_capture,threading.RLock(),guard=lambda:"mapping jump",on_suspect=lambda why:withdrew.set())
            try:
                manager.begin();manager.observe(grid([0],1))
                self.assertTrue(withdrew.wait(2))
                self.assertEqual(manager.status()["state"],"suspect")
                self.assertFalse(manager.request()["ok"])
                self.assertEqual(manager.store.list(),[])
            finally:manager.close()


if __name__=="__main__":unittest.main()
