# SPDX-License-Identifier: MulanPSL-2.0
"""Observable localization transitions and competing completion paths."""
import threading
import time
import unittest
from types import SimpleNamespace
from mapping_rbnx.localization_flow import LocalizationFlow
from mapping_rbnx.map_ops import _LocalizationBackend


def match(score=.9, runner=.5):
    return SimpleNamespace(pose=(1., 2., .3), score=score, runner_up=runner,
                           detail='scan match', candidates=[SimpleNamespace(
                               pose=(float(i), 2., .3), score=score-i*.01) for i in range(5)])


class Backend:
    conclusive = staticmethod(_LocalizationBackend.conclusive)
    def __init__(self, result=None):
        self.result = result or match()
        self.searches = 0
        self.filters = 0
        self.search_gate = None
        self.filter_entered = threading.Event()
        self.filter_result = None
        self.verified = []
        self.handed = []
        self.verify_ok = True
        self.cleaned = threading.Event()
    def prepare(self, run): pass
    def match(self, run):
        self.searches += 1
        if self.search_gate: self.search_gate.wait(2)
        return self.result, 'scan'
    def filter(self, run):
        self.filters += 1
        self.filter_entered.set()
        if self.filter_result: return self.filter_result, 'filter converged'
        run.signal.wait(2)
        return None, 'filter timeout'
    def stop_filter(self): pass
    def verify(self, run, pose):
        self.verified.append(pose)
        return self.verify_ok, 'laser verified' if self.verify_ok else 'laser disagrees'
    def handover(self, run, pose):
        self.handed.append(pose)
        return True, 'engine verified'
    def cleanup(self, run, completed): self.cleaned.set()


class LocalizationFlowTests(unittest.TestCase):
    def wait(self, flow, phase):
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            state = flow.snapshot()
            if state['phase'] == phase: return state
            time.sleep(.005)
        self.fail(f'expected {phase}, got {flow.snapshot()}')

    def test_clear_top1_searches_once_and_skips_filter(self):
        flow, backend = LocalizationFlow(), Backend()
        flow.start(backend, 'lab', 1)
        state = self.wait(flow, 'ready')
        self.assertEqual((backend.searches, backend.filters), (1, 0))
        self.assertEqual(backend.verified, backend.handed)
        self.assertEqual(state['candidates'], [])

    def test_both_thresholds_are_required(self):
        for result in [match(.70,.3), match(.9,.88)]:
            self.assertFalse(Backend.conclusive(result))
        self.assertTrue(Backend.conclusive(match(.9,.6)))

    def test_ambiguous_match_offers_five_and_starts_filter(self):
        flow, backend = LocalizationFlow(), Backend(match(.9,.88))
        run_id = flow.start(backend, 'lab', 1)
        self.assertTrue(backend.filter_entered.wait(1))
        state = self.wait(flow, 'choosing')
        self.assertEqual(len(state['candidates']), 5)
        # Polling reads one immutable result; it cannot run another search.
        for _ in range(10): flow.snapshot()
        self.assertEqual(backend.searches, 1)
        self.assertTrue(flow.submit((4,5,0), run_id)[0])
        self.wait(flow, 'ready')
        self.assertEqual(backend.handed, [(4,5,0)])
        self.assertEqual(flow.snapshot()['candidates'], [])

    def test_filter_can_finish_without_user_selection(self):
        flow, backend = LocalizationFlow(), Backend(match(.9,.88))
        backend.filter_result = (6,7,0)
        flow.start(backend, 'lab', 1)
        self.wait(flow, 'ready')
        self.assertEqual(backend.handed, [(6,7,0)])
        self.assertEqual(flow.snapshot()['candidates'], [])

    def test_restart_discards_late_search_and_rejects_old_selection(self):
        flow = LocalizationFlow()
        old, new = Backend(match(.9,.88)), Backend()
        old.search_gate = threading.Event()
        old_id = flow.start(old, 'lab', 1)
        while not old.searches: time.sleep(.005)
        new_id = flow.start(new, 'lab', 2)
        self.assertFalse(flow.submit((9,9,0), old_id)[0])
        old.search_gate.set()
        state = self.wait(flow, 'ready')
        self.assertEqual(state['run_id'], new_id)
        self.assertEqual(old.handed, [])
        self.assertEqual(old.filters, 0)
        self.assertEqual(new.searches, 1)

    def test_restart_interrupts_filter_and_clears_candidates(self):
        flow, old, new = LocalizationFlow(), Backend(match(.9,.88)), Backend()
        flow.start(old, 'lab', 1)
        self.assertTrue(old.filter_entered.wait(1))
        flow.start(new, 'lab', 2)
        self.wait(flow, 'ready')
        self.assertEqual(old.handed, [])
        self.assertEqual(flow.snapshot()['candidates'], [])

    def test_seed_during_search_wins_over_automatic_result(self):
        flow, backend = LocalizationFlow(), Backend()
        backend.search_gate = threading.Event()
        run_id = flow.start(backend, 'lab', 1)
        while not backend.searches: time.sleep(.005)
        self.assertTrue(flow.submit((8,9,1), run_id)[0])
        backend.search_gate.set()
        self.wait(flow, 'ready')
        self.assertEqual(backend.handed, [(8,9,1)])

    def test_failed_laser_check_never_hands_over(self):
        flow, backend = LocalizationFlow(), Backend()
        backend.verify_ok = False
        flow.start(backend, 'lab', 1)
        self.wait(flow, 'failed')
        self.assertEqual(backend.handed, [])

    def test_filter_timeout_keeps_manual_selection_available(self):
        flow, backend = LocalizationFlow(), Backend(match(.9,.88))
        backend.filter = lambda run: (None, 'timeout')
        run_id = flow.start(backend, 'lab', 1)
        state = self.wait(flow, 'choosing')
        self.assertEqual(len(state['candidates']), 5)
        self.assertTrue(flow.submit((8,9,0), run_id)[0])
        self.wait(flow, 'ready')

    def test_supplied_pose_skips_global_search(self):
        flow, backend = LocalizationFlow(), Backend()
        flow.start(backend, 'lab', 1, seed=(4,5,0))
        self.wait(flow, 'ready')
        self.assertEqual(backend.searches, 0)
        self.assertEqual(backend.handed, [(4,5,0)])

    def test_user_choice_wins_when_filter_finishes_at_the_same_time(self):
        flow, backend = LocalizationFlow(), Backend(match(.9,.88))
        def finish_filter(run):
            self.assertTrue(flow.submit((8,9,1), run.run_id)[0])
            return (3,4,0), 'filter converged'
        backend.filter = finish_filter
        flow.start(backend, 'lab', 1)
        self.wait(flow, 'ready')
        self.assertEqual(backend.handed, [(8,9,1)])

    def test_verification_closes_selection_window(self):
        flow, backend = LocalizationFlow(), Backend()
        def verify(run, pose):
            self.assertFalse(flow.submit((8,9,1), run.run_id)[0])
            return True, 'verified'
        backend.verify = verify
        flow.start(backend, 'lab', 1)
        self.wait(flow, 'ready')
        self.assertEqual(backend.handed, [backend.result.pose])

    def test_reference_zero_keeps_verification_pending_until_new_scan(self):
        from unittest.mock import patch
        from mapping_rbnx import reference_time
        flow, backend = LocalizationFlow(), Backend()
        gate = threading.Event()
        entered = threading.Event()
        def verify(run, pose):
            entered.set()
            return reference_time.wait(gate, .01), 'fresh scan'
        backend.verify = verify
        with patch.object(reference_time, '_clock', SimpleNamespace(read=lambda: (0, 1, 1))):
            try:
                flow.start(backend, 'lab', 1)
                self.assertTrue(entered.wait(1))
                time.sleep(.04)
                self.assertEqual(flow.snapshot()['phase'], 'verifying')
                self.assertEqual(backend.handed, [])
                gate.set()
                self.wait(flow, 'ready')
            finally:
                flow.cancel()

    def test_reference_reset_while_choosing_discards_candidates_and_joins(self):
        from unittest.mock import patch
        from mapping_rbnx import reference_time
        flow, backend = LocalizationFlow(), Backend(match(.9,.88))
        backend.filter = lambda run: (None, 'filter timeout')
        sample = [(0, 1, 1)]
        with patch.object(reference_time, '_clock', SimpleNamespace(read=lambda: sample[0])):
            try:
                flow.start(backend, 'lab', 1)
                self.wait(flow, 'choosing')
                sample[0] = (0, 2, 1)
                state = self.wait(flow, 'failed')
                self.assertEqual(state['candidates'], [])
                self.assertEqual(backend.handed, [])
                self.assertFalse(flow.submit((1,2,0))[0])
            finally:
                flow.cancel()
            self.assertTrue(all(not worker.is_alive() for worker in flow._workers))

    def test_cancel_joins_manual_selection_wait_during_pause(self):
        from unittest.mock import patch
        from mapping_rbnx import reference_time
        flow, backend = LocalizationFlow(), Backend(match(.9,.88))
        backend.filter = lambda run: (None, 'filter timeout')
        with patch.object(reference_time, '_clock', SimpleNamespace(read=lambda: (0, 1, 1))):
            flow.start(backend, 'lab', 1)
            self.wait(flow, 'choosing')
            flow.cancel()
            self.assertEqual(backend.handed, [])
            self.assertTrue(all(not worker.is_alive() for worker in flow._workers))


class LocalizationPresentationTests(unittest.TestCase):
    def test_completed_flow_ignores_old_particle_covariance(self):
        from unittest.mock import patch
        from mapping_rbnx import webui
        with (
            patch.object(webui.map_ops, 'active_algo', return_value='slam_toolbox'),
            patch.object(webui.localizers, 'enabled', return_value=True),
            patch.object(webui.map_ops, 'get_mode_impl', return_value={'mode':'localization'}),
            patch.object(webui.lifecycle, 'current', return_value={'map_id':'lab','generation':1}),
            patch.object(webui.map_ops._LOCALIZATION, 'snapshot', return_value={
                'state':'done','phase':'ready','map_id':'lab','generation':1,'candidates':[]}),
            patch.dict(webui._latest, {'localizer_pose':object()}),
        ):
            self.assertEqual(webui._localization_state()['state'], 'converged')

    def test_supervisor_does_not_interrupt_an_initial_search(self):
        from unittest.mock import patch
        from mapping_rbnx import webui
        with (
            patch.object(webui.map_ops, 'active_algo', return_value='slam_toolbox'),
            patch.object(webui.map_ops._LOCALIZATION, 'snapshot', return_value={'map_id':'lab','state':'running'}),
            patch.object(webui.map_ops, 'relocalize_impl') as restart,
            patch.object(webui, '_trust', {'bad_since':1.,'withdrawn':False}),
        ):
            webui._enforce_trust(False, 'lab', 0.)
            webui._enforce_trust(False, 'lab', 0.)
            restart.assert_not_called()


if __name__ == '__main__': unittest.main()
