"""Retain remote hypotheses on a partial graph without changing other paths."""
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from mapping_rbnx import global_match,localizers

class IncrementalSearchTests(unittest.TestCase):
    def test_real_partial_room_hypothesis_survives_the_wider_search(self):
        with np.load(Path(__file__).parent/'testdata/house187_partial_reentry.npz') as v:
            height,width=map(int,v['shape'])
            grid=localizers._Grid(width,height,bytearray(v['occupied']),bytearray(v['known']),float(v['resolution']),tuple(v['origin']))
            scan=SimpleNamespace(**{k:float(v[k]) for k in ['angle_min','angle_increment','range_min','range_max']},ranges=v['ranges'])
            truth=v['truth'].copy()
        match=global_match.global_scan_match(grid,scan,sensor_xy=(.18,0.),fine_candidate_budget=256)
        self.assertIsNotNone(match)
        supported=[c for c in match.candidates if math.dist(c.pose[:2],truth[:2])<.15 and abs(math.atan2(math.sin(c.pose[2]-truth[2]),math.cos(c.pose[2]-truth[2])))<math.radians(3)]
        self.assertTrue(supported,'a correct remote pose must remain available for confirmation')
        # This is an ambiguous scan. Retaining its hypothesis must not force it
        # to win or weaken the ordinary auto-acceptance threshold.
        self.assertLess(supported[0].score,localizers.STATIC_MATCH_MIN)

    def test_only_slam_incremental_backend_requests_more_hypotheses(self):
        from mapping_rbnx import map_ops
        for engine,mode,budget in [('slam_toolbox','mapping',256),('slam_toolbox','localization',None),('rtabmap','mapping',None),('rtabmap','localization',None)]:
            backend=map_ops._LocalizationBackend(None,None,'/unused','fixture',target_mode=mode)
            with self.subTest(engine=engine,mode=mode),patch.object(map_ops,'active_algo',return_value=engine),patch.object(localizers,'match_statically',return_value=(None,'no scan')) as match:
                backend.match(None)
                self.assertEqual(match.call_args.kwargs,{} if budget is None else {'fine_candidate_budget':budget})
