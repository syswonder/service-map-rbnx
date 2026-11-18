import math
import unittest
from mapping_rbnx.slam_planarity import PlanarScanGuard


class PlanarObservationTests(unittest.TestCase):
    def test_yaw_motion_and_small_floor_vibration_keep_the_same_plane(self):
        guard = PlanarScanGuard()
        for yaw in (0., .7, 3.1, -2.):
            self.assertIsNone(guard.check(.735, (0., 0., math.sin(yaw/2), math.cos(yaw/2))))
        self.assertIsNone(guard.check(.75, (0., 0., 0., 1.)))

    def test_climbing_furniture_cannot_seed_or_pollute_the_plane(self):
        guard = PlanarScanGuard()
        pitch = math.radians(12)
        self.assertIn("tilted", guard.check(.85, (0., math.sin(pitch/2), 0., math.cos(pitch/2))))
        self.assertIsNone(guard.reference_height)
        self.assertIsNone(guard.check(.735, (0., 0., 0., 1.)))
        self.assertIn("height", guard.check(.86, (0., 0., 0., 1.)))
        self.assertIsNone(guard.check(.735, (0., 0., 0., 1.)))

    def test_upside_down_mounting_and_quaternion_sign_are_equivalent(self):
        guard = PlanarScanGuard()
        self.assertIsNone(guard.check(.4, (1., 0., 0., 0.)))
        self.assertIsNone(guard.check(.4, (-1., 0., 0., 0.)))

    def test_invalid_transforms_do_not_change_the_reference(self):
        guard = PlanarScanGuard()
        self.assertIsNotNone(guard.check(float('nan'), (0., 0., 0., 1.)))
        self.assertIsNotNone(guard.check(.4, (0., 0., 0., 0.)))
        self.assertIsNone(guard.reference_height)
