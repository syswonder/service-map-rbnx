import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent

try:
    from nav_msgs.msg import OccupancyGrid
except ImportError:  # outside a ROS environment
    OccupancyGrid = None


def load_relay():
    spec = importlib.util.spec_from_file_location(
        "map_cover_robot", ROOT / "scripts" / "map_cover_robot.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def grid(ox, oy, w, h, res=0.5):
    g = OccupancyGrid()
    g.info.resolution = res
    g.info.width, g.info.height = w, h
    g.info.origin.position.x, g.info.origin.position.y = ox, oy
    g.data = list(range(w * h))
    return g


@unittest.skipIf(OccupancyGrid is None, "needs ROS 2 message types")
class CoverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.relay = load_relay()

    def test_a_box_inside_the_grid_leaves_it_alone(self):
        g = grid(0.0, 0.0, 4, 4)
        self.assertIs(self.relay.cover(g, (0.5, 0.5, 1.5, 1.5)), g)

    def test_a_box_behind_the_grid_is_added_as_unknown(self):
        g = grid(0.0, 0.0, 2, 2)
        out = self.relay.cover(g, (-1.0, 0.0, 1.0, 1.0))
        self.assertEqual((out.info.width, out.info.height), (4, 2))
        self.assertEqual(out.info.origin.position.x, -1.0)
        self.assertEqual(list(out.data), [-1, -1, 0, 1, -1, -1, 2, 3])
        self.assertEqual(g.info.width, 2)  # the input is not modified


if __name__ == "__main__":
    unittest.main()
