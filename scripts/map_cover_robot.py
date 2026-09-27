#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Republish slam_toolbox's grid on /map, grown to cover the robot.

A grid holds only cells some laser ray crossed. A lidar mounted ahead of the
base never crosses the cell the base stands on, so at the start of a session
the robot is outside the map, and nav2 refuses every goal ("the robot's start
position is off the global costmap") until the robot happens to turn round.
Cells added here are unknown, so nothing is claimed about them; a grid that
already covers the robot passes through unchanged.

The added area covers everywhere the robot has been this session, not just
where it is now. Shrinking it as the robot drove inwards made nav2 rebuild its
costmap every second, and a plan requested mid-rebuild failed with the same
"start position is off the global costmap".
"""
from __future__ import annotations

import math
from copy import deepcopy

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# Room around the base: its own footprint plus what nav2 inflates.
MARGIN_M = 0.5


def cover(grid: OccupancyGrid, box: tuple[float, float, float, float]) -> OccupancyGrid:
    """`grid` grown with unknown cells to contain `box` (x0, y0, x1, y1)."""
    info = grid.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    w, h = info.width, info.height
    x0, y0, x1, y1 = box
    left = max(0, math.ceil((ox - x0) / res))
    bottom = max(0, math.ceil((oy - y0) / res))
    right = max(0, math.ceil((x1 - (ox + w * res)) / res))
    top = max(0, math.ceil((y1 - (oy + h * res)) / res))
    if not (left or bottom or right or top):
        return grid
    nw, nh = w + left + right, h + bottom + top
    data = [-1] * (nw * nh)
    for row in range(h):
        start = (row + bottom) * nw + left
        data[start:start + w] = grid.data[row * w:(row + 1) * w]
    out = OccupancyGrid()
    out.header = grid.header
    out.info = deepcopy(info)
    out.info.width, out.info.height = nw, nh
    out.info.origin.position.x = ox - left * res
    out.info.origin.position.y = oy - bottom * res
    out.data = data
    return out


def _extent(grid: OccupancyGrid) -> tuple[float, float, float, float]:
    i = grid.info
    x, y = i.origin.position.x, i.origin.position.y
    return x, y, x + i.width * i.resolution, y + i.height * i.resolution


def _contains(a, b, tol: float = 1e-3) -> bool:
    return (a[0] <= b[0] + tol and a[1] <= b[1] + tol
            and a[2] >= b[2] - tol and a[3] >= b[3] - tol)


class MapCoverRobot(Node):
    def __init__(self) -> None:
        super().__init__("map_cover_robot")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self._grid: OccupancyGrid | None = None
        self._box = None  # (x0, y0, x1, y1): where the robot has been, plus MARGIN_M
        self._sent = None  # (width, height, origin x, origin y) last published
        self.create_subscription(OccupancyGrid, "/slam_toolbox/map", self._on_map, latched)
        # The engine publishes every few seconds, and TF may not exist yet when
        # a grid arrives; checking again keeps nav2 from waiting on the next one.
        self.create_timer(1.0, self._publish)

    def _on_map(self, grid: OccupancyGrid) -> None:
        old = self._grid
        if old is not None and not _contains(_extent(grid), _extent(old)):
            # The grid lost ground it had, so it is a new map (a load or a
            # reset) and where the robot went on the old one means nothing.
            self._box = None
        self._grid, self._sent = grid, None
        self._publish()

    def _publish(self) -> None:
        if self._grid is None:
            return
        try:
            t = self._tf.lookup_transform(self._map_frame, self._base_frame, Time(),
                                          timeout=Duration(seconds=0.1)).transform.translation
            b = (t.x - MARGIN_M, t.y - MARGIN_M, t.x + MARGIN_M, t.y + MARGIN_M)
            if self._box is not None:
                b = (min(b[0], self._box[0]), min(b[1], self._box[1]),
                     max(b[2], self._box[2]), max(b[3], self._box[3]))
            self._box = b
        except Exception:  # noqa: BLE001 -- no pose this tick: keep the last box
            pass
        out = cover(self._grid, self._box) if self._box else self._grid
        shape = (out.info.width, out.info.height,
                 out.info.origin.position.x, out.info.origin.position.y)
        if shape != self._sent:
            self._pub.publish(out)
            self._sent = shape


def main() -> None:
    rclpy.init()
    node = MapCoverRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
