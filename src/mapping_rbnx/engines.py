# SPDX-License-Identifier: MulanPSL-2.0
"""Per-engine map operations behind one interface.

`map_ops` owns the engine-independent half of save / load / reset: the map
directory layout, the occupancy + cloud preview, metadata, atomic publish and
the lifecycle announcements. What differs per SLAM engine is only how its pose
graph is flushed to a file, how a saved graph is put back, and how a reset is
requested. Those three operations live here, one implementation per `algo`, so
adding an engine does not touch the callers.

Artifacts stay engine-tagged but the directory contract does not change: every
saved map is `{MAPPING_MAPS_DIR}/<map_id>/` with `occupancy.{pgm,yaml,png}`,
`cloud.pcd`, `meta.yaml` plus the engine's own graph file(s) — `rtabmap.db` for
RTAB-Map, `<map_id>.posegraph` + `<map_id>.data` for slam_toolbox.
"""
from __future__ import annotations

import logging
import os
import time
from typing import Callable, Optional, Protocol

log = logging.getLogger("mapping.engines")

# slam_toolbox's serialization writes `<stem>.posegraph` and `<stem>.data`; the
# stem is fixed rather than derived from map_id so a directory copied under a
# new name still loads.
SLAM_TOOLBOX_STEM = "posegraph"


class EngineOps(Protocol):
    """Engine-specific half of the map operations."""

    name: str
    graph_files: tuple[str, ...]

    def graph_ready(self, map_dir: str) -> tuple[bool, str]:
        """Is the saved graph in `map_dir` present and loadable?"""

    def snapshot(self, node, staging_dir: str, timeout_s: float) -> tuple[bool, str]:
        """Write the live graph into `staging_dir` (engine artifact names)."""

    def activate(self, node, map_dir: str, map_id: str, timeout_s: float,
                 pose: Optional[tuple[float, float, float]] = None) -> tuple[bool, str]:
        """Put the saved graph back and switch to localization, at `pose` when
        one is given and at the graph's first node otherwise."""

    def reset(self, node, timeout_s: float) -> tuple[bool, str]:
        """Discard the live graph and start a fresh mapping session."""

    def yield_map_frame(self, node, timeout_s: float) -> tuple[bool, str]:
        """Stop publishing `map -> odom` so a localizer can own it."""


def _call_service(node, srv_type, name: str, request, timeout_s: float):
    """Call a ROS service, returning (ok, response_or_detail). Kept local so
    this module does not import map_ops (which imports this one)."""
    try:
        client = node.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=min(timeout_s, 10.0)):
            return False, f"service {name} unavailable"
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False, f"service {name} timed out after {timeout_s:.0f}s"
        return True, future.result()
    except Exception as e:  # noqa: BLE001
        return False, f"service {name} raised {e}"


class SlamToolboxOps:
    """slam_toolbox (Karto scan matching + pose-graph, ROS 2 reference 2D SLAM).

    Serialization is `slam_toolbox/srv/SerializePoseGraph` (writes
    `<filename>.posegraph` + `<filename>.data`) and restoring is
    `slam_toolbox/srv/DeserializePoseGraph`, whose `match_type` decides how the
    robot re-enters the map: `LOCALIZE_AT_POSE` starts localization at a given
    pose, which is what `load_map` means here. A reset is
    `slam_toolbox/srv/Reset`, falling back to deserializing nothing when the
    running version predates that service.
    """

    name = "slam_toolbox"
    graph_files = (f"{SLAM_TOOLBOX_STEM}.posegraph", f"{SLAM_TOOLBOX_STEM}.data")

    def __init__(self, namespace: str = "") -> None:
        self._ns = (namespace or os.environ.get("MAPPING_SLAM_TOOLBOX_NS", "/slam_toolbox")).rstrip("/")

    def _srv(self, leaf: str) -> str:
        return f"{self._ns}/{leaf}"

    def graph_ready(self, map_dir: str) -> tuple[bool, str]:
        """Both serialization files must be there; slam_toolbox fails opaquely
        when only one is (the `.data` holds the scans, the `.posegraph` the
        graph)."""
        missing = [f for f in self.graph_files if not os.path.isfile(os.path.join(map_dir, f))]
        if missing:
            return False, f"missing {', '.join(missing)} in {map_dir}"
        size = sum(os.path.getsize(os.path.join(map_dir, f)) for f in self.graph_files)
        if size <= 0:
            return False, f"empty pose graph in {map_dir}"
        return True, f"pose graph ok ({size} bytes)"

    def snapshot(self, node, staging_dir: str, timeout_s: float) -> tuple[bool, str]:
        """Ask slam_toolbox to serialize into the staging directory and wait for
        both files to appear (the service returns before the write completes)."""
        try:
            from slam_toolbox.srv import SerializePoseGraph
        except Exception as e:  # noqa: BLE001
            return False, f"slam_toolbox service types unavailable: {e}"
        stem = os.path.join(staging_dir, SLAM_TOOLBOX_STEM)
        req = SerializePoseGraph.Request()
        req.filename = stem
        ok, resp = _call_service(node, SerializePoseGraph, self._srv("serialize_map"), req, timeout_s)
        if not ok:
            return False, str(resp)
        deadline = time.monotonic() + max(5.0, timeout_s / 4.0)
        while time.monotonic() < deadline:
            ready, detail = self.graph_ready(staging_dir)
            if ready:
                return True, f"serialized pose graph ({detail})"
            time.sleep(0.2)
        return False, f"serialize_map returned but {stem}.posegraph/.data did not appear"

    def activate(self, node, map_dir: str, map_id: str, timeout_s: float,
                 pose: Optional[tuple[float, float, float]] = None) -> tuple[bool, str]:
        """Deserialize the saved graph and localize in it. Without a pose the
        robot is placed at the first node of the saved graph, which is what an
        operator means by "load the map I recorded from the dock"."""
        try:
            from slam_toolbox.srv import DeserializePoseGraph
        except Exception as e:  # noqa: BLE001
            return False, f"slam_toolbox service types unavailable: {e}"
        ready, detail = self.graph_ready(map_dir)
        if not ready:
            return False, detail
        req = DeserializePoseGraph.Request()
        req.filename = os.path.join(map_dir, SLAM_TOOLBOX_STEM)
        if pose is not None:
            req.match_type = DeserializePoseGraph.Request.LOCALIZE_AT_POSE
            req.initial_pose.x, req.initial_pose.y, req.initial_pose.theta = (
                float(pose[0]), float(pose[1]), float(pose[2]))
        else:
            req.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
        ok, resp = _call_service(node, DeserializePoseGraph, self._srv("deserialize_map"), req, timeout_s)
        if not ok:
            return False, str(resp)
        return True, f"deserialized {req.filename} ({'localize at pose' if pose else 'first node'})"

    def yield_map_frame(self, node, timeout_s: float) -> tuple[bool, str]:
        """Stop publishing `map -> odom`.

        Two nodes publishing the same tf edge do not average — they overwrite
        each other, and the robot teleports between their two answers several
        times a second. When a particle filter takes over on a saved map, the
        SLAM engine has to let go, which for slam_toolbox means pausing its
        graph updates (`pause_new_measurements`) so it stops emitting the
        transform.
        """
        try:
            from slam_toolbox.srv import Pause
        except Exception as e:  # noqa: BLE001
            return False, f"slam_toolbox Pause service type unavailable ({e})"
        ok, resp = _call_service(node, Pause, self._srv("pause_new_measurements"),
                                 Pause.Request(), timeout_s)
        if not ok:
            return False, str(resp)
        return True, "slam_toolbox paused; the localizer owns map -> odom"

    def reset(self, node, timeout_s: float) -> tuple[bool, str]:
        """Clear the live graph. `Reset` exists from slam_toolbox 2.6; older
        builds are told so rather than silently continuing on a stale map."""
        try:
            from slam_toolbox.srv import Reset
        except Exception as e:  # noqa: BLE001
            return False, f"slam_toolbox Reset service type unavailable ({e}); upgrade slam_toolbox to reset in place"
        ok, resp = _call_service(node, Reset, self._srv("reset"), Reset.Request(), timeout_s)
        if not ok:
            return False, str(resp)
        return True, "slam_toolbox reset"


class RtabmapOps:
    """RTAB-Map, delegating to the implementations that live in `map_ops`.

    The functions are injected rather than imported so this module stays free of
    a circular import; `map_ops` registers them at import time.
    """

    name = "rtabmap"
    graph_files = ("rtabmap.db",)

    def __init__(self, *, graph_ready: Callable, snapshot: Callable,
                 activate: Callable, reset: Callable) -> None:
        self._graph_ready = graph_ready
        self._snapshot = snapshot
        self._activate = activate
        self._reset = reset

    def graph_ready(self, map_dir: str) -> tuple[bool, str]:
        return self._graph_ready(map_dir)

    def snapshot(self, node, staging_dir: str, timeout_s: float) -> tuple[bool, str]:
        return self._snapshot(node, staging_dir, timeout_s)

    def activate(self, node, map_dir: str, map_id: str, timeout_s: float,
                 pose: Optional[tuple[float, float, float]] = None) -> tuple[bool, str]:
        return self._activate(node, map_dir, map_id, timeout_s, pose)

    def reset(self, node, timeout_s: float) -> tuple[bool, str]:
        return self._reset(node, timeout_s)

    def yield_map_frame(self, node, timeout_s: float) -> tuple[bool, str]:
        """RTAB-Map is put in localization mode by `load_map` and keeps owning
        the transform there, so there is nothing to hand over."""
        return True, "rtabmap keeps map -> odom in localization mode"


_REGISTRY: dict[str, EngineOps] = {}


def register(ops: EngineOps) -> None:
    """Make `ops` the implementation for its `name`."""
    _REGISTRY[ops.name] = ops


def engine_for(algo: str) -> Optional[EngineOps]:
    """Return the operations for `algo`, or None when the engine does not
    implement map persistence (dlio / fastlio2 are odometry-only)."""
    return _REGISTRY.get((algo or "").strip().lower())


def graph_files_for(algo: str) -> tuple[str, ...]:
    """Artifact filenames the engine writes inside a saved map directory."""
    ops = engine_for(algo)
    return tuple(getattr(ops, "graph_files", ())) if ops else ()


register(SlamToolboxOps())
