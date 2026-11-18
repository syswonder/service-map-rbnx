# SPDX-License-Identifier: MulanPSL-2.0
"""Immutable slam_toolbox graph checkpoints; no live-grid capture or pause.

serialize_map takes the engine's graph mutex for a consistent pair. Everything
else (decode, rendering, hashing, fsync) happens in a separate worker/process.
The robot never receives a stop command. Serialization is O(graph size), not a
zero-cost incremental journal; capture_ms is exposed so operators can size it.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time
import uuid

GRAPH_FILES = ("posegraph.posegraph", "posegraph.data")


class MappingDiverged(RuntimeError):
    pass


def validate_reference(directory,metadata):
    reference=metadata.get("reference_nodes",{})
    if not reference:
        return {}
    nodes={str(n["id"]):n["pose"] for n in map(json.loads,(Path(directory)/"nodes.jsonl").read_text().splitlines())}
    if not reference.keys() <= nodes.keys():
        raise MappingDiverged("baseline graph lost nodes")
    shift=max(math.hypot(nodes[k][0]-v[0],nodes[k][1]-v[1]) for k,v in reference.items())
    angle=max(abs(math.atan2(math.sin(nodes[k][2]-v[2]),math.cos(nodes[k][2]-v[2]))) for k,v in reference.items())
    if shift>metadata.get("max_baseline_shift_m",.5) or angle>math.radians(15):
        raise MappingDiverged(f"baseline graph deformation requires review ({shift:.2f} m, {math.degrees(angle):.1f} deg)")
    return {"max_baseline_shift_m":shift,"max_baseline_yaw_rad":angle}


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[a-f0-9]{32}", value):
        raise ValueError("invalid checkpoint/session id")
    return value


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def fsync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x") as f:
            json.dump(value, f, allow_nan=False, indent=2)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary, path)
        fsync_dir(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def graph_tool():
    return os.environ.get("MAPPING_SLAM_GRAPH_TOOL", str(
        Path(__file__).resolve().parents[2]/"rbnx-build/slam-graph-tool/slam_graph_tool"))


def inspect_graph(directory, resolution=.05):
    directory = Path(directory)
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("invalid map resolution")
    for name in GRAPH_FILES:
        p = directory/name
        if not p.is_file() or p.stat().st_size == 0:
            raise ValueError(f"missing or empty {name}")
    proc = subprocess.run([graph_tool(), str(directory/"posegraph"), str(directory), str(resolution)],
                          capture_output=True, text=True, timeout=120)
    if proc.returncode:
        raise ValueError(f"graph cold-load failed: {proc.stderr[-500:]}")
    graph = json.loads((directory/"graph.json").read_text())
    if graph["nodes"] < 1 or graph["components"] != 1:
        raise ValueError(f"graph must be nonempty and connected: {graph}")
    # Preview comes from these saved scans, never a separately sampled /map.
    from PIL import Image
    with Image.open(directory/"occupancy.pgm") as image:
        image.save(directory/"occupancy.png")
    return graph


class CheckpointStore:
    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)

    def path(self, session_id, checkpoint_id):
        return self.root/identifier(session_id)/identifier(checkpoint_id)

    def validate(self, session_id, checkpoint_id):
        directory = self.path(session_id, checkpoint_id)
        manifest = json.loads((directory/"manifest.json").read_text())
        if manifest.get("session_id") != session_id or manifest.get("checkpoint_id") != checkpoint_id:
            raise ValueError("checkpoint identity mismatch")
        if manifest.get("engine") != "slam_toolbox" or manifest.get("schema") != 1:
            raise ValueError("unsupported checkpoint format")
        expected = set(GRAPH_FILES) | {"occupancy.pgm", "occupancy.yaml", "occupancy.png", "graph.json", "nodes.jsonl"}
        if manifest.get("metadata", {}).get("sensor"):
            expected.add("sensor.json")
        if set(manifest.get("files", {})) != expected:
            raise ValueError("incomplete checkpoint manifest")
        for name, record in manifest["files"].items():
            p = directory/name
            if p.is_symlink() or not p.is_file() or p.stat().st_size != record["bytes"] or digest(p) != record["sha256"]:
                raise ValueError(f"checkpoint integrity failed: {name}")
        return directory, manifest

    def list(self, session_id=None):
        sessions = [self.root/identifier(session_id)] if session_id else [p for p in self.root.iterdir() if re.fullmatch(r"[a-f0-9]{32}", p.name)]
        items = []
        for session in sessions:
            for p in session.glob("*/manifest.json"):
                if not re.fullmatch(r"[a-f0-9]{32}", p.parent.name):
                    continue
                try:
                    m = json.loads(p.read_text())
                    items.append({k: m[k] for k in ("session_id", "checkpoint_id", "created_at", "reason", "graph", "capture_ms", "total_ms", "parent_id")})
                except (OSError, ValueError, KeyError):
                    continue
        return sorted(items, key=lambda m: m["created_at"], reverse=True)

    def create(self, session_id, capture, *, metadata, reason, parent_id=None, resolution=.05, before_commit=None):
        session_dir = self.root/identifier(session_id)
        session_dir.mkdir(exist_ok=True)
        checkpoint_id = uuid.uuid4().hex
        staging = session_dir/f".{checkpoint_id}.staging"
        staging.mkdir()
        started = time.monotonic()
        try:
            captured_metadata = capture(staging) or {}
            capture_ms = (time.monotonic()-started)*1000
            resolution = captured_metadata.get("engine_parameters", {}).get("resolution", resolution)
            graph = inspect_graph(staging, resolution)
            graph.update(validate_reference(staging,metadata))
            files = {}
            for p in staging.iterdir():
                if p.is_file():
                    files[p.name] = {"bytes": p.stat().st_size, "sha256": digest(p)}
                    with p.open("rb") as f:
                        os.fsync(f.fileno())
            manifest = {"schema": 1, "engine": "slam_toolbox", "session_id": session_id,
                        "checkpoint_id": checkpoint_id, "parent_id": parent_id,
                        "created_at": time.time(), "reason": reason, "graph": graph,
                        "capture_ms": capture_ms, "total_ms": (time.monotonic()-started)*1000,
                        "metadata": {**metadata, **captured_metadata}, "files": files}
            atomic_json(staging/"manifest.json", manifest)
            fsync_dir(staging)
            if before_commit:
                before_commit()
            destination = session_dir/checkpoint_id
            os.rename(staging, destination)
            fsync_dir(session_dir)
            # A crash before this pointer update still leaves a discoverable,
            # valid manifest. Never point at an incomplete staging directory.
            atomic_json(session_dir/"latest.json", {"checkpoint_id": checkpoint_id})
            return manifest
        finally:
            shutil.rmtree(staging, ignore_errors=True)


class Coverage:
    """Count newly known cells in a fixed world lattice, not image dimensions.

    The lattice resolution is fixed per session. Geometry/TF frame changes
    must start another session. Unknown bounding-box padding does not count.
    """
    def __init__(self):
        self.cells = set()
        self.resolution = None
        self.frame = None

    def observe(self, msg):
        import numpy as np
        info = msg.info
        res = float(info.resolution)
        if not math.isfinite(res) or res <= 0 or len(msg.data) != info.width*info.height:
            raise ValueError("invalid occupancy grid")
        q = info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if self.resolution is None:
            self.resolution, self.frame = res, msg.header.frame_id
        if not math.isclose(res, self.resolution, rel_tol=1e-6) or msg.header.frame_id != self.frame:
            raise ValueError("map resolution/frame changed; start another session")
        known = np.flatnonzero(np.asarray(msg.data) >= 0)
        x, y = (known % info.width + .5)*res, (known // info.width + .5)*res
        wx = info.origin.position.x + math.cos(yaw)*x-math.sin(yaw)*y
        wy = info.origin.position.y + math.sin(yaw)*x+math.cos(yaw)*y
        # World lattice tolerates whole-cell origin shifts from map expansion.
        ix, iy = np.floor(wx/res+1e-7).astype(np.int64), np.floor(wy/res+1e-7).astype(np.int64)
        self.cells.update(zip(ix.tolist(), iy.tolist()))
        return len(self.cells)*res*res


class MappingSession:
    """One worker, one in-flight checkpoint, explicit lifecycle boundaries."""
    def __init__(self, root, capture, operation_lock, *, area_m2=20., min_interval_s=15., keep=20,
                 guard=None, on_suspect=None, max_interval_s=60.):
        if (not math.isfinite(area_m2) or area_m2 <= 0 or not math.isfinite(min_interval_s) or min_interval_s < 0
                or keep < 1 or int(keep) != keep or not math.isfinite(max_interval_s) or max_interval_s < min_interval_s):
            raise ValueError("invalid checkpoint policy")
        self.store = CheckpointStore(root)
        self.capture, self.operation_lock = capture, operation_lock
        self.area_m2, self.min_interval_s, self.keep = area_m2, min_interval_s, keep
        self.guard, self.on_suspect, self.max_interval_s = guard, on_suspect, max_interval_s
        self._lock = threading.RLock()
        self._wake, self._closed = threading.Event(), threading.Event()
        self._latest_grid = None
        self._request = None
        self._coverage = Coverage()
        self._state = {"state": "idle", "session_id": "", "base_map_id": "", "known_area_m2": 0.,
                       "new_area_m2": 0., "last_checkpoint": None, "error": ""}
        self._metadata = {}
        self._baseline_area = 0.
        self._last_saved = 0.
        self._thread = threading.Thread(target=self._work, name="slam-checkpoints", daemon=True)
        self._thread.start()

    def begin(self, *, base_map_id="", metadata=None, baseline_grid=None, parent=None):
        with self.operation_lock, self._lock:
            self._coverage = Coverage()
            self._baseline_area = self._coverage.observe(baseline_grid) if baseline_grid else 0.
            self._last_saved = time.monotonic()
            self._latest_grid = None
            self._request = None
            self._metadata = dict(metadata or {})
            self._metadata["restored_from"] = parent
            self._state = {"state": "mapping", "session_id": uuid.uuid4().hex, "base_map_id": base_map_id,
                           "known_area_m2": self._baseline_area, "new_area_m2": 0., "last_checkpoint": None, "error": "",
                           "started_at": time.time()}
            return self.status()

    def suspend(self):
        with self.operation_lock, self._lock:
            self._state["state"] = "idle"
            self._latest_grid = None
            self._request = None

    def status(self):
        with self._lock:
            return {**self._state, "area_threshold_m2": self.area_m2, "min_interval_s": self.min_interval_s}

    def observe(self, grid):
        with self._lock:
            if self._state["state"] in {"mapping", "saving"}:
                self._latest_grid = grid
                self._wake.set()

    def request(self, reason="manual"):
        with self._lock:
            if self._state["state"] != "mapping" or self._request:
                return {"ok": False, "detail": "mapping is inactive or a checkpoint is already pending"}
            self._request = reason
            self._wake.set()
            return {"ok": True, "detail": "checkpoint queued; mapping continues", "session_id": self._state["session_id"]}

    def close(self):
        self._closed.set()
        self._wake.set()
        self._thread.join(timeout=125)

    def check_health(self):
        if self.guard:
            reason = self.guard()
            if reason:
                raise MappingDiverged(reason)

    def _work(self):
        while not self._closed.is_set():
            self._wake.wait(.5)
            self._wake.clear()
            session_id = None
            try:
                with self.operation_lock:
                    with self._lock:
                        if self._state["state"] != "mapping":
                            continue
                        session_id = self._state["session_id"]
                        if self.guard:
                            reason = self.guard()
                            if reason:
                                raise MappingDiverged(reason)
                        grid, self._latest_grid = self._latest_grid, None
                        if grid is not None:
                            self._state["known_area_m2"] = self._coverage.observe(grid)
                        area = self._state["known_area_m2"]
                        self._state["new_area_m2"] = max(0., area-self._baseline_area)
                        due = self._state["new_area_m2"] >= self.area_m2 and time.monotonic()-self._last_saved >= self.min_interval_s
                        elapsed = time.monotonic()-self._last_saved
                        reason = self._request or ("area_growth" if due else
                            "periodic" if elapsed >= self.max_interval_s and self._coverage.cells else None)
                        if not reason:
                            continue
                        self._request = None
                        self._state["state"] = "saving"
                        session_id = self._state["session_id"]
                        previous = self._state["last_checkpoint"]
                        metadata = {**self._metadata, "base_map_id": self._state["base_map_id"], "coverage_area_m2": area}
                    saved = self.store.create(session_id, self.capture, metadata=metadata, reason=reason,
                                              parent_id=previous["checkpoint_id"] if previous else None,
                                              resolution=self._coverage.resolution or .05, before_commit=self.check_health)
                    with self._lock:
                        self._state.update(state="mapping", last_checkpoint=saved, error="")
                        self._baseline_area = area
                        self._state["new_area_m2"] = 0.
                        self._last_saved = time.monotonic()
                    # Keep the current session bounded; never delete another
                    # session's baseline or a checkpoint still being written.
                    for old in self.store.list(session_id)[self.keep:]:
                        shutil.rmtree(self.store.path(session_id, old["checkpoint_id"]))
                    fsync_dir(self.store.root/session_id)
            except MappingDiverged as exc:
                with self.operation_lock, self._lock:
                    if self._state["session_id"] != session_id:
                        continue
                    self._state.update(state="suspect", error=str(exc))
                    if self.on_suspect:
                        self.on_suspect(str(exc))
            except Exception as exc:
                with self._lock:
                    self._state.update(state="mapping", error=str(exc))
                    # Back off automatic retries after disk/engine failures.
                    self._last_saved = time.monotonic()
