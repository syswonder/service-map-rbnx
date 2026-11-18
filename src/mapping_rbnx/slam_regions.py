# SPDX-License-Identifier: MulanPSL-2.0
"""Create an immutable SLAM Toolbox branch with selected observations retired."""
from __future__ import annotations
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import yaml
from .grid_edits import map_directory
from .slam_checkpoints import GRAPH_FILES, inspect_graph

def bounds_of(value):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError('bounds must be [xmin, ymin, xmax, ymax] in map meters')
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) for x in value):
        raise ValueError('bounds must contain numbers')
    x0, y0, x1, y1 = map(float, value)
    if not all(math.isfinite(x) for x in (x0, y0, x1, y1)) or x0 >= x1 or y0 >= y1:
        raise ValueError('bounds must be finite and have positive width and height')
    return [x0, y0, x1, y1]

def fingerprint(source):
    return {name: hashlib.sha256((source/name).read_bytes()).hexdigest() for name in GRAPH_FILES}

def retained_start(directory, pose, grid):
    """Return a reason if a local rebuild is being initialized in retired space."""
    path = Path(directory)/'local-update.json'
    if not path.is_file():
        return None
    manifest = json.loads(path.read_text())
    x0,y0,x1,y1 = bounds_of(manifest['bounds'])
    margin = float(manifest['margin_m'])
    x,y = pose[:2]
    if x0-margin <= x <= x1+margin and y0-margin <= y <= y1+margin:
        return 'start local rebuilding outside the selected region, in retained observed space'
    index = grid._index(x,y)
    if index is None or not grid.known[index] or grid.cells[index]:
        return 'start local rebuilding on retained known free space'
    return None

def prepare(root, base_map_id, map_id, bounds):
    """Publish a new map, never modify the source or start robot movement.

    The selected historical rays are invalidated in the native dataset. All
    graph vertices and edges remain. Initialization and incremental mapping
    subsequently use the usual verified mapping workflow on the derived map.
    """
    root = Path(root).resolve()
    source, destination = map_directory(root, base_map_id), map_directory(root, map_id)
    bounds = bounds_of(bounds)
    if source == destination or destination.exists():
        raise ValueError('choose a new map_id; existing maps are immutable')
    meta = yaml.safe_load((source/'meta.yaml').read_text()) or {}
    if meta.get('engine') != 'slam_toolbox':
        raise ValueError('local graph updates are only supported for slam_toolbox')
    active = source/'.grid-edits/active'
    if active.exists() and active.read_text().strip() != 'original':
        raise ValueError('this map has separate raster edits; use an unedited graph map for local rebuilding')
    occupancy = yaml.safe_load((source/'occupancy.yaml').read_text())
    resolution = float(occupancy['resolution'])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('invalid map resolution')
    tool = os.environ.get('MAPPING_SLAM_REGION_TOOL', str(
        Path(__file__).resolve().parents[2]/'rbnx-build/slam-graph-tool/slam_region_tool'))
    root.mkdir(parents=True, exist_ok=True)
    with (root/'.region-update.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if destination.exists():
            raise ValueError('map_id already exists')
        hashes = fingerprint(source)
        with tempfile.TemporaryDirectory(prefix='.region-', dir=root) as temporary:
            work = Path(temporary)
            original = work/'original'; original.mkdir()
            for name in GRAPH_FILES:
                shutil.copy2(source/name, original/name)
            before = inspect_graph(original, resolution)
            baseline_nodes = (original/'nodes.jsonl').read_bytes()
            output = work/'output'; output.mkdir()
            proc = subprocess.run([tool, str(original/'posegraph'), str(output),
                *map(str, bounds), str(resolution)], capture_output=True, text=True, timeout=180)
            if proc.returncode:
                raise ValueError('region edit failed: '+proc.stderr[-1000:])
            verified = subprocess.run([tool, '--verify-pair', str(output/'posegraph')],
                capture_output=True, text=True, timeout=120)
            if verified.returncode:
                raise ValueError('edited Mapper/Dataset verification failed: '+verified.stderr[-1000:])
            after = inspect_graph(output, resolution)
            if after['nodes'] != before['nodes'] or after['edges'] != before['edges']:
                raise ValueError('region edit changed graph topology')
            if (output/'nodes.jsonl').read_bytes() != baseline_nodes:
                raise ValueError('region edit changed node identities or poses')
            if fingerprint(source) != hashes:
                raise ValueError('source graph changed while preparing update')
            native = json.loads((output/'region-native.json').read_text())
            manifest = dict(schema=1, source_map_id=base_map_id, map_id=map_id,
                created_at=time.time(), bounds=bounds, margin_m=resolution,
                source_sha256=hashes, source_graph=before, prepared_graph=after,
                policy='invalidate_intersecting_ray', native=native, native_pair_verified=True,
                start_requirement='initialize in retained observed space; do not start in the cleared region')
            (output/'local-update.json').write_text(json.dumps(manifest, indent=2))
            for name in ('sensor.json',):
                if (source/name).is_file():
                    shutil.copy2(source/name, output/name)
            meta.update(map_id=map_id, engine='slam_toolbox', source_map_id=base_map_id,
                local_update=True, note='Local rebuild branch; start in retained observed space')
            (output/'meta.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))
            for path in output.iterdir():
                if path.is_file():
                    with path.open('rb') as stream:
                        os.fsync(stream.fileno())
            descriptor = os.open(output, os.O_RDONLY|os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            if destination.exists():
                raise ValueError('map_id already exists')
            os.rename(output, destination)
            descriptor = os.open(root, os.O_RDONLY|os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
    return dict(ok=True, map_id=map_id, base_map_id=base_map_id,
        artifact_path=str(destination), local_update=manifest,
        detail='Local rebuild branch prepared; initialize mapping in retained observed space')
