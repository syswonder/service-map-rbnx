#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""
Offline benchmark for mapping_rbnx.global_match on the public PRBonn/IR-MCL
2D-LiDAR datasets (Intel Lab, Freiburg 079, MIT CSAIL).

No ROS runtime is required. Each frame is treated independently:
    saved occupancy map + one LaserScan -> global_scan_match -> compare with GT.

The PRBonn converted datasets store:
  - occmap.npy
  - train.json / val.json / test.json
where test.json contains LiDAR parameters and each scan contains
transform_matrix + range_readings.

Important:
  IR-MCL's own visualizer interprets occmap.npy as occmap[x, y], with
  0.05 m/cell and the array centre at world (0, 0). We convert that into
  RoboNix _Grid's row-major [y, x] representation here.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np

# Make this script runnable directly from service-map-rbnx/scripts/.
REPO_ROOT = Path(__file__).resolve().parents[1]
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mapping_rbnx import global_match, localizers  # noqa: E402


@dataclass
class Trial:
    index: int
    gt_x: float
    gt_y: float
    gt_yaw: float
    pred_x: float | None
    pred_y: float | None
    pred_yaw: float | None
    score: float | None
    runner_up: float | None
    margin: float | None
    pos_err: float | None
    yaw_err_deg: float | None
    top1_correct: bool
    top3_correct: bool
    top5_correct: bool
    accepted: bool
    result: str
    runtime_s: float
    detail: str


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_error_deg(a: float, b: float) -> float:
    return abs(math.degrees(wrap_angle(a - b)))


def load_prbonn_grid(dataset_dir: Path, resolution: float):
    map_path = dataset_dir / "occmap.npy"
    occmap = np.load(map_path)
    if occmap.ndim != 2:
        raise ValueError(f"{map_path} must be a 2-D array, got {occmap.shape}")

    # PRBonn/IR-MCL visualization convention:
    #   occmap[x_index, y_index]
    #   world x = (x_index - shape[0]//2) * resolution
    #   world y = (y_index - shape[1]//2) * resolution
    #
    # Observed semantics in their visualizer:
    #   > 0.9 : occupied
    #   ==0.5 : unknown
    #   other : known/free (typically 0.0)
    occupied_xy = occmap > 0.9
    unknown_xy = np.isclose(occmap, 0.5, atol=1e-6)
    known_xy = ~unknown_xy

    # RoboNix _Grid is row-major [y, x], and row index grows with world +y.
    occupied_yx = occupied_xy.T.astype(np.uint8)
    known_yx = known_xy.T.astype(np.uint8)

    width = occmap.shape[0]
    height = occmap.shape[1]
    cx = width // 2
    cy = height // 2
    origin = (-cx * resolution, -cy * resolution)

    grid = localizers._Grid(
        width,
        height,
        bytearray(occupied_yx.tobytes()),
        bytearray(known_yx.tobytes()),
        resolution,
        origin,
    )

    vals, counts = np.unique(occmap, return_counts=True)
    pairs = sorted(zip(counts.tolist(), vals.tolist()), reverse=True)[:8]
    common = ", ".join(f"{v:g}:{n}" for n, v in pairs)
    print(
        f"[map] {map_path.name}: shape={occmap.shape}, res={resolution:.3f} m, "
        f"origin=({origin[0]:.2f}, {origin[1]:.2f})"
    )
    print(
        f"[map] occupied={occupied_xy.mean():.1%}, unknown={unknown_xy.mean():.1%}, "
        f"common values(value:count)={common}"
    )
    return grid


def load_meta(dataset_dir: Path, split: str) -> dict:
    path = dataset_dir / f"{split}.json"
    with path.open("r", encoding="utf-8") as f:
        meta = json.load(f)

    required = ("num_beams", "angle_min", "angle_res", "max_range", "scans")
    missing = [k for k in required if k not in meta]
    if missing:
        raise ValueError(f"{path} missing keys: {missing}")
    if not meta["scans"]:
        raise ValueError(f"{path} contains no scans")

    print(
        f"[data] {path.name}: frames={len(meta['scans'])}, "
        f"beams={meta['num_beams']}, "
        f"angle=[{math.degrees(meta['angle_min']):.1f}°, "
        f"{math.degrees(meta.get('angle_max', meta['angle_min'] + meta['angle_res'] * meta['num_beams'])):.1f}°], "
        f"max_range={meta['max_range']} m"
    )
    return meta


def frame_pose(frame: dict) -> tuple[float, float, float]:
    T = np.asarray(frame["transform_matrix"], dtype=np.float64)
    if T.ndim != 2 or T.shape[0] < 2 or T.shape[1] < 3:
        raise ValueError(f"bad transform_matrix shape: {T.shape}")
    x = float(T[0, 2])
    y = float(T[1, 2])
    yaw = math.atan2(float(T[1, 0]), float(T[0, 0]))
    return x, y, wrap_angle(yaw)


def make_scan(meta: dict, frame: dict):
    ranges = list(map(float, frame["range_readings"]))
    return SimpleNamespace(
        ranges=ranges,
        angle_min=float(meta["angle_min"]),
        angle_increment=float(meta["angle_res"]),
        range_min=0.02,
        range_max=float(meta["max_range"]),
    )


def choose_indices(n: int, samples: int, sampling: str, seed: int) -> list[int]:
    if samples <= 0 or samples >= n:
        return list(range(n))
    if sampling == "uniform":
        return np.linspace(0, n - 1, samples, dtype=np.int64).tolist()
    rng = np.random.default_rng(seed)
    return sorted(rng.choice(n, size=samples, replace=False).tolist())


def grid_cell_status(grid, x: float, y: float) -> str:
    col = int((x - grid.origin[0]) / grid.resolution)
    row = int((y - grid.origin[1]) / grid.resolution)
    if not (0 <= col < grid.width and 0 <= row < grid.height):
        return "outside"
    i = row * grid.width + col
    if grid.cells[i]:
        return "occupied"
    if grid.known[i]:
        return "free"
    return "unknown"


def run_trial(
    grid,
    meta: dict,
    frame: dict,
    index: int,
    pos_thresh: float,
    yaw_thresh_deg: float,
    min_score: float,
    min_margin: float,
) -> Trial:
    gt_x, gt_y, gt_yaw = frame_pose(frame)
    scan = make_scan(meta, frame)

    t0 = time.perf_counter()
    match = global_match.global_scan_match(grid, scan, sensor_xy=(0.0, 0.0))
    runtime = time.perf_counter() - t0

    if match is None:
        return Trial(
            index, gt_x, gt_y, gt_yaw,
            None, None, None, None, None, None, None, None,
            False, False, False, False, "no_match", runtime,
            "global_scan_match returned None",
        )

    px, py, pyaw = match.pose
    pos_err = math.hypot(px - gt_x, py - gt_y)
    yaw_err = yaw_error_deg(pyaw, gt_yaw)
    correct = pos_err < pos_thresh and yaw_err < yaw_thresh_deg
    candidate_correct = [
        math.hypot(candidate.pose[0] - gt_x, candidate.pose[1] - gt_y) < pos_thresh
        and yaw_error_deg(candidate.pose[2], gt_yaw) < yaw_thresh_deg
        for candidate in match.candidates
    ]
    margin = float(match.score - match.runner_up)

    if match.score < min_score:
        accepted = False
        result = "low_score_reject"
    elif margin < min_margin:
        accepted = False
        result = "ambiguous"
    else:
        accepted = True
        result = "correct_accept" if correct else "WRONG_ACCEPT"

    return Trial(
        index=index,
        gt_x=gt_x,
        gt_y=gt_y,
        gt_yaw=gt_yaw,
        pred_x=float(px),
        pred_y=float(py),
        pred_yaw=float(pyaw),
        score=float(match.score),
        runner_up=float(match.runner_up),
        margin=margin,
        pos_err=pos_err,
        yaw_err_deg=yaw_err,
        top1_correct=correct,
        top3_correct=any(candidate_correct[:3]),
        top5_correct=any(candidate_correct[:5]),
        accepted=accepted,
        result=result,
        runtime_s=runtime,
        detail=str(match.detail),
    )


def save_csv(path: Path, trials: list[Trial]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(Trial.__dataclass_fields__.keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for t in trials:
            d = t.__dict__.copy()
            for k in ("gt_yaw", "pred_yaw"):
                if d[k] is not None:
                    d[k] = math.degrees(d[k])
            w.writerow(d)


def summarize(
    trials: list[Trial],
    strict_pos: float,
    strict_yaw: float,
) -> None:
    n = len(trials)
    with_match = [t for t in trials if t.pred_x is not None]
    accepted = [t for t in trials if t.accepted]
    wrong_accept = [t for t in trials if t.result == "WRONG_ACCEPT"]
    ambiguous = [t for t in trials if t.result == "ambiguous"]
    low_score = [t for t in trials if t.result == "low_score_reject"]
    no_match = [t for t in trials if t.result == "no_match"]
    top1 = [t for t in trials if t.top1_correct]
    top3 = [t for t in trials if t.top3_correct]
    top5 = [t for t in trials if t.top5_correct]

    strict = [
        t for t in with_match
        if t.pos_err is not None
        and t.yaw_err_deg is not None
        and t.pos_err < strict_pos
        and t.yaw_err_deg < strict_yaw
    ]

    runtimes = np.array([t.runtime_s for t in trials], dtype=float)

    print("\n=== Summary ===")
    print(f"Trials:                         {n}")
    print(f"Top-1 success @ configured:    {len(top1):4d}/{n} = {len(top1)/n:.1%}")
    print(f"Top-3 success @ configured:    {len(top3):4d}/{n} = {len(top3)/n:.1%}")
    print(f"Top-5 success @ configured:    {len(top5):4d}/{n} = {len(top5)/n:.1%}")
    print(
        f"Top-1 success @ {strict_pos:.2f}m/{strict_yaw:.1f}deg: "
        f"{len(strict):4d}/{n} = {len(strict)/n:.1%}"
    )
    print(f"Correct accepted:               {sum(t.result == 'correct_accept' for t in trials):4d}")
    print(f"WRONG accepted:                 {len(wrong_accept):4d}")
    print(f"Ambiguous/rejected by margin:   {len(ambiguous):4d}")
    print(f"Rejected by low score:          {len(low_score):4d}")
    print(f"No match:                       {len(no_match):4d}")
    if accepted:
        precision = 1.0 - len(wrong_accept) / len(accepted)
        print(f"Accepted-pose precision:        {precision:.1%}")
    print(f"Mean runtime / scan:            {runtimes.mean():.3f} s")
    print(f"Median runtime / scan:          {np.median(runtimes):.3f} s")

    if with_match:
        pos = np.array([t.pos_err for t in with_match], dtype=float)
        yaw = np.array([t.yaw_err_deg for t in with_match], dtype=float)
        print(f"Median position error:          {np.median(pos):.3f} m")
        print(f"Median yaw error:               {np.median(yaw):.2f} deg")


def main() -> int:
    p = argparse.ArgumentParser(
        description="Benchmark RoboNix stationary global_scan_match on PRBonn 2D datasets."
    )
    p.add_argument("--dataset-dir", required=True, type=Path,
                   help="Dataset directory containing occmap.npy and the selected split JSON")
    p.add_argument("--split", default="test", choices=("train", "val", "test"))
    p.add_argument("--samples", type=int, default=5,
                   help="Number of independent scans to test; <=0 means all.")
    p.add_argument("--sampling", choices=("uniform", "random"), default="uniform")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--map-res", type=float, default=0.05,
                   help="PRBonn converted OGM resolution in metres/cell.")
    p.add_argument("--pos-thresh", type=float, default=1.0)
    p.add_argument("--yaw-thresh-deg", type=float, default=15.0)
    p.add_argument("--strict-pos", type=float, default=0.5)
    p.add_argument("--strict-yaw-deg", type=float, default=10.0)
    p.add_argument("--min-score", type=float, default=localizers.STATIC_MATCH_MIN)
    p.add_argument("--min-margin", type=float, default=localizers.STATIC_MATCH_MARGIN)
    p.add_argument("--csv-out", type=Path, default=None)
    args = p.parse_args()

    dataset_dir = args.dataset_dir.expanduser().resolve()
    if not dataset_dir.is_dir():
        raise SystemExit(f"dataset directory not found: {dataset_dir}")

    grid = load_prbonn_grid(dataset_dir, args.map_res)
    meta = load_meta(dataset_dir, args.split)
    frames = meta["scans"]
    indices = choose_indices(len(frames), args.samples, args.sampling, args.seed)

    # Cheap preflight before the expensive matching loop.
    statuses = [grid_cell_status(grid, *frame_pose(frames[i])[:2]) for i in indices]
    print(
        "[preflight] GT pose cells: "
        + ", ".join(f"{s}={statuses.count(s)}" for s in ("free", "unknown", "occupied", "outside"))
    )
    if statuses.count("outside") > len(statuses) // 2:
        print(
            "[warning] Most GT poses lie outside the converted map. "
            "The dataset/map coordinate convention likely differs from the expected PRBonn format."
        )

    print(
        f"[eval] samples={len(indices)}, top1 threshold="
        f"{args.pos_thresh:.2f} m/{args.yaw_thresh_deg:.1f} deg, "
        f"accept score>={args.min_score:.2f}, margin>={args.min_margin:.2f}"
    )
    print()
    print(
        f"{'#':>4} {'frame':>7} {'pos(m)':>8} {'yaw(deg)':>9} "
        f"{'score':>7} {'margin':>7} {'time(s)':>8}  result"
    )

    trials: list[Trial] = []
    for k, idx in enumerate(indices, 1):
        trial = run_trial(
            grid, meta, frames[idx], idx,
            args.pos_thresh, args.yaw_thresh_deg,
            args.min_score, args.min_margin,
        )
        trials.append(trial)

        pos = "-" if trial.pos_err is None else f"{trial.pos_err:.3f}"
        yaw = "-" if trial.yaw_err_deg is None else f"{trial.yaw_err_deg:.1f}"
        score = "-" if trial.score is None else f"{trial.score:.3f}"
        margin = "-" if trial.margin is None else f"{trial.margin:.3f}"
        print(
            f"{k:4d} {idx:7d} {pos:>8} {yaw:>9} "
            f"{score:>7} {margin:>7} {trial.runtime_s:8.3f}  {trial.result}"
        )

    summarize(trials, args.strict_pos, args.strict_yaw_deg)

    csv_out = args.csv_out
    if csv_out is None:
        csv_out = REPO_ROOT / "benchmark_results" / f"{dataset_dir.name}_{args.split}.csv"
    save_csv(csv_out, trials)
    print(f"\nCSV: {csv_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
