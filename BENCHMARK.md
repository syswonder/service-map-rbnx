# Offline global localization benchmark

`scripts/benchmark_global_match.py` evaluates a saved occupancy map and one
stationary 2D LiDAR scan at a time. It calls `global_scan_match` without ROS and
compares the returned pose hypotheses with the dataset ground truth. This tests
single-scan global localization; it does not test map building, tracking, or
robot motion.

## Dataset layout and setup

The experiments use the public 2D SLAM archive at
<https://www.ipb.uni-bonn.de/html/projects/kuang2023ral/2dslam.zip>. Each
dataset directory passed to `--dataset-dir` must contain `occmap.npy` and at
least the JSON file for the selected split (`test.json` by default). The
archive provides this layout:

```text
<dataset-root>/
├── intel/
│   ├── occmap.npy
│   ├── train.json
│   ├── val.json
│   └── test.json
├── fr079/                 # same four files
└── mit/                   # same four files
```

`<dataset-root>` can be anywhere. For a self-contained local setup, put it in
`service-map-rbnx/benchmark_data/`; this directory is ignored by Git:

```text
service-map-rbnx/
├── BENCHMARK.md
├── scripts/
│   ├── benchmark_global_match.py
│   └── show_map.py
├── benchmark_data/         # downloaded input, ignored by Git
│   ├── intel/
│   ├── fr079/
│   └── mit/
└── benchmark_results/      # generated CSV/plots, ignored by Git
```

From the repository root, prepare that layout with:

```bash
python3 -m venv .venv
.venv/bin/pip install numpy
mkdir -p benchmark_data
wget https://www.ipb.uni-bonn.de/html/projects/kuang2023ral/2dslam.zip \
  -O benchmark_data/2dslam.zip
unzip benchmark_data/2dslam.zip -d benchmark_data
```

The script treats `occmap.npy` as `[x, y]`, transposes it into RoboNix's
`[y, x]` grid, and uses the archive's 0.05 m/cell map resolution. Check the
printed GT cell preflight counts if you use another dataset or conversion.

## Run

From the repository root, a quick deterministic check uses five evenly spaced
scans:

```bash
.venv/bin/python scripts/benchmark_global_match.py \
  --dataset-dir benchmark_data/intel --split test --samples 5
```

Run every test scan in all three environments:

```bash
for dataset in intel fr079 mit; do
  .venv/bin/python scripts/benchmark_global_match.py \
    --dataset-dir "benchmark_data/$dataset" --split test --samples 0
done
```

For data outside the repository, pass its directory instead, for example
`--dataset-dir ../datasets/2dslam/intel` or an absolute path. Paths supplied
to `--dataset-dir` and `--csv-out` are resolved relative to the shell's current
working directory; `~` is expanded. The scripts find repository source files
from their own location, so input data does not need to be next to the scripts.

`--samples 0` evaluates the complete split. The default is five uniformly
spaced frames; `--sampling random --seed 42` selects a reproducible random
subset. `--csv-out PATH` changes the per-frame CSV destination, which defaults
to `benchmark_results/<dataset>_<split>.csv`. Repeated runs with the same
dataset and split overwrite that CSV. The result directory is ignored by Git.

Top-1, Top-3, and Top-5 success mean that at least one of the first 1, 3, or 5
returned hypotheses is within **1.0 m and 15°** of ground truth. The stricter
reported Top-1 metric uses **0.5 m and 10°**. Acceptance is a separate decision:
the best hypothesis needs score at least **0.75** and a score lead of at least
**0.08** over the best geometrically distinct rival. `WRONG_ACCEPT` means an
accepted Top-1 pose exceeds the configured error thresholds. The CSV records
per-frame errors, scores, acceptance, and runtime; yaw columns are in degrees.
Runtime includes matching and distance-field construction, but not dataset
loading. The matcher searches five distinct hypotheses, with a 0.2 m coarse
lattice, a 0.25 m coarse likelihood width, and polishing of each hypothesis.

To inspect an occupancy map, install Matplotlib and save a plot:

```bash
.venv/bin/pip install matplotlib
.venv/bin/python scripts/show_map.py --dataset-dir benchmark_data/intel \
  --output benchmark_results/intel_map.png
```

## Recorded full-split results

The completed experiments recorded the following for the final 0.2 m / Top-5 polish / coarse sigma 0.25 m configuration.
These are measurements from that experiment, not guaranteed performance on
another machine, map, or sensor. The raw CSVs stay outside Git.

| Test map | Scans | Top-1 @ 1 m / 15° | Top-3 | Top-5 | Correct accepted | Wrong accepted | Mean time / scan |
|---|---:|---:|---:|---:|---:|---:|---:|
| Intel Lab | 182 | 175 (96.2%) | 179 (98.4%) | 180 (98.9%) | 153 | 0 | 1.104 s |
| Freiburg 079 | 959 | 934 (97.4%) | 941 (98.1%) | 942 (98.2%) | 801 | 0 | 1.022 s |
| MIT CSAIL | 82 | 73 (89.0%) | 77 (93.9%) | 77 (93.9%) | 56 | 0 | 2.283 s |

The earlier 0.4 m coarse-lattice baseline achieved Top-1 rates of 73.6%,
70.2%, and 64.6% on Intel, Freiburg 079, and MIT respectively. This comparison
is from the same experimental record; it is not a claim about a new run.
