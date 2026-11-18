#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
pkg_dir="$(cd "$(dirname "$0")/.." && pwd)"
cmake -S "$pkg_dir/native/slam_graph_tool" -B "$pkg_dir/rbnx-build/slam-graph-tool" -DCMAKE_BUILD_TYPE=Release
cmake --build "$pkg_dir/rbnx-build/slam-graph-tool" -j2
