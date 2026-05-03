#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx container entrypoint.
#
# Four processes run in this container, in order:
#   1. rmw_zenohd                 — Zenoh router; rmw_zenoh_cpp ROS2
#                                   nodes connect to it as transport.
#   2. zenoh-bridge-dds           — mirrors the sim container's
#                                   FastRTPS topics into Zenoh so our
#                                   rmw_zenoh-side subscribers see
#                                   /scanner / /odom / etc.
#   3. atlas_bridge               — Python: registers cap with atlas,
#                                   queries sensor primitives, writes
#                                   resolved.yaml, declares own outputs.
#   4. SLAM engine                — rtabmap (default) / dlio / fastlio2
#                                   (selected by config.algo via
#                                   /mapping/scripts/start_engine.sh).
#
# All four are siblings; we trap on EXIT so SIGTERM tears the lot down.

set -eo pipefail

source /opt/ros/humble/setup.bash

cd /mapping

# Codegen output (proto_gen) is at the package root by default
# (`rbnx codegen` writes <pkg>/proto_gen/). atlas_bridge needs the
# generated atlas_pb2 etc., so prepend it.
export PYTHONPATH="/mapping/src:/mapping/proto_gen:${PYTHONPATH:-}"
if [ -d /robonix-py ]; then
    export PYTHONPATH="/robonix-py:${PYTHONPATH}"
fi

mkdir -p /mapping/rbnx-build/data

# Direct-DDS path: same RMW (FastRTPS) as sim, --network host shares
# the IP namespace so UDP multicast discovery works, and --ipc=host
# shares /dev/shm so SHM data transfer lines up. No Zenoh router or
# bridge needed in this layout.
ZENOHD_PID=
BRIDGE_PID=

ENGINE_PID=
ATLAS_PID=

cleanup() {
    [ -n "$ENGINE_PID" ] && kill -TERM "$ENGINE_PID" 2>/dev/null || true
    [ -n "$ATLAS_PID" ]  && kill -TERM "$ATLAS_PID"  2>/dev/null || true
    kill -TERM "$BRIDGE_PID" 2>/dev/null || true
    kill -TERM "$ZENOHD_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 1

# ── 3. atlas_bridge ─────────────────────────────────────────────────────────
python3 -m mapping_rbnx.atlas_bridge 2>&1 | sed 's/^/[bridge] /' &
ATLAS_PID=$!

# Wait for atlas_bridge to choose an algo + write resolved config.
# atlas_bridge writes /tmp/mapping_algo (one-line algo name) once
# config is parsed AND /tmp/<algo>_resolved.yaml when atlas-discovered
# topics are available. Both are gating signals for start_engine.sh.
for _ in $(seq 1 30); do
    [ -f /tmp/mapping_algo ] && break
    sleep 0.5
done
ALGO="$(cat /tmp/mapping_algo 2>/dev/null || echo rtabmap)"
export MAPPING_ALGO="$ALGO"
RESOLVED="/tmp/${ALGO}_resolved.yaml"
for _ in $(seq 1 30); do
    [ -f "$RESOLVED" ] && break
    sleep 0.5
done

# ── 4. SLAM engine (selected by algo) ───────────────────────────────────────
bash /mapping/scripts/start_engine.sh 2>&1 | sed 's/^/[engine] /' &
ENGINE_PID=$!

wait "$ENGINE_PID"
