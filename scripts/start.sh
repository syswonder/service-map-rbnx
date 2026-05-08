#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx start phase — docker-run wrapper. Same pattern as
# system/scene/scripts/start.sh.
#
# Container shape: --network host + --ipc=host + FastRTPS UDP-only so
# the in-container rtabmap can subscribe to whatever sim/robot
# container is publishing scan + RGBD + odom on the host DDS bus.
#
# Trap discipline: when boot SIGTERMs our PGID, this trap stops the
# container so SLAM doesn't outlive the deploy.
set -euo pipefail

CT="${ROBONIX_MAPPING_CONTAINER:-robonix_mapping}"
IMG="${ROBONIX_MAPPING_IMAGE:-robonix-mapping}"

cleanup() {
    docker stop "$CT" >/dev/null 2>&1 || true
    kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Drop a stopped container from a previous run.
docker rm -f "$CT" >/dev/null 2>&1 || true

mkdir -p rbnx-build/data

declare -a EXTRA_MOUNTS=()
if [[ -n "${RBNX_CONFIG_FILE:-}" ]]; then
    EXTRA_MOUNTS+=(-v "${RBNX_CONFIG_FILE}:${RBNX_CONFIG_FILE}:ro")
fi

# X11 forwarding for rtabmap_viz inside the mapping container. We
# auto-detect DISPLAY when it's not in the env (the user ran
# `rbnx boot` from a fresh shell without exporting): probe the
# standard local Xorg slots, accept the first that responds. If
# none does, skip X11 wiring and rtabmap_viz won't render — the
# launch file's `enable_viz` flag still spawns it but Qt prints
# the "could not connect to display" warning we've seen before.
if [[ -z "${DISPLAY:-}" ]]; then
    if command -v xset &>/dev/null; then
        for d in :0 :1 :10; do
            if DISPLAY="$d" xset q &>/dev/null; then
                export DISPLAY="$d"
                break
            fi
        done
    fi
fi

declare -a X11_ARGS=()
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
    xhost +local:docker >/dev/null 2>&1 || true
    X11_ARGS=(
        -e DISPLAY="$DISPLAY"
        -e QT_X11_NO_MITSHM=1
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    )
fi

exec docker run --rm \
    --name "$CT" \
    --network host \
    --ipc=host \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-mapping}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e RBNX_CONFIG_FILE="${RBNX_CONFIG_FILE:-}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e MAPPING_GRPC_PORT="${MAPPING_GRPC_PORT:-50120}" \
    -e MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-true}" \
    "${X11_ARGS[@]}" \
    -v "$(pwd)":/mapping \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    "${EXTRA_MOUNTS[@]}" \
    "$IMG"
