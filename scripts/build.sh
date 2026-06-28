#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx build phase — runs `rbnx codegen`, then builds for the
# selected deployment target.
#
# Target is chosen by the per-target package manifest's `build:` line
# (see package_manifest*.yaml). Add a target by adding a case branch
# below plus its Dockerfile / native step — nothing else changes.
#   x86-docker     x86_64 + docker, ROS2 in image (docker/Dockerfile)   [default]
#   jetson-docker  arm64 Jetson + docker, L4T base (docker/Dockerfile.jetson)
#   jetson-native  arm64 Jetson + host ROS2 — no docker; ensure
#                  ros-humble-rtabmap-ros is apt-installed on the host.
#
# RBNX_BUILD_CLEAN=1     nuke rbnx-build/ and rebuild without docker cache.
# RBNX_BUILD_VARIANT=fastlio2_full  (x86-docker only) heavy FASTLIO2 image.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
CLEAN="${RBNX_BUILD_CLEAN:-}"
VARIANT="${RBNX_BUILD_VARIANT:-light}"
IMG="${ROBONIX_MAPPING_IMAGE:-robonix-mapping}"
TARGET="${RBNX_BUILD_TARGET:-x86-docker}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. Codegen (proto stubs for atlas + IDL types + MCP types) — every target ─
# --mcp is REQUIRED: atlas_bridge.py imports `map_mcp` (SaveMap/LoadMap/
# PoseEstimate/SwitchMode request/response dataclasses for the MCP tools).
# Without it codegen emits only proto stubs, `map_mcp` is missing, and the
# bridge dies at import with ModuleNotFoundError → the service never registers.
if command -v rbnx >/dev/null 2>&1; then
    FLAGS=(--mcp)
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[build] rbnx codegen ${FLAGS[*]}"
    rbnx codegen -p "$PKG" "${FLAGS[@]}"
else
    echo "[build] WARNING: rbnx not in PATH — skipping proto codegen"
    echo "[build]   install robonix-cli + run \`rbnx setup\` once from the robonix source root"
fi

echo "[build] target=$TARGET"

# ── 2. Per-target build ─────────────────────────────────────────────────────
case "$TARGET" in
    x86-docker|jetson-docker)
        if ! command -v docker >/dev/null 2>&1; then
            echo "[build] error: target $TARGET needs docker on PATH" >&2
            exit 1
        fi
        DOCKER_BUILD_FLAGS=(--network=host)
        [[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)
        if [[ "$TARGET" == "jetson-docker" ]]; then
            DF=docker/Dockerfile.jetson
        else
            case "$VARIANT" in
                light)         DF=docker/Dockerfile ;;
                fastlio2_full) DF=docker/Dockerfile.fastlio2_full ;;
                *) echo "[build] unknown RBNX_BUILD_VARIANT: $VARIANT (light|fastlio2_full)" >&2; exit 2 ;;
            esac
        fi
        if [[ "$CLEAN" != "1" ]] && docker image inspect "$IMG" >/dev/null 2>&1; then
            echo "[build] image $IMG present; rebuilding incrementally"
        fi
        echo "[build] docker build -f $DF -t $IMG"
        docker build "${DOCKER_BUILD_FLAGS[@]}" -f "$DF" -t "$IMG" docker/
        ;;

    jetson-native)
        # No docker: rtabmap runs as a host process (start_native.sh). The
        # only build-time requirement is that the host has ROS2 Humble +
        # rtabmap. We don't apt-install for the operator (needs sudo and a
        # specific ROS apt setup) — we verify and tell them what's missing.
        echo "[build] native target — verifying host ROS2 + rtabmap"
        missing=0
        if ! command -v ros2 >/dev/null 2>&1; then
            echo "[build] ERROR: ros2 not on PATH — source /opt/ros/humble/setup.bash" >&2
            missing=1
        fi
        if ! ros2 pkg prefix rtabmap_slam >/dev/null 2>&1; then
            echo "[build] ERROR: rtabmap not installed. On the Jetson host run:" >&2
            echo "[build]   sudo apt install ros-humble-rtabmap-ros" >&2
            missing=1
        fi
        [[ "$missing" == "1" ]] && exit 1
        echo "[build] host rtabmap OK ($(ros2 pkg prefix rtabmap_slam))"
        ;;

    *)
        echo "[build] unknown RBNX_BUILD_TARGET: $TARGET" >&2
        echo "[build]   supported: x86-docker | jetson-docker | jetson-native" >&2
        exit 2
        ;;
esac

echo "[build] done (target=$TARGET)."
