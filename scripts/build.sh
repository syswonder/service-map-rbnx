#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx build phase — runs `rbnx codegen` then `docker build`
# against the light Dockerfile. Same shape as system/scene/scripts/build.sh.
#
# RBNX_BUILD_CLEAN=1 nukes rbnx-build/ and rebuilds without docker cache.
# RBNX_BUILD_VARIANT=fastlio2_full picks the heavy Dockerfile (FASTLIO2
# colcon build) instead of the default light one.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
CLEAN="${RBNX_BUILD_CLEAN:-}"
VARIANT="${RBNX_BUILD_VARIANT:-light}"
IMG="${ROBONIX_MAPPING_IMAGE:-robonix-mapping}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. Codegen (proto stubs for atlas + IDL types) ──────────────────────────
if command -v rbnx >/dev/null 2>&1; then
    FLAGS=()
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[build] rbnx codegen ${FLAGS[*]}"
    rbnx codegen -p "$PKG" "${FLAGS[@]}"
else
    echo "[build] WARNING: rbnx not in PATH — skipping proto codegen"
    echo "[build]   install robonix-cli + run \`rbnx setup\` once from the robonix source root"
fi

# ── 2. Docker image ─────────────────────────────────────────────────────────
if ! command -v docker >/dev/null 2>&1; then
    echo "[build] error: docker not found on PATH" >&2
    exit 1
fi

DOCKER_BUILD_FLAGS=()
[[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)

case "$VARIANT" in
    light)         DF=docker/Dockerfile ;;
    fastlio2_full) DF=docker/Dockerfile.fastlio2_full ;;
    *) echo "[build] unknown RBNX_BUILD_VARIANT: $VARIANT (light|fastlio2_full)" >&2; exit 2 ;;
esac

# `docker build` is idempotent so this is a soft optimisation.
if [[ "$CLEAN" != "1" ]] && docker image inspect "$IMG" >/dev/null 2>&1; then
    echo "[build] image $IMG present; rebuilding incrementally"
fi

echo "[build] docker build -f $DF -t $IMG (variant=$VARIANT)"
docker build "${DOCKER_BUILD_FLAGS[@]}" -f "$DF" -t "$IMG" docker/

echo "[build] done."
