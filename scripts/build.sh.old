#!/usr/bin/env bash
# build.sh — Build the mapping_rbnx package.
# Called by `rbnx build -p .`.
#
# Default: Docker build (auto-detect Jetson vs x86).
# Override with RBNX_BUILD_MODE=native for host colcon build (requires ROS2 Humble).
#
# Prereq: run `rbnx setup` once from the robonix source root so `rbnx codegen`
# can resolve contracts/IDL paths.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(dirname "$SCRIPT_DIR")}"
BUILD_DIR="$PKG_ROOT/rbnx-build"
BUILD_MODE="${RBNX_BUILD_MODE:-docker}"

echo "=== mapping_rbnx build (mode: $BUILD_MODE) ==="

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
    rm -rf "$BUILD_DIR" "$PKG_ROOT/proto_gen"
fi

# ── 1. Init FASTLIO2_ROS2 submodule ──────────────────────────────────────────
FASTLIO2_DIR="$PKG_ROOT/third_party/FASTLIO2_ROS2"
if [ ! -f "$FASTLIO2_DIR/fastlio2/CMakeLists.txt" ]; then
    echo "[build] initializing FASTLIO2_ROS2 submodule..."
    (cd "$PKG_ROOT" && git submodule update --init --recursive third_party/FASTLIO2_ROS2)
fi

# ── 2. Python deps ───────────────────────────────────────────────────────────
pip install --quiet --no-cache-dir \
    grpcio>=1.60.0 grpcio-tools>=1.60.0 protobuf>=4.25.0 numpy>=1.24.0 pyyaml>=6.0 \
    2>/dev/null || echo "[build] pip install skipped (not in venv or already satisfied)"

# ── 3. Codegen — one call replaces ~40 lines of manual robonix-codegen + protoc ──
if command -v rbnx >/dev/null 2>&1; then
    FLAGS=()
    [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
    rbnx codegen -p "$PKG_ROOT" "${FLAGS[@]}"
else
    echo "[build] WARNING: rbnx not in PATH — skipping proto codegen"
    echo "[build]   install robonix-cli + run \`rbnx setup\` once from the robonix source root"
fi

# ── 4. Build SLAM packages ──────────────────────────────────────────────────
if [[ "$BUILD_MODE" == "native" ]]; then
    echo "[build] Native colcon build (requires ROS2 Humble on host)..."
    WS_ROOT="${ROS2_WS:-$(cd "$PKG_ROOT/.." 2>/dev/null && pwd)}"
    WS_SRC="$WS_ROOT/src"
    mkdir -p "$WS_SRC"

    for pkg in interface fastlio2 pgo localizer hba; do
        src="$FASTLIO2_DIR/$pkg"
        dst="$WS_SRC/$pkg"
        if [ ! -e "$dst" ] && [ -d "$src" ]; then
            ln -sf "$src" "$dst"
            echo "[build] symlinked $pkg → $dst"
        fi
    done

    cd "$WS_ROOT"
    if [ -f /opt/ros/humble/setup.bash ]; then
        # shellcheck source=/dev/null
        source /opt/ros/humble/setup.bash
    fi
    colcon build --symlink-install \
        --packages-select interface fastlio2 pgo localizer hba \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo "[build] Docker build..."
    cd "$PKG_ROOT"
    if [ -f /etc/nv_tegra_release ] 2>/dev/null; then
        echo "[build] Jetson detected — Dockerfile.jetson"
        docker build -f docker/Dockerfile.jetson -t mapping_rbnx:jetson .
    else
        echo "[build] x86 — Dockerfile"
        docker build -f docker/Dockerfile -t mapping_rbnx:latest .
    fi
fi

# ── 5. Stamp ─────────────────────────────────────────────────────────────────
mkdir -p "$BUILD_DIR"
date -Iseconds > "$BUILD_DIR/.rbnx-built"
echo "=== mapping_rbnx build complete ==="
