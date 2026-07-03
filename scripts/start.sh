#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx start phase. Two execution shapes (same pattern as
# system/scene/scripts/start.sh):
#
#   1. native  (jetson_orin, or any host with ROS 2 Humble + rtabmap_*
#       installed natively) — runs scripts/start_native.sh: atlas_bridge
#       + rtabmap launch as host processes, no docker. Preferred on the
#       car (avoids the nvidia-container-runtime + DDS-namespace hops).
#   2. docker  (default fallback) — docker run against `robonix-mapping`.
#
# Selection (operator-set env in the shell that runs rbnx boot/start;
# the cap config arrives via Driver(CMD_INIT) so it can't be read here):
#   ROBONIX_MAPPING_FORCE=native|docker     # explicit hard pin
#   ROBONIX_MAPPING_PLATFORM=<platform>     # match NATIVE_PLATFORMS
#   default → docker
set -eo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

NATIVE_PLATFORMS=("jetson_orin")
is_native_platform() {
    local p="$1"
    for w in "${NATIVE_PLATFORMS[@]}"; do
        [[ "$p" == "$w" ]] && return 0
    done
    return 1
}

MODE=""
case "${ROBONIX_MAPPING_FORCE:-}" in
    native) MODE=native ;;
    docker) MODE=docker ;;
    "") ;;
    *) echo "[mapping/start] ROBONIX_MAPPING_FORCE=${ROBONIX_MAPPING_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
if [[ -z "$MODE" ]]; then
    if is_native_platform "${ROBONIX_MAPPING_PLATFORM:-}"; then MODE=native; else MODE=docker; fi
fi
echo "[mapping/start] mode=${MODE} (FORCE=${ROBONIX_MAPPING_FORCE:-} PLATFORM=${ROBONIX_MAPPING_PLATFORM:-})"

if [[ "$MODE" == "native" ]]; then
    exec bash "${PKG}/scripts/start_native.sh"
fi

# ── Docker path (original behaviour) ───────────────────────────────────
set -u

CT="${ROBONIX_MAPPING_CONTAINER:-robonix_mapping}"
IMG="${ROBONIX_MAPPING_IMAGE:-robonix-mapping}"

cleanup() {
    docker stop "$CT" >/dev/null 2>&1 || true
    kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

docker rm -f "$CT" >/dev/null 2>&1 || true
mkdir -p rbnx-build/data

declare -a ZENOH_ARGS=()
if [[ -n "${ROBONIX_ZENOH_ROUTER:-}" ]]; then
    ZENOH_ARGS=(-e "ROBONIX_ZENOH_ROUTER=${ROBONIX_ZENOH_ROUTER}")
fi
if [[ -n "${ROBONIX_ZENOH_MODE:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_MODE=${ROBONIX_ZENOH_MODE}")
fi
if [[ -n "${ROBONIX_ZENOH_LISTEN:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_LISTEN=${ROBONIX_ZENOH_LISTEN}")
fi

declare -a EXTRA_MOUNTS=()
# NOTE: RBNX_CONFIG_FILE intentionally NOT mounted — config arrives via
# Driver(CMD_INIT, config_json) over gRPC, never a file.

# X11 for rtabmap_viz inside the container (auto-detect DISPLAY).
if [[ -z "${DISPLAY:-}" ]]; then
    if command -v xset &>/dev/null; then
        for d in :0 :1 :10; do
            if DISPLAY="$d" xset q &>/dev/null; then export DISPLAY="$d"; break; fi
        done
    fi
fi
declare -a X11_ARGS=()
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
    xhost +local:docker >/dev/null 2>&1 || true
    X11_ARGS=(-e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

exec docker run --rm \
    --name "$CT" \
    --network host \
    --ipc=host \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-mapping}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    "${ZENOH_ARGS[@]}" \
    -e MAPPING_GRPC_PORT="${MAPPING_GRPC_PORT:-50120}" \
    -e MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-true}" \
    "${X11_ARGS[@]}" \
    -v "$(pwd)":/mapping \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    "${EXTRA_MOUNTS[@]}" \
    "$IMG"
