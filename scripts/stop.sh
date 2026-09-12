#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

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
    *) echo "[mapping/stop] ROBONIX_MAPPING_FORCE=${ROBONIX_MAPPING_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
if [[ -z "$MODE" ]]; then
    if is_native_platform "${ROBONIX_MAPPING_PLATFORM:-}"; then MODE=native; else MODE=docker; fi
fi

echo "[mapping/stop] mode=${MODE}"
if [[ "$MODE" == "docker" ]]; then
    docker rm -f "${ROBONIX_MAPPING_CONTAINER:-robonix_mapping}" >/dev/null 2>&1 || true
    exit 0
fi

ENGINE_PID_FILE="${MAPPING_ENGINE_PID_FILE:-/tmp/mapping_engine_pid}"

process_group_alive() {
    local pgid="$1"
    ps -eo pgid= | awk -v wanted="$pgid" '$1 == wanted { found=1 } END { exit !found }'
}

terminate_recorded_engine_group() {
    local pgid="" members=""
    pgid="$(cat "$ENGINE_PID_FILE" 2>/dev/null || true)"
    if [[ ! "$pgid" =~ ^[0-9]+$ ]]; then
        rm -f "$ENGINE_PID_FILE"
        return 0
    fi
    members="$(ps -eo pgid=,args= | awk -v wanted="$pgid" '$1 == wanted { $1=""; sub(/^ +/, ""); print }')"
    if [[ -z "$members" ]]; then
        rm -f "$ENGINE_PID_FILE"
        return 0
    fi
    if ! grep -Eq 'slam_toolbox|tf_to_pose\.py|scan_to_map_outputs\.py|ros2 launch' <<<"$members"; then
        echo "[mapping/stop] refusing to signal reused/foreign process group $pgid" >&2
        return 0
    fi
    echo "[mapping/stop] terminating slam_toolbox process group $pgid"
    kill -TERM -- "-$pgid" 2>/dev/null || true
    for _ in $(seq 1 25); do
        process_group_alive "$pgid" || break
        sleep 0.2
    done
    if process_group_alive "$pgid"; then
        echo "[mapping/stop] force-killing slam_toolbox process group $pgid"
        kill -KILL -- "-$pgid" 2>/dev/null || true
    fi
    rm -f "$ENGINE_PID_FILE" /tmp/mapping_engine_request /tmp/mapping_engine_args
}

# Stop the supervisor first so it cannot restart the engine while the detached
# launch group is being reaped. The launch is deliberately in its own session
# for runtime engine swaps, so killing only the supervisor is not sufficient.
pkill -TERM -f "${PKG}/scripts/start_engine.sh" 2>/dev/null || true
sleep 0.2
terminate_recorded_engine_group
pkill -TERM -f "${PKG}.*mapping_rbnx.atlas_bridge" 2>/dev/null || true
sleep 1
pkill -KILL -f "${PKG}.*mapping_rbnx.atlas_bridge" 2>/dev/null || true
pkill -KILL -f "${PKG}/scripts/start_engine.sh" 2>/dev/null || true
