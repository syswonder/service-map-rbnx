#!/usr/bin/env bash
# Isolated replay only. Mount existing sensor data and generated ROS interfaces.
set -eo pipefail
task_root="$(cd "$(dirname "$0")/../.." && pwd)"
: "LAB_DIR:?Set LAB_DIR to the prepared map-maintenance lab (deps/ and data/)}"
: "RESULTS_DIR:?Set RESULTS_DIR to a new results directory}"
mkdir -p "$RESULTS_DIR"
docker run --rm --network none --entrypoint bash \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-119}" \
  -e 'CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>' \
  -v "$task_root:/work" -v "$LAB_DIR:/lab:ro" -v "$RESULTS_DIR:/results" \
  "${LAB_IMAGE:-robonix-map-maintenance-lab:20261026}" -lc '
    source /opt/ros/humble/setup.bash
    source /lab/deps/codegen/ros2_idl/install/setup.bash
    export PYTHONPATH=/work/src:/lab/deps:/lab/deps/codegen/proto_gen:/lab/deps/codegen/mapping_proto_gen:/lab/deps/codegen/robonix_mcp_types:$PYTHONPATH
    exec /usr/bin/python3 "/work/experiments/unified_mapping/$1" "${@:2}"
  ' -- "$@"
