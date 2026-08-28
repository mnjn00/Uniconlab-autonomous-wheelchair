#!/usr/bin/env bash
# Canonical field entry point for the RTX-accelerated ROS1 hybrid profile.
set -eo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
COMMAND="${1:-}"
[ "$#" -gt 0 ] && shift || true

usage() {
  cat <<'EOF'
Usage:
  bash tools/hybrid.sh setup-gpu
  bash tools/hybrid.sh start [start_hybrid_avoidance.sh options]
  bash tools/hybrid.sh go    [go_hybrid.sh options]
  bash tools/hybrid.sh stop
  bash tools/hybrid.sh gpu-status
  bash tools/hybrid.sh person-bypass-status

`setup-gpu` installs/verifies both RTX paths:
  1. CuPy nearest-neighbour acceleration for DWA
  2. NVIDIA CUDA-PointPillars + FP16 TensorRT object detection

`start` first brings up the ordinary paused hybrid graph, then replaces the
stop-only person policy and fixed-corridor raw gate with the reviewed
stationary-threat trajectory-bypass nodes. Moving/unknown threats still stop.

Important environment variables:
  START_POINTPILLARS=true|false
  REQUIRE_LEARNED=true|false
  REQUIRE_GPU=true|false
  POINTPILLARS_MODEL=/path/to/pointpillar.plan
  POINTPILLARS_REQUIRE_RTX2060=true|false
  CLIFF_REQUIRED=false|true
  PERSON_BYPASS_CONFIRM_S=3.0
  PERSON_BYPASS_MAX_GAP_S=0.45
  PERSON_BYPASS_LATERAL_HYSTERESIS_M=0.25
  PERSON_BYPASS_SPEED_MPS=0.35
  PERSON_BYPASS_CLEARANCE_M=0.80
EOF
}

run_without_nounset() {
  local source="$1"
  shift
  [ -f "$source" ] || {
    echo "ERROR: missing hybrid script: $source" >&2
    return 66
  }
  local temporary status
  temporary="$(mktemp "$SCRIPT_DIR/.hybrid-runtime.XXXXXX.sh")"
  sed 's/^set -euo pipefail$/set -eo pipefail/' "$source" > "$temporary"
  if bash "$temporary" "$@"; then
    status=0
  else
    status=$?
  fi
  rm -f "$temporary"
  return "$status"
}

case "$COMMAND" in
  setup-gpu)
    run_without_nounset "$SCRIPT_DIR/install_nuc_gpu_runtime.sh" "$@" &&
      run_without_nounset "$SCRIPT_DIR/setup_rtx2060_pointpillars.sh" "$@"
    ;;
  start)
    run_without_nounset "$SCRIPT_DIR/start_hybrid_avoidance.sh" "$@" &&
      run_without_nounset "$SCRIPT_DIR/activate_person_bypass.sh" activate
    ;;
  go)
    run_without_nounset "$SCRIPT_DIR/go_hybrid.sh" "$@"
    ;;
  person-bypass-status)
    run_without_nounset "$SCRIPT_DIR/activate_person_bypass.sh" --check
    ;;
  gpu-status)
    run_without_nounset "$SCRIPT_DIR/check_nuc_gpu_dwa.sh" "${1:-0}" &&
      run_without_nounset "$SCRIPT_DIR/check_rtx2060_pointpillars.sh" "${1:-0}"
    ;;
  stop)
    STOP="${BASE_STOP:-$HOME/stop.sh}"
    [ -f "$STOP" ] || STOP="$SCRIPT_DIR/stop.sh"
    [ -f "$STOP" ] || {
      echo "ERROR: stop.sh not found" >&2
      exit 66
    }
    bash "$STOP"
    ;;
  -h|--help|help|"")
    usage
    ;;
  *)
    echo "ERROR: unknown hybrid command: $COMMAND" >&2
    usage >&2
    exit 64
    ;;
esac
