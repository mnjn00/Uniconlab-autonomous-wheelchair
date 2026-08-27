#!/usr/bin/env bash
# Canonical field entry point for the hybrid profile.
#
# ROS Noetic setup scripts read variables before defining them and can abort
# under `set -u`. The underlying reviewed scripts intentionally remain
# ordinary bash files; this launcher executes a temporary same-directory copy
# with nounset relaxed, preserving their SCRIPT_DIR/repository resolution.
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

Environment is passed through unchanged. Important variables:
  REQUIRE_LEARNED=false|true
  START_POINTPILLARS=true|false
  POINTPILLARS_MODEL=/path/to/pointpillar.plan
  LEARNED_VISION_TOPIC=/pointpillars/detections
  CLIFF_REQUIRED=false|true
EOF
}

run_without_nounset() {
  local source="$1"
  shift
  [ -f "$source" ] || {
    echo "ERROR: missing hybrid script: $source" >&2
    exit 66
  }
  local temporary
  temporary="$(mktemp "$SCRIPT_DIR/.hybrid-runtime.XXXXXX.sh")"
  trap 'rm -f "$temporary"' EXIT HUP INT TERM
  sed 's/^set -euo pipefail$/set -eo pipefail/' "$source" > "$temporary"
  bash "$temporary" "$@"
}

case "$COMMAND" in
  setup-gpu)
    run_without_nounset "$SCRIPT_DIR/setup_rtx2060_pointpillars.sh" "$@"
    ;;
  start)
    run_without_nounset "$SCRIPT_DIR/start_hybrid_avoidance.sh" "$@"
    ;;
  go)
    run_without_nounset "$SCRIPT_DIR/go_hybrid.sh" "$@"
    ;;
  gpu-status)
    run_without_nounset "$SCRIPT_DIR/check_rtx2060_pointpillars.sh" "$@"
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
