#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODE="${1:-host}"

usage() {
  cat <<'EOF'
Usage:
  bash tools/test_static_threat_bypass.sh host
  bash tools/test_static_threat_bypass.sh qa
  bash tools/test_static_threat_bypass.sh live-check

host runs the complete static-threat bypass chain test suite, compiles its
Python surfaces, and checks that the generic status command is advertised. qa
runs the deterministic policy/proposal/gate lifecycle driver. Neither connects
to ROS or starts a wheel-control process.

live-check reads the already-running ROS graph through the fail-closed
preflight. It does not start, stop, replace, or command any node.
EOF
}

case "$MODE" in
  -h|--help|help)
    usage
    exit 0
    ;;
  host)
    [ "$#" -eq 0 ] || [ "$#" -eq 1 ] || {
      echo "ERROR: host takes no additional arguments" >&2
      exit 64
    }
    CACHE_ROOT="$(mktemp -d)"
    trap 'rm -rf "$CACHE_ROOT"' EXIT HUP INT TERM
    export PYTHONDONTWRITEBYTECODE=1
    export PYTHONPYCACHEPREFIX="$CACHE_ROOT/pycache"
    cd "$REPO_ROOT"
    printf 'Testing checkout: %s\n' "$REPO_ROOT"
    printf 'Testing commit:   %s\n' "$(git rev-parse HEAD)"
    python3 -m pytest -p no:cacheprovider -q \
      src/static_livox_localization/test/test_cluster_guard.py \
      src/static_livox_localization/test/test_person_bypass_policy.py \
      src/static_livox_localization/test/test_dwa_policy.py \
      src/static_livox_localization/test/test_gpu_dwa_backend.py \
      src/static_livox_localization/test/test_static_threat_follower.py \
      src/static_livox_localization/test/test_static_threat_semantic.py \
      src/static_livox_localization/test/test_static_threat_trajectory_gate.py \
      src/static_livox_localization/test/test_trajectory_gate_rear_clearance.py \
      src/static_livox_localization/test/test_waypoint_follower_surface.py \
      src/wheelchair_safety/tests/test_safety_gate.py \
      tests/test_dwa_band.py \
      tests/test_person_bypass_runtime_surface.py \
      tests/test_python_node_packaging.py \
      tests/test_static_threat_host_qa.py
    python3 -m compileall -q \
      src/static_livox_localization/scripts \
      src/static_livox_localization/test \
      tests \
      tools/static_threat_bypass_host_qa.py
    STATUS_HELP="$(bash "$SCRIPT_DIR/hybrid.sh" help)"
    case "$STATUS_HELP" in
      *"static-threat-bypass-status"*) ;;
      *) echo "ERROR: generic static-threat status command missing" >&2; exit 1 ;;
    esac
    printf 'STATIC_THREAT_HOST_TEST_PASS\n'
    ;;
  qa)
    [ "$#" -eq 1 ] || {
      echo "ERROR: qa takes no additional arguments" >&2
      exit 64
    }
    export PYTHONDONTWRITEBYTECODE=1
    cd "$REPO_ROOT"
    python3 tools/static_threat_bypass_host_qa.py
    printf 'STATIC_THREAT_HOST_QA_PASS\n'
    ;;
  live-check)
    [ "$#" -eq 1 ] || {
      echo "ERROR: live-check takes no additional arguments" >&2
      exit 64
    }
    exec bash "$SCRIPT_DIR/hybrid.sh" static-threat-bypass-status
    ;;
  *)
    echo "ERROR: unknown mode: $MODE" >&2
    usage >&2
    exit 64
    ;;
esac
