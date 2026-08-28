#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODE="${1:-host}"

usage() {
  cat <<'EOF'
Usage:
  bash tools/test_static_threat_bypass.sh host
  bash tools/test_static_threat_bypass.sh live-check

host runs the isolated policy, follower, CPU/GPU planner, and runtime-surface
tests. It does not connect to ROS or start a wheel-control process.

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
      src/static_livox_localization/test/test_person_bypass_policy.py \
      src/static_livox_localization/test/test_dwa_policy.py \
      src/static_livox_localization/test/test_gpu_dwa_backend.py \
      tests/test_dwa_band.py \
      tests/test_person_bypass_runtime_surface.py \
      tests/test_python_node_packaging.py
    python3 -m compileall -q \
      src/static_livox_localization/scripts/dwa_core.py \
      src/static_livox_localization/scripts/dwa_follower.py \
      src/static_livox_localization/scripts/gpu_dwa_backend.py \
      src/static_livox_localization/scripts/person_bypass_policy.py \
      src/static_livox_localization/scripts/person_bypass_dwa_follower.py \
      src/static_livox_localization/scripts/person_bypass_preflight.py \
      src/static_livox_localization/scripts/safety_gate.py
    printf 'STATIC_THREAT_HOST_TEST_PASS\n'
    ;;
  live-check)
    [ "$#" -eq 1 ] || {
      echo "ERROR: live-check takes no additional arguments" >&2
      exit 64
    }
    exec bash "$SCRIPT_DIR/hybrid.sh" person-bypass-status
    ;;
  *)
    echo "ERROR: unknown mode: $MODE" >&2
    usage >&2
    exit 64
    ;;
esac
