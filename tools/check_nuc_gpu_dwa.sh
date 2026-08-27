#!/usr/bin/env bash
# Verify the CuPy/RTX backend used by gpu_dwa_follower.py. With no ROS master,
# this checks a real CUDA allocation and reduction. With the hybrid graph up,
# it additionally requires the running follower to report the CuPy backend.
set -euo pipefail

WAIT_S="${1:-0}"
case "$WAIT_S" in *[!0-9.]*|'') echo "usage: check_nuc_gpu_dwa.sh [wait-seconds]" >&2; exit 64 ;; esac
REQUIRE_RTX2060="${REQUIRE_RTX2060:-true}"
LOCALIZATION_WS="${LOCALIZATION_WS:-$HOME/livox_static_localization_ws}"

fail() { echo "GPU_DWA_CHECK_FAILED: $*" >&2; exit 1; }
case "$REQUIRE_RTX2060" in true|false) ;; *) fail "REQUIRE_RTX2060 must be true or false" ;; esac
command -v nvidia-smi >/dev/null 2>&1 || fail "nvidia-smi is unavailable"
GPU_NAME="$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1 | xargs)"
[ -n "$GPU_NAME" ] || fail "no NVIDIA GPU was reported"
if [ "$REQUIRE_RTX2060" = "true" ] && [[ "$GPU_NAME" != *"RTX 2060"* ]]; then
  fail "expected RTX 2060, found '$GPU_NAME'"
fi

python3 - "$REQUIRE_RTX2060" <<'PY'
import sys
try:
    import cupy as cp
except Exception as error:
    raise SystemExit("CuPy import failed: %s" % error)
properties = cp.cuda.runtime.getDeviceProperties(0)
name = properties["name"]
if isinstance(name, bytes):
    name = name.decode("utf-8", "replace")
if sys.argv[1] == "true" and "RTX 2060" not in name:
    raise SystemExit("expected RTX 2060, found %r" % name)
x = cp.arange(1_000_000, dtype=cp.float32)
value = float(cp.sum(x * 2.0).get())
expected = 999_999.0 * 1_000_000.0
if abs(value - expected) / expected > 2e-6:
    raise SystemExit("CUDA arithmetic probe mismatch: %r" % value)
cp.cuda.Device().synchronize()
print("GPU_DWA_CUDA_OK")
print("  gpu          : %s" % name)
print("  cuda_runtime : %s" % cp.cuda.runtime.runtimeGetVersion())
print("  free_mb      : %.1f" % (cp.cuda.runtime.memGetInfo()[0] / 1048576.0))
PY

# Artifact-only checks are useful immediately after setup, before roscore.
if ! pgrep -f '[r]osmaster' >/dev/null 2>&1; then
  echo "  ROS master is not running; CuPy CUDA check passed"
  exit 0
fi

set +u
source /opt/ros/noetic/setup.bash
[ ! -f "$LOCALIZATION_WS/devel/setup.bash" ] || \
  source "$LOCALIZATION_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

python3 - "$WAIT_S" <<'PY'
import subprocess
import sys
import time

wait_s = float(sys.argv[1])
deadline = time.monotonic() + max(0.1, wait_s)
last = "follower parameters unavailable"
while time.monotonic() <= deadline:
    node = subprocess.run(
        ["rosnode", "ping", "-c1", "/waypoint_follower"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if node.returncode == 0:
        active = subprocess.run(
            ["rosparam", "get", "/waypoint_follower/gpu_active"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        backend = subprocess.run(
            ["rosparam", "get", "/waypoint_follower/distance_backend"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        active_text = active.stdout.strip().lower()
        backend_text = backend.stdout.strip().strip("'\"").lower()
        if active.returncode == 0 and backend.returncode == 0 and \
                active_text == "true" and backend_text == "cupy":
            print("GPU_DWA_RUNTIME_OK")
            print("  backend : cupy")
            print("  node    : /waypoint_follower")
            raise SystemExit(0)
        last = "gpu_active=%r backend=%r" % (active_text, backend_text)
    time.sleep(0.25)
print("GPU_DWA_NOT_READY: %s" % last, file=sys.stderr)
raise SystemExit(1)
PY

nvidia-smi --query-compute-apps=pid,process_name,used_memory \
  --format=csv,noheader 2>/dev/null || true
