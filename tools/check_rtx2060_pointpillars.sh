#!/usr/bin/env bash
# Verify both the installed RTX 2060 PointPillars artifacts and, when ROS is
# running, the live GPU inference status used by hybrid preflight.
set -euo pipefail

WAIT_S="${1:-0}"
case "$WAIT_S" in *[!0-9.]*|'') echo "usage: check_rtx2060_pointpillars.sh [wait-seconds]" >&2; exit 64 ;; esac
ENV_FILE="${POINTPILLARS_ENV:-$HOME/.config/unicon/pointpillars.env}"
LOCALIZATION_WS="${LOCALIZATION_WS:-$HOME/livox_static_localization_ws}"
REQUIRE_RTX2060="${REQUIRE_RTX2060:-true}"
MAX_INFERENCE_MS="${POINTPILLARS_MAX_INFERENCE_MS:-90.0}"
MAX_STATUS_AGE_S="${POINTPILLARS_MAX_STATUS_AGE_S:-1.5}"

fail() { echo "RTX_POINTPILLARS_CHECK_FAILED: $*" >&2; exit 1; }
say() { printf '\n=== %s ===\n' "$1"; }
case "$REQUIRE_RTX2060" in true|false) ;; *) fail "REQUIRE_RTX2060 must be true or false" ;; esac

[ -f "$ENV_FILE" ] || fail "$ENV_FILE is missing; run tools/hybrid.sh setup-gpu"
set +u
# shellcheck disable=SC1090
source "$ENV_FILE"
set -u

say "installed GPU and model"
command -v nvidia-smi >/dev/null 2>&1 || fail "nvidia-smi is unavailable"
GPU_NAME="$(nvidia-smi --id="${POINTPILLARS_GPU_DEVICE:-0}" --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | xargs)"
[ -n "$GPU_NAME" ] || fail "configured GPU is unavailable"
if [ "$REQUIRE_RTX2060" = "true" ] && [[ "$GPU_NAME" != *"RTX 2060"* ]]; then
  fail "expected RTX 2060, found '$GPU_NAME'"
fi
nvidia-smi --id="${POINTPILLARS_GPU_DEVICE:-0}" \
  --query-gpu=name,driver_version,memory.total,memory.used,utilization.gpu,temperature.gpu \
  --format=csv,noheader
[ -f "${CUDA_POINTPILLARS_ROOT:?}/build/libpointpillar_core.so" ] || \
  fail "libpointpillar_core.so is missing"
[ -s "${POINTPILLARS_MODEL:?}" ] || fail "TensorRT engine is missing"
[ "${POINTPILLARS_UPSTREAM_COMMIT:-}" = "ce7e2bd694c90207435c8751d61cdb38d48a9f4c" ] || \
  fail "unexpected CUDA-PointPillars upstream commit"
NODE="$LOCALIZATION_WS/devel/lib/static_livox_localization/rtx_pointpillars_node"
[ -x "$NODE" ] || fail "ROS inference node is not built: $NODE"
echo "  core  : $CUDA_POINTPILLARS_ROOT/build/libpointpillar_core.so"
echo "  engine: $POINTPILLARS_MODEL"
echo "  node  : $NODE"

# Artifact-only validation is useful immediately after setup, before roscore.
if ! pgrep -f '[r]osmaster' >/dev/null 2>&1; then
  echo "  ROS master is not running; artifact check passed"
  exit 0
fi

set +u
source /opt/ros/noetic/setup.bash
source "$LOCALIZATION_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

say "live TensorRT inference"
python3 - "$WAIT_S" "$MAX_INFERENCE_MS" "$MAX_STATUS_AGE_S" "$REQUIRE_RTX2060" <<'PY'
import json
import math
import sys
import time

import rospy
from std_msgs.msg import String

wait_s = float(sys.argv[1])
max_inference_ms = float(sys.argv[2])
max_age_s = float(sys.argv[3])
require_rtx = sys.argv[4] == "true"
deadline = time.monotonic() + max(0.1, wait_s)
rospy.init_node("rtx_pointpillars_check", anonymous=True, disable_signals=True)
last_error = "no status received"
while not rospy.is_shutdown():
    remaining = max(0.1, min(2.0, deadline - time.monotonic()))
    try:
        message = rospy.wait_for_message(
            "/pointpillars/status", String, timeout=remaining)
        data = json.loads(message.data)
        if not isinstance(data, dict):
            raise ValueError("status is not a JSON object")
        status = str(data.get("status", ""))
        age = rospy.Time.now().to_sec() - float(data["stamp"])
        inference_ms = float(data.get("inference_ms", math.inf))
        device = str(data.get("device_name", ""))
        if status != "OK":
            raise ValueError("status=%s (%s)" % (status, data.get("detail", "")))
        if data.get("gpu_active") is not True:
            raise ValueError("gpu_active is not true")
        if require_rtx and "RTX 2060" not in device:
            raise ValueError("unexpected GPU %r" % device)
        if not math.isfinite(age) or age < -0.05 or age > max_age_s:
            raise ValueError("status age %.3f s" % age)
        if not math.isfinite(inference_ms) or inference_ms > max_inference_ms:
            raise ValueError("inference %.3f ms exceeds %.3f ms" %
                             (inference_ms, max_inference_ms))
        print("RTX_POINTPILLARS_OK")
        print("  gpu          : %s" % device)
        print("  inference_ms : %.3f" % inference_ms)
        print("  points       : %s" % data.get("used_points"))
        print("  detections   : %s" % data.get("detections"))
        print("  cuda_free_mb : %s" % data.get("cuda_memory_free_mb"))
        raise SystemExit(0)
    except (rospy.ROSException, KeyError, TypeError, ValueError,
            json.JSONDecodeError) as error:
        last_error = str(error)
    if time.monotonic() >= deadline:
        break
print("RTX_POINTPILLARS_NOT_READY: %s" % last_error, file=sys.stderr)
raise SystemExit(1)
PY

say "NVIDIA compute processes"
nvidia-smi --query-compute-apps=pid,process_name,used_memory \
  --format=csv,noheader 2>/dev/null || true
