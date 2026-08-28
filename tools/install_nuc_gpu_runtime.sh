#!/usr/bin/env bash
# Install and verify the CuPy runtime used by gpu_dwa_follower.py on the NUC.
set -euo pipefail

say() { printf '\n=== %s ===\n' "$1"; }
fail() { echo "ERROR: $*" >&2; exit 1; }

command -v nvidia-smi >/dev/null 2>&1 || fail "nvidia-smi is missing; install the NVIDIA driver first"
command -v python3 >/dev/null 2>&1 || fail "python3 is missing"

say "checking the installed NVIDIA GPU"
GPU_NAME="$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1 | sed 's/^ *//;s/ *$//')"
[ -n "$GPU_NAME" ] || fail "no NVIDIA GPU was reported"
printf '  GPU: %s\n' "$GPU_NAME"
case "$GPU_NAME" in
  *RTX\ 2060*|*GeForce\ RTX\ 2060*) ;;
  *) echo "WARNING: expected RTX 2060, found: $GPU_NAME" >&2 ;;
esac

# Prefer the installed toolkit over the driver's maximum supported version.
# nvidia-smi can report CUDA 13 while /usr/local/cuda is still 12.x.
if command -v nvcc >/dev/null 2>&1; then
  CUDA_MAJOR="$(nvcc --version | sed -n 's/.*release \([0-9][0-9]*\)\..*/\1/p' | tail -1)"
  CUDA_SOURCE="nvcc toolkit"
else
  CUDA_MAJOR="$(nvidia-smi | sed -n 's/.*CUDA Version: \([0-9][0-9]*\)\..*/\1/p' | head -1)"
  CUDA_SOURCE="driver capability (nvcc not installed)"
fi
case "$CUDA_MAJOR" in
  11) CUPY_PACKAGE="cupy-cuda11x" ;;
  12) CUPY_PACKAGE="cupy-cuda12x" ;;
  13) CUPY_PACKAGE="cupy-cuda13x" ;;
  *) fail "unsupported or unreadable CUDA major: ${CUDA_MAJOR:-unknown}; install a CUDA 11/12 toolkit" ;;
esac
printf '  CUDA selection: %s.x from %s\n' "$CUDA_MAJOR" "$CUDA_SOURCE"

PYTHON_MAJOR="$(python3 -c 'import sys; print(sys.version_info.major)')"
PYTHON_MINOR="$(python3 -c 'import sys; print(sys.version_info.minor)')"
[ "$PYTHON_MAJOR" = "3" ] || fail "Python 3 is required"
case "$PYTHON_MINOR" in
  8)
    # ROS Noetic on Ubuntu 20.04 uses Python 3.8. CuPy 13+ requires Python
    # 3.9, while 12.3.0 still publishes CUDA 11/12 Python-3.8 wheels.
    CUPY_VERSION="12.3.0"
    [ "$CUDA_MAJOR" != "13" ] || \
      fail "Python 3.8 has no supported CUDA-13 CuPy wheel; install/use a CUDA 12 toolkit"
    ;;
  9|10|11|12|13)
    # Last CuPy 13 release: stable on Ubuntu 20.04 and supports CUDA 11-13.
    CUPY_VERSION="13.6.0"
    ;;
  *)
    fail "unsupported Python 3.$PYTHON_MINOR; use the ROS Noetic Python 3.8 environment"
    ;;
esac
printf '  Python: %s.%s; CuPy target: %s==%s\n' \
  "$PYTHON_MAJOR" "$PYTHON_MINOR" "$CUPY_PACKAGE" "$CUPY_VERSION"

say "installing the pinned CuPy wheel"
python3 -m pip --version >/dev/null 2>&1 || \
  fail "python3-pip is missing: sudo apt-get install python3-pip"
# Multiple CuPy wheel variants in one interpreter are unsupported and can
# import the wrong shared libraries. Remove only CuPy variants, not NumPy.
python3 -m pip uninstall -y \
  cupy cupy-cuda11x cupy-cuda12x cupy-cuda13x >/dev/null 2>&1 || true
python3 -m pip install --user --no-cache-dir \
  "${CUPY_PACKAGE}==${CUPY_VERSION}"

say "running a real CUDA allocation and reduction"
EXPECTED_CUPY="$CUPY_VERSION" python3 - <<'PY'
import os
import cupy as cp

expected_version = os.environ["EXPECTED_CUPY"]
if cp.__version__ != expected_version:
    raise SystemExit(
        "wrong CuPy version imported: %s (expected %s)" %
        (cp.__version__, expected_version))
name = cp.cuda.runtime.getDeviceProperties(0)["name"]
if isinstance(name, bytes):
    name = name.decode("utf-8", "replace")
x = cp.arange(1_000_000, dtype=cp.float32)
value = float((x * 2.0).sum())
expected = 999_999.0 * 1_000_000.0
relative = abs(value - expected) / expected
if relative > 2e-6:
    raise SystemExit("CUDA arithmetic probe mismatch: %r" % value)
cp.cuda.Device().synchronize()
print("  CuPy:", cp.__version__)
print("  CuPy device:", name)
print("  CUDA runtime:", cp.cuda.runtime.runtimeGetVersion())
print("  probe: OK")
PY

echo ""
echo "GPU runtime is ready. Hybrid start defaults to REQUIRE_GPU=true."
