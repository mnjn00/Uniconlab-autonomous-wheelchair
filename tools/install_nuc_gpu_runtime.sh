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

CUDA_MAJOR="$(nvidia-smi | sed -n 's/.*CUDA Version: \([0-9][0-9]*\)\..*/\1/p' | head -1)"
case "$CUDA_MAJOR" in
  11) CUPY_PACKAGE="cupy-cuda11x" ;;
  12) CUPY_PACKAGE="cupy-cuda12x" ;;
  *) fail "unsupported or unreadable driver CUDA major: ${CUDA_MAJOR:-unknown}; install CUDA 11/12 compatible driver" ;;
esac

say "installing the matching CuPy wheel"
python3 -m pip --version >/dev/null 2>&1 || \
  fail "python3-pip is missing: sudo apt-get install python3-pip"
python3 -m pip install --user --upgrade "$CUPY_PACKAGE"

say "running a real CUDA allocation and reduction"
python3 - <<'PY'
import cupy as cp
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
print("  CuPy device:", name)
print("  CUDA runtime:", cp.cuda.runtime.runtimeGetVersion())
print("  probe: OK")
PY

echo ""
echo "GPU runtime is ready. Hybrid start defaults to REQUIRE_GPU=true."
