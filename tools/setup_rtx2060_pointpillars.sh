#!/usr/bin/env bash
# Build the pinned NVIDIA CUDA-PointPillars core and an FP16 TensorRT engine
# directly on the Phantom Canyon NUC's RTX 2060, then rebuild the ROS1 package
# with the hardware inference node enabled.
set -euo pipefail

UPSTREAM_URL="https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars.git"
UPSTREAM_COMMIT="ce7e2bd694c90207435c8751d61cdb38d48a9f4c"
POINTPILLARS_ROOT="${CUDA_POINTPILLARS_ROOT:-$HOME/opt/CUDA-PointPillars}"
LOCALIZATION_WS="${LOCALIZATION_WS:-$HOME/livox_static_localization_ws}"
CONFIG_DIR="${UNICON_CONFIG_DIR:-$HOME/.config/unicon}"
ENV_FILE="$CONFIG_DIR/pointpillars.env"
REQUIRE_RTX2060="${REQUIRE_RTX2060:-true}"
INSTALL_APT="${INSTALL_APT:-false}"
REBUILD_ENGINE="${REBUILD_ENGINE:-false}"
GPU_DEVICE="${POINTPILLARS_GPU_DEVICE:-0}"

say() { printf '\n=== %s ===\n' "$1"; }
fail() { echo "ERROR: $*" >&2; exit 1; }

case "$REQUIRE_RTX2060" in true|false) ;; *) fail "REQUIRE_RTX2060 must be true or false" ;; esac
case "$INSTALL_APT" in true|false) ;; *) fail "INSTALL_APT must be true or false" ;; esac
case "$REBUILD_ENGINE" in true|false) ;; *) fail "REBUILD_ENGINE must be true or false" ;; esac
case "$GPU_DEVICE" in *[!0-9]*|'') fail "POINTPILLARS_GPU_DEVICE must be a non-negative integer" ;; esac

say "checking the Phantom Canyon GPU"
command -v nvidia-smi >/dev/null 2>&1 || fail "nvidia-smi is unavailable; install a working NVIDIA driver first"
GPU_NAME="$(nvidia-smi --id="$GPU_DEVICE" --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | xargs)"
[ -n "$GPU_NAME" ] || fail "CUDA GPU $GPU_DEVICE is unavailable"
echo "  GPU: $GPU_NAME"
if [ "$REQUIRE_RTX2060" = "true" ] && [[ "$GPU_NAME" != *"RTX 2060"* ]]; then
  fail "expected RTX 2060, found '$GPU_NAME' (set REQUIRE_RTX2060=false only after review)"
fi

say "checking ROS, CUDA, TensorRT and build tools"
missing=()
for tool in git cmake make python3 sha256sum; do
  command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
done
if ! command -v git-lfs >/dev/null 2>&1; then missing+=("git-lfs"); fi
if ! command -v nvcc >/dev/null 2>&1; then missing+=("nvcc/CUDA toolkit"); fi
if ! command -v rospack >/dev/null 2>&1; then missing+=("ROS Noetic"); fi
if [ "${#missing[@]}" -gt 0 ]; then
  if [ "$INSTALL_APT" = "true" ]; then
    sudo apt-get update
    sudo apt-get install -y git-lfs cmake build-essential ros-noetic-vision-msgs
  else
    printf '  missing: %s\n' "${missing[*]}" >&2
    echo "  Install build-essential, git-lfs, ros-noetic-vision-msgs, CUDA and TensorRT." >&2
    echo "  INSTALL_APT=true can install only the ordinary apt packages; CUDA/TensorRT still require NVIDIA packages." >&2
    exit 2
  fi
fi

set +u
source /opt/ros/noetic/setup.bash
set -u
rospack find vision_msgs >/dev/null 2>&1 || \
  fail "ros-noetic-vision-msgs is missing (sudo apt install ros-noetic-vision-msgs)"
[ -f "$LOCALIZATION_WS/devel/setup.bash" ] || \
  fail "localization workspace is not built: $LOCALIZATION_WS"

CUDA_HOME="${CUDA_HOME:-$(dirname "$(dirname "$(readlink -f "$(command -v nvcc)")")")}" 
CUDA_INC="$CUDA_HOME/include"
if [ -d "$CUDA_HOME/targets/x86_64-linux/lib" ]; then
  CUDA_LIB="$CUDA_HOME/targets/x86_64-linux/lib"
elif [ -d "$CUDA_HOME/lib64" ]; then
  CUDA_LIB="$CUDA_HOME/lib64"
else
  fail "cannot locate CUDA libraries below $CUDA_HOME"
fi
[ -f "$CUDA_INC/cuda_runtime.h" ] || fail "CUDA headers missing below $CUDA_INC"

TENSORRT_HEADER="$(find /usr/include /usr/local/include -name NvInfer.h -print -quit 2>/dev/null || true)"
[ -n "$TENSORRT_HEADER" ] || fail "TensorRT headers are missing (NvInfer.h)"
TENSORRT_INC="$(dirname "$TENSORRT_HEADER")"
TENSORRT_LIBRARY="$(ldconfig -p 2>/dev/null | awk '/libnvinfer\.so( |$)/ {print $NF; exit}')"
[ -n "$TENSORRT_LIBRARY" ] || fail "TensorRT runtime library libnvinfer.so is missing"
TENSORRT_LIB="$(dirname "$TENSORRT_LIBRARY")"
TRTEXEC="$(command -v trtexec 2>/dev/null || true)"
[ -n "$TRTEXEC" ] || [ ! -x /usr/src/tensorrt/bin/trtexec ] || TRTEXEC=/usr/src/tensorrt/bin/trtexec
[ -n "$TRTEXEC" ] && [ -x "$TRTEXEC" ] || fail "TensorRT trtexec is missing"

CUDASM="$(python3 - "$GPU_DEVICE" <<'PY'
import ctypes
import sys
lib = ctypes.cdll.LoadLibrary('libcuda.so')
if lib.cuInit(0) != 0:
    raise SystemExit(2)
major = ctypes.c_int()
minor = ctypes.c_int()
device = ctypes.c_int(int(sys.argv[1]))
if lib.cuDeviceComputeCapability(ctypes.byref(major), ctypes.byref(minor), device) != 0:
    raise SystemExit(3)
print(f"{major.value}{minor.value}")
PY
)" || fail "could not read CUDA compute capability"
echo "  CUDA home : $CUDA_HOME"
echo "  TensorRT  : $TENSORRT_LIBRARY"
echo "  trtexec   : $TRTEXEC"
echo "  CUDA SM   : $CUDASM"
[ "$CUDASM" -ge 75 ] || fail "PointPillars GPU must have compute capability 7.5 or newer"

say "checking out pinned NVIDIA CUDA-PointPillars"
mkdir -p "$(dirname "$POINTPILLARS_ROOT")"
if [ ! -d "$POINTPILLARS_ROOT/.git" ]; then
  git clone "$UPSTREAM_URL" "$POINTPILLARS_ROOT"
else
  ORIGIN="$(git -C "$POINTPILLARS_ROOT" remote get-url origin 2>/dev/null || true)"
  case "$ORIGIN" in
    "$UPSTREAM_URL"|"git@github.com:NVIDIA-AI-IOT/CUDA-PointPillars.git") ;;
    *) fail "$POINTPILLARS_ROOT is not the pinned NVIDIA repository (origin=$ORIGIN)" ;;
  esac
fi
git -C "$POINTPILLARS_ROOT" fetch --tags origin
git -C "$POINTPILLARS_ROOT" checkout --detach "$UPSTREAM_COMMIT"
git -C "$POINTPILLARS_ROOT" reset --hard "$UPSTREAM_COMMIT"
git -C "$POINTPILLARS_ROOT" submodule update --init --recursive
git -C "$POINTPILLARS_ROOT" lfs install --local
git -C "$POINTPILLARS_ROOT" lfs pull
[ "$(git -C "$POINTPILLARS_ROOT" rev-parse HEAD)" = "$UPSTREAM_COMMIT" ] || \
  fail "upstream checkout did not land on the pinned commit"
ONNX="$POINTPILLARS_ROOT/model/pointpillar.onnx"
[ -f "$ONNX" ] || fail "PointPillars ONNX model is missing after git-lfs pull"
[ "$(stat -c %s "$ONNX")" -gt 1000000 ] || \
  fail "$ONNX is still a Git LFS pointer; check git-lfs/network access"

say "building CUDA voxelization, TensorRT plugins and NMS for SM $CUDASM"
export CUDA_Inc="$CUDA_INC"
export CUDA_Lib="$CUDA_LIB"
export CUDA_Bin="$CUDA_HOME/bin"
export TensorRT_Inc="$TENSORRT_INC"
export TensorRT_Lib="$TENSORRT_LIB"
export TensorRT_Bin="$(dirname "$TRTEXEC")"
export CUDASM
export USE_Python=OFF
rm -rf "$POINTPILLARS_ROOT/build"
cmake -S "$POINTPILLARS_ROOT" -B "$POINTPILLARS_ROOT/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$POINTPILLARS_ROOT/build" --parallel "$(nproc)"
CORE_LIB="$POINTPILLARS_ROOT/build/libpointpillar_core.so"
[ -f "$CORE_LIB" ] || fail "CUDA-PointPillars core library was not built"

say "building an FP16 TensorRT engine on this RTX 2060"
ENGINE="$POINTPILLARS_ROOT/model/pointpillar.plan"
ONNX_SHA="$(sha256sum "$ONNX" | awk '{print $1}')"
TRT_VERSION="$($TRTEXEC --version 2>&1 | head -1 || true)"
ENGINE_META="$POINTPILLARS_ROOT/model/pointpillar.plan.meta"
NEED_ENGINE="$REBUILD_ENGINE"
if [ ! -s "$ENGINE" ] || [ ! -f "$ENGINE_META" ]; then
  NEED_ENGINE=true
elif ! grep -qx "gpu=$GPU_NAME" "$ENGINE_META" || \
     ! grep -qx "sm=$CUDASM" "$ENGINE_META" || \
     ! grep -qx "onnx_sha256=$ONNX_SHA" "$ENGINE_META"; then
  NEED_ENGINE=true
fi
if [ "$NEED_ENGINE" = "true" ]; then
  TMP_ENGINE="$ENGINE.tmp.$$"
  rm -f "$TMP_ENGINE"
  LD_LIBRARY_PATH="$POINTPILLARS_ROOT/build:$CUDA_LIB:$TENSORRT_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$TRTEXEC" \
      --onnx="$ONNX" \
      --fp16 \
      --plugins="$CORE_LIB" \
      --saveEngine="$TMP_ENGINE" \
      --inputIOFormats=fp16:chw,int32:chw,int32:chw \
      --builderOptimizationLevel=3 \
      > "$POINTPILLARS_ROOT/model/trtexec-rtx2060.log" 2>&1 || {
        tail -80 "$POINTPILLARS_ROOT/model/trtexec-rtx2060.log" >&2
        rm -f "$TMP_ENGINE"
        fail "TensorRT engine build failed"
      }
  [ -s "$TMP_ENGINE" ] || fail "trtexec produced no engine"
  mv "$TMP_ENGINE" "$ENGINE"
  {
    printf 'upstream_commit=%s\n' "$UPSTREAM_COMMIT"
    printf 'gpu=%s\n' "$GPU_NAME"
    printf 'sm=%s\n' "$CUDASM"
    printf 'onnx_sha256=%s\n' "$ONNX_SHA"
    printf 'trtexec=%s\n' "$TRT_VERSION"
    printf 'precision=fp16\n'
  } > "$ENGINE_META"
else
  echo "  existing engine matches this GPU and ONNX model"
fi

say "running NVIDIA's sample inference as a GPU smoke test"
SMOKE_DIR="$(mktemp -d)"
trap 'rm -rf "$SMOKE_DIR"' EXIT HUP INT TERM
LD_LIBRARY_PATH="$POINTPILLARS_ROOT/build:$CUDA_LIB:$TENSORRT_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$POINTPILLARS_ROOT/build/pointpillar" \
    "$POINTPILLARS_ROOT/data/" "$SMOKE_DIR/" --timer \
    > "$POINTPILLARS_ROOT/model/smoke-rtx2060.log" 2>&1 || {
      tail -80 "$POINTPILLARS_ROOT/model/smoke-rtx2060.log" >&2
      fail "official sample inference failed on the RTX 2060"
    }
find "$SMOKE_DIR" -name '*.txt' -size +0c -print -quit | grep -q . || \
  fail "sample inference produced no prediction files"

say "building the ROS1 PointPillars node in the field workspace"
set +u
source /opt/ros/noetic/setup.bash
source "$LOCALIZATION_WS/devel/setup.bash"
set -u
export CUDA_POINTPILLARS_ROOT="$POINTPILLARS_ROOT"
export LD_LIBRARY_PATH="$POINTPILLARS_ROOT/build:$CUDA_LIB:$TENSORRT_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$LOCALIZATION_WS"
if [ -d "$LOCALIZATION_WS/.catkin_tools" ]; then
  catkin config --append-args --cmake-args \
    -DENABLE_RTX_POINTPILLARS=ON \
    -DCUDA_POINTPILLARS_ROOT="$POINTPILLARS_ROOT"
  catkin build static_livox_localization
else
  catkin_make \
    -DENABLE_RTX_POINTPILLARS=ON \
    -DCUDA_POINTPILLARS_ROOT="$POINTPILLARS_ROOT"
fi
NODE="$LOCALIZATION_WS/devel/lib/static_livox_localization/rtx_pointpillars_node"
[ -x "$NODE" ] || fail "ROS1 RTX PointPillars node was not built: $NODE"

say "writing the persistent runtime environment"
mkdir -p "$CONFIG_DIR"
{
  printf 'export CUDA_POINTPILLARS_ROOT=%q\n' "$POINTPILLARS_ROOT"
  printf 'export POINTPILLARS_MODEL=%q\n' "$ENGINE"
  printf 'export POINTPILLARS_GPU_DEVICE=%q\n' "$GPU_DEVICE"
  printf 'export POINTPILLARS_INPUT_TOPIC=%q\n' "/cloud_registered_body"
  printf 'export POINTPILLARS_DETECTIONS_TOPIC=%q\n' "/pointpillars/detections"
  printf 'export POINTPILLARS_STATUS_TOPIC=%q\n' "/pointpillars/status"
  printf 'export POINTPILLARS_UPSTREAM_COMMIT=%q\n' "$UPSTREAM_COMMIT"
  printf 'export POINTPILLARS_LIBRARY_PATH=%q\n' "$POINTPILLARS_ROOT/build:$CUDA_LIB:$TENSORRT_LIB"
  printf 'export LD_LIBRARY_PATH="$POINTPILLARS_LIBRARY_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\n'
} > "$ENV_FILE"
chmod 0600 "$ENV_FILE"

say "RTX 2060 PointPillars setup complete"
echo "  upstream : $UPSTREAM_COMMIT"
echo "  engine   : $ENGINE"
echo "  ROS node : $NODE"
echo "  env      : $ENV_FILE"
echo ""
echo "Start the complete paused stack with:"
echo "  bash $HOME/wheelchair_localization_src/tools/hybrid.sh start"
echo "Inspect live GPU inference with:"
echo "  bash $HOME/wheelchair_localization_src/tools/hybrid.sh gpu-status"
