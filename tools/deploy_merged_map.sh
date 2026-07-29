#!/usr/bin/env bash
# Verify and deploy the canonical merged map bundle used for field localization.
#
# The 567 MiB PLY is the immutable canonical source. The NUC receives the
# hash-pinned 0.20 m PCD derived from that source; loading the 37.2 M-point PLY
# directly would expand into PCL's aligned point storage and a registration
# search index. Auto-init and ICP must therefore share the runtime PCD.
#
# Usage:
#   ./tools/deploy_merged_map.sh /path/to/merged_0707_0725_v1
#   ./tools/deploy_merged_map.sh --verify-only /path/to/merged_0707_0725_v1
set -euo pipefail

CANONICAL_NAME="mergedmap.ply"
CANONICAL_SHA256="3639f5942101e67d8f62baf533017475146ebb681f4a8482ecaf0f2a7cec6536"
CANONICAL_POINTS="37180425"
RUNTIME_NAME="merged_0707_0725_0p20m_xyzi.pcd"
RUNTIME_SHA256="ee317581328d3eaeee86ba448b0068c1016ca1452664b6cdaba2d874320d0431"
RUNTIME_POINTS="2696359"
MAP_ID="merged_0707_0725_v1"
ROUTE_NAME="20260727_new_route_waypoints.json"
BAND_NAME="20260727_new_route_safety_band.json"

usage() {
  echo "usage: deploy_merged_map.sh [--verify-only] <map-directory>" >&2
  exit 64
}

VERIFY_ONLY=0
if [ "${1:-}" = "--verify-only" ]; then
  VERIFY_ONLY=1
  shift
fi
[ "$#" -eq 1 ] || usage

SRC_DIR="$(cd "$1" && pwd -P)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEST_DIR="${DEST_DIR:-$HOME/wheelchair_localization_maps/$MAP_ID}"
CANONICAL_PATH="$SRC_DIR/$CANONICAL_NAME"
RUNTIME_PATH="$SRC_DIR/$RUNTIME_NAME"
TRAJ_PATH="$SRC_DIR/traj_lidar.txt"
ROUTE_PATH="$REPO_ROOT/routes/$ROUTE_NAME"
BAND_PATH="$REPO_ROOT/routes/$BAND_NAME"

for required in "$CANONICAL_PATH" "$RUNTIME_PATH" "$TRAJ_PATH" \
                "$ROUTE_PATH" "$BAND_PATH"; do
  if [ ! -f "$required" ]; then
    echo "ERROR: required localization asset missing: $required" >&2
    exit 2
  fi
done

sha256_file() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    shasum -a 256 "$1" | awk '{print $1}'
  fi
}

require_equal() {
  if [ "$2" != "$3" ]; then
    echo "ERROR: $1 mismatch: observed=$2 expected=$3" >&2
    exit 3
  fi
}

canonical_sha="$(sha256_file "$CANONICAL_PATH")"
runtime_sha="$(sha256_file "$RUNTIME_PATH")"
canonical_points="$(awk '/^element vertex / { print $3; exit }' "$CANONICAL_PATH")"
runtime_points="$(awk '/^POINTS / { print $2; exit }' "$RUNTIME_PATH")"
runtime_data="$(awk '/^DATA / { print $2; exit }' "$RUNTIME_PATH")"

require_equal "canonical SHA-256" "$canonical_sha" "$CANONICAL_SHA256"
require_equal "runtime SHA-256" "$runtime_sha" "$RUNTIME_SHA256"
require_equal "canonical point count" "$canonical_points" "$CANONICAL_POINTS"
require_equal "runtime point count" "$runtime_points" "$RUNTIME_POINTS"
require_equal "runtime PCD encoding" "$runtime_data" "binary"

canonical_bytes="$(wc -c < "$CANONICAL_PATH" | tr -d '[:space:]')"
runtime_bytes="$(wc -c < "$RUNTIME_PATH" | tr -d '[:space:]')"

echo "status=verified"
echo "map_id=$MAP_ID"
echo "canonical_map=$CANONICAL_PATH"
echo "canonical_sha256=$canonical_sha"
echo "canonical_points=$canonical_points"
echo "canonical_size_bytes=$canonical_bytes"
echo "runtime_map=$RUNTIME_PATH"
echo "runtime_sha256=$runtime_sha"
echo "runtime_points=$runtime_points"
echo "runtime_size_bytes=$runtime_bytes"
echo "route=$ROUTE_NAME"
echo "safety_band=$BAND_NAME"
echo "imu=builtin"
echo "speed_mps=0.6"

if [ "$VERIFY_ONLY" = "1" ]; then
  exit 0
fi

if [ -L "$DEST_DIR" ]; then
  echo "ERROR: destination directory must not be a symlink: $DEST_DIR" >&2
  exit 4
fi
mkdir -p "$DEST_DIR"
DEST_DIR="$(cd "$DEST_DIR" && pwd -P)"
canonical_dest="$DEST_DIR/$CANONICAL_NAME"
runtime_dest="$DEST_DIR/$RUNTIME_NAME"
trajectory_dest="$DEST_DIR/traj_lidar.txt"
manifest="$DEST_DIR/localization-map-manifest.json"
for target in "$canonical_dest" "$runtime_dest" "$trajectory_dest" "$manifest"; do
  if [ -L "$target" ] || { [ -e "$target" ] && [ ! -f "$target" ]; }; then
    echo "ERROR: destination target must be a regular file: $target" >&2
    exit 4
  fi
done

canonical_tmp=""
runtime_tmp=""
trajectory_tmp=""
manifest_tmp=""
cleanup_temps() {
  [ -z "$canonical_tmp" ] || rm -f "$canonical_tmp"
  [ -z "$runtime_tmp" ] || rm -f "$runtime_tmp"
  [ -z "$trajectory_tmp" ] || rm -f "$trajectory_tmp"
  [ -z "$manifest_tmp" ] || rm -f "$manifest_tmp"
}
trap cleanup_temps EXIT

if [ -e "$canonical_dest" ] && [ "$CANONICAL_PATH" -ef "$canonical_dest" ]; then
  :
else
  canonical_tmp="$(mktemp "$DEST_DIR/.canonical-map.XXXXXX")"
  cp -f "$CANONICAL_PATH" "$canonical_tmp"
  mv -f "$canonical_tmp" "$canonical_dest"
  canonical_tmp=""
fi
if [ -e "$runtime_dest" ] && [ "$RUNTIME_PATH" -ef "$runtime_dest" ]; then
  :
else
  runtime_tmp="$(mktemp "$DEST_DIR/.runtime-map.XXXXXX")"
  cp -f "$RUNTIME_PATH" "$runtime_tmp"
  mv -f "$runtime_tmp" "$runtime_dest"
  runtime_tmp=""
fi
if [ -e "$trajectory_dest" ] && [ "$TRAJ_PATH" -ef "$trajectory_dest" ]; then
  :
else
  trajectory_tmp="$(mktemp "$DEST_DIR/.trajectory.XXXXXX")"
  cp -f "$TRAJ_PATH" "$trajectory_tmp"
  mv -f "$trajectory_tmp" "$trajectory_dest"
  trajectory_tmp=""
fi

manifest_tmp="$(mktemp "$DEST_DIR/.localization-map-manifest.XXXXXX")"
manifest="$DEST_DIR/localization-map-manifest.json"
{
  printf '{\n'
  printf '  "map_id": "%s",\n' "$MAP_ID"
  printf '  "canonical_name": "%s",\n' "$CANONICAL_NAME"
  printf '  "canonical_sha256": "%s",\n' "$CANONICAL_SHA256"
  printf '  "canonical_points": %s,\n' "$CANONICAL_POINTS"
  printf '  "runtime_name": "%s",\n' "$RUNTIME_NAME"
  printf '  "runtime_sha256": "%s",\n' "$RUNTIME_SHA256"
  printf '  "runtime_points": %s,\n' "$RUNTIME_POINTS"
  printf '  "voxel_resolution_m": 0.20\n'
  printf '}\n'
} > "$manifest_tmp"
mv -f "$manifest_tmp" "$manifest"
manifest_tmp=""

echo "deployed_canonical_map=$canonical_dest"
echo "deployed_runtime_map=$runtime_dest"
echo "deployed_trajectory=$trajectory_dest"
echo "deployed_manifest=$manifest"
