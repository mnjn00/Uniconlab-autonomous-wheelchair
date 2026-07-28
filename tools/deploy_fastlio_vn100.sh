#!/usr/bin/env bash
# Install the VN-100 FAST-LIO launch + config into fast_lio_ws.
#
# FAST-LIO resolves `$(find fast_lio)/...` inside its own package, so these
# two files have to live there rather than in this repo's share directory.
# Both keep their repo basenames, so this is a plain copy - the built-in-IMU
# mapping_mid360.launch/mid360.yaml pair is left untouched as the rollback.
set -eo pipefail

FASTLIO_PKG="${FASTLIO_PKG:-$HOME/fast_lio_ws/src/FAST_LIO}"
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src/static_livox_localization"

[ -d "$FASTLIO_PKG/config" ] || {
  echo "ERROR: $FASTLIO_PKG/config not found; set FASTLIO_PKG" >&2; exit 1; }
[ -d "$FASTLIO_PKG/launch" ] || {
  echo "ERROR: $FASTLIO_PKG/launch not found; set FASTLIO_PKG" >&2; exit 1; }

cp -f "$SRC/config/fastlio_mid360_vn100.yaml" "$FASTLIO_PKG/config/"
cp -f "$SRC/launch/fastlio_mid360_vn100.launch" "$FASTLIO_PKG/launch/"

echo "installed into $FASTLIO_PKG:"
echo "  config/fastlio_mid360_vn100.yaml"
echo "  launch/fastlio_mid360_vn100.launch"
echo "start_wheelchair_localization.sh selects it by default (VN_IMU=1);"
echo "VN_IMU=0 reverts to mapping_mid360.launch."
