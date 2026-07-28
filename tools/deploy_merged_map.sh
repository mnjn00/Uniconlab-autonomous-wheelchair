#!/usr/bin/env bash
# Convert a GLIM/iridescence .ply map into the .pcd the localizer reads,
# and put it where start_wheelchair_localization.sh expects it.
#
# The two formats carry the same x/y/z/intensity float32 records in the same
# order, so this rewrites the header and copies the payload byte for byte -
# no resampling, no precision loss.
#
# Usage:
#   ./deploy_merged_map.sh /path/to/merged_0707_0725_v1
# where that directory holds merged_0707_0725.ply and traj_lidar.txt.
set -eo pipefail

SRC_DIR="${1:?usage: deploy_merged_map.sh <map-directory>}"
DEST_DIR="${DEST_DIR:-$HOME/wheelchair_localization_maps/$(basename "$SRC_DIR")}"

PLY="$(find "$SRC_DIR" -maxdepth 1 -name '*.ply' | head -1)"
[ -n "$PLY" ] || { echo "ERROR: no .ply in $SRC_DIR" >&2; exit 1; }
[ -f "$SRC_DIR/traj_lidar.txt" ] || {
  echo "ERROR: $SRC_DIR/traj_lidar.txt missing - the localizer seeds from it" >&2
  exit 1
}

mkdir -p "$DEST_DIR"
PCD="$DEST_DIR/$(basename "${PLY%.ply}").pcd"

python3 - "$PLY" "$PCD" <<'PY'
import re
import sys

ply_path, pcd_path = sys.argv[1], sys.argv[2]
with open(ply_path, "rb") as source:
    header = b""
    while not header.endswith(b"end_header\n"):
        block = source.read(1)
        if not block:
            sys.exit("ERROR: %s has no PLY header terminator" % ply_path)
        header += block
    text = header.decode("ascii", "replace")

    if "format binary_little_endian" not in text:
        sys.exit("ERROR: only binary_little_endian PLY is supported; got:\n" + text)
    properties = re.findall(r"property\s+(\S+)\s+(\S+)", text)
    if [p[1] for p in properties] != ["x", "y", "z", "intensity"] or \
            any(p[0] != "float" for p in properties):
        sys.exit("ERROR: expected exactly float x,y,z,intensity; got %r"
                 % (properties,))
    count = int(re.search(r"element vertex (\d+)", text).group(1))

    with open(pcd_path, "wb") as out:
        out.write((
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 1 1 1 1\n"
            "WIDTH %d\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS %d\n"
            "DATA binary\n" % (count, count)).encode())
        copied = 0
        while True:
            chunk = source.read(16 << 20)
            if not chunk:
                break
            out.write(chunk)
            copied += len(chunk)

expected = count * 16
if copied != expected:
    sys.exit("ERROR: payload is %d bytes, header declares %d points (%d bytes)"
             % (copied, count, expected))
print("converted %d points" % count)
PY

cp -f "$SRC_DIR/traj_lidar.txt" "$DEST_DIR/traj_lidar.txt"

echo "map  : $PCD"
echo "traj : $DEST_DIR/traj_lidar.txt"
echo "start_wheelchair_localization.sh will pick these up by default."
