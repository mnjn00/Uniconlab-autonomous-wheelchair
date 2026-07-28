#!/usr/bin/env bash
# Put everything the vehicle needs onto the NUC, from the machine holding
# the map. Run this from a checkout of this repo, on the laptop.
#
# Three things have to arrive, and only one of them travels by git:
#   - code, route and safety band : `git pull` on the NUC (in this repo)
#   - the merged map              : 567 MB, copied here from local disk
#   - the FAST-LIO VN-100 pair    : installed into fast_lio_ws, which is
#                                   outside this repo
#
# Usage:
#   ./push_to_nuc.sh /Volumes/무제/merged_0707_0725_v1
#   NUC=mprp3@10.26.116.199 REF=main ./push_to_nuc.sh <map-dir>
#
# Everything is verified before anything is changed, and nothing is built
# or switched over until the transfer has been checked.
set -eo pipefail

MAP_SRC="${1:?usage: push_to_nuc.sh <map-directory>}"
NUC="${NUC:-mprp3@10.26.116.199}"
REF="${REF:-main}"
REPO="${REPO:-\$HOME/wheelchair_localization_src}"
WS="${WS:-\$HOME/livox_static_localization_ws}"
MAPS="${MAPS:-\$HOME/wheelchair_localization_maps}"

say() { printf '\n=== %s ===\n' "$1"; }

say "checking the map source"
PLY="$(find "$MAP_SRC" -maxdepth 1 -name '*.ply' | head -1)"
[ -n "$PLY" ] || { echo "ERROR: no .ply in $MAP_SRC" >&2; exit 1; }
[ -f "$MAP_SRC/traj_lidar.txt" ] || {
  echo "ERROR: $MAP_SRC/traj_lidar.txt missing - the localizer seeds from it" >&2
  exit 1; }
echo "  $(basename "$PLY") $(du -h "$PLY" | cut -f1)"

say "checking the NUC is reachable"
ssh -o BatchMode=yes -o ConnectTimeout=8 "$NUC" true || {
  echo "ERROR: cannot reach $NUC." >&2
  echo "       On Thunderbolt use NUC=nuc-tb; over Wi-Fi check the IP." >&2
  exit 1; }

say "copying the map (resumable - safe to re-run if interrupted)"
DEST="$(basename "$MAP_SRC")"
ssh -o BatchMode=yes "$NUC" "mkdir -p $MAPS/$DEST"
rsync -aP --inplace -e "ssh -o BatchMode=yes" \
  "$PLY" "$MAP_SRC/traj_lidar.txt" "$NUC:$MAPS/$DEST/"

say "verifying the copy byte for byte"
LOCAL_SUM="$(md5 -q "$PLY" 2>/dev/null || md5sum "$PLY" | cut -d' ' -f1)"
REMOTE_SUM="$(ssh -o BatchMode=yes "$NUC" \
  "md5sum $MAPS/$DEST/$(basename "$PLY") | cut -d' ' -f1")"
[ "$LOCAL_SUM" = "$REMOTE_SUM" ] || {
  echo "ERROR: checksum mismatch after transfer" >&2
  echo "  local  $LOCAL_SUM" >&2
  echo "  remote $REMOTE_SUM" >&2
  exit 1; }
echo "  md5 $LOCAL_SUM matches"

say "updating code, route and band on the NUC"
ssh -o BatchMode=yes "$NUC" "bash -euo pipefail -s" <<EOF
cd $REPO
git fetch --all --prune
git checkout $REF
git pull --ff-only
echo "  repo now at: \\\$(git rev-parse --short HEAD) \\\$(git log -1 --format=%s)"

for f in routes/20260727_new_route_waypoints.json \\
         routes/20260727_new_route_safety_band.json; do
  [ -f "\\\$f" ] || { echo "ERROR: \\\$f missing after pull" >&2; exit 1; }
done
python3 -c "
import json
w = json.load(open('routes/20260727_new_route_waypoints.json'))
b = json.load(open('routes/20260727_new_route_safety_band.json'))
assert all('z' in p for p in w['waypoints']), 'waypoints need z'
assert any('left_drop_m' in s for s in b['stations']), 'band needs drop depths'
print('  route: %d waypoints (with height)' % w['count'])
print('  band : %d stations (with drop depths)' % len(b['stations']))
"

echo "  converting the map"
./tools/deploy_merged_map.sh $MAPS/$DEST

echo "  installing the FAST-LIO VN-100 launch/config"
./tools/deploy_fastlio_vn100.sh

# The running code lives in a catkin workspace kept separate from this
# repo, so the package has to be pushed across and rebuilt. Verified
# rather than assumed - the layout differs between machines.
PKG="$WS/src/static_livox_localization"
if [ -L "\$PKG" ]; then
  echo "  ws package is a symlink into the repo; nothing to copy"
elif [ -d "\$PKG" ]; then
  echo "  syncing package into \$WS"
  rsync -a --delete "$REPO/src/static_livox_localization/" "\$PKG/"
else
  echo "ERROR: \$PKG not found - cannot tell how the workspace is wired." >&2
  echo "       Point WS= at the right catkin workspace and re-run." >&2
  exit 1
fi
echo "  building"
cd "$WS"
source /opt/ros/noetic/setup.bash
catkin_make >/tmp/nuc_build.log 2>&1 || {
  echo "ERROR: catkin_make failed; tail of /tmp/nuc_build.log:" >&2
  tail -25 /tmp/nuc_build.log >&2
  exit 1; }
echo "  build OK"
EOF

say "done"
cat <<'TXT'
Nothing has been started on the vehicle. Next, on the NUC, bring it up in
stages - the map, the VN-100 fusion and the leaned driving line have never
run on hardware:

  1) stack only, no driving:
       ./start_wheelchair_localization.sh
     confirm /vectornav/IMU appears BEFORE FAST-LIO initialises, and that
     the localizer reaches TRACKING against the merged map.
     If it does not, VN_IMU=0 ./start_wheelchair_localization.sh isolates
     whether the VN-100 or the map is at fault.

  2) drive it manually with the follower left PAUSED, record the black
     box, and check cross-track error and hold reasons offline before any
     autonomous run.
TXT
