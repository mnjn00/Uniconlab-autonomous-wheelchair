#!/usr/bin/env bash
# Put everything the vehicle needs onto the NUC, from the machine holding
# the map. Run this from a checkout of this repo, on the laptop.
#
# Three things have to arrive, and only one of them travels by git:
#   - code, route and safety band : `git pull` on the NUC (in this repo)
#   - canonical + runtime map     : verified here, copied from local disk
#   - optional VN-100 override    : installed into fast_lio_ws, which is
#                                   outside this repo
#
# Usage:
#   ./push_to_nuc.sh /Volumes/무제/merged_0707_0725_v1
#   NUC=mprp3@10.26.116.199 REF=main ./push_to_nuc.sh <map-dir>
#
# Everything is verified before anything is changed, and nothing is built
# or switched over until the transfer has been checked.
set -euo pipefail

MAP_SRC="${1:?usage: push_to_nuc.sh <map-directory>}"
NUC="${NUC:-mprp3@10.26.116.199}"
REF="${REF:-main}"
GIT_REMOTE="${GIT_REMOTE:-origin}"
MAP_ID="merged_0707_0725_v1"
DEST="$MAP_ID"

say() { printf '\n=== %s ===\n' "$1"; }

if [[ "$NUC" == -* ]] || [[ ! "$NUC" =~ ^[A-Za-z0-9_.@:-]+$ ]]; then
  echo "ERROR: NUC must be a plain user@host or host value" >&2
  exit 64
fi
git check-ref-format --branch "$REF" >/dev/null || {
  echo "ERROR: invalid REF: $REF" >&2
  exit 64
}

say "checking the map source"
PLY="$MAP_SRC/mergedmap.ply"
[ -f "$PLY" ] || { echo "ERROR: $PLY missing" >&2; exit 1; }
RUNTIME_PCD="$MAP_SRC/merged_0707_0725_0p20m_xyzi.pcd"
[ -f "$RUNTIME_PCD" ] || {
  echo "ERROR: $RUNTIME_PCD missing - this is the runtime localization target" >&2
  exit 1; }
[ -f "$MAP_SRC/traj_lidar.txt" ] || {
  echo "ERROR: $MAP_SRC/traj_lidar.txt missing - the localizer seeds from it" >&2
  exit 1; }
for asset in "$PLY" "$RUNTIME_PCD" "$MAP_SRC/traj_lidar.txt"; do
  [ ! -L "$asset" ] || {
    echo "ERROR: localization source assets must not be symlinks: $asset" >&2
    exit 1
  }
done
./tools/deploy_merged_map.sh --verify-only "$MAP_SRC"

say "binding deployment to the reviewed commit"
REPO_ROOT="$(git rev-parse --show-toplevel)"
DEPLOY_DIRTY="$(git -C "$REPO_ROOT" status --porcelain --untracked-files=all -- \
  src/static_livox_localization \
  tools/push_to_nuc.sh \
  tools/deploy_merged_map.sh \
  tools/deploy_fastlio_vn100.sh \
  tools/start_wheelchair_localization.sh \
  routes \
  runtime/record_moving_localization_trial.sh \
  docs/runbooks/livox-moving-localization-ko.md)"
if [ -n "$DEPLOY_DIRTY" ]; then
  echo "ERROR: deployment inputs contain uncommitted changes:" >&2
  printf '%s\n' "$DEPLOY_DIRTY" >&2
  echo "Commit and push the reviewed localization change before deployment." >&2
  exit 1
fi
EXPECTED_COMMIT="$(git -C "$REPO_ROOT" rev-parse HEAD)"
git -C "$REPO_ROOT" fetch "$GIT_REMOTE" "$REF"
REMOTE_REF_COMMIT="$(git -C "$REPO_ROOT" rev-parse "$GIT_REMOTE/$REF")"
[ "$EXPECTED_COMMIT" = "$REMOTE_REF_COMMIT" ] || {
  echo "ERROR: local HEAD is not the exact $GIT_REMOTE/$REF commit" >&2
  echo "  local : $EXPECTED_COMMIT" >&2
  echo "  remote: $REMOTE_REF_COMMIT" >&2
  exit 1
}
echo "  expected commit: $EXPECTED_COMMIT"

say "checking the NUC is reachable"
ssh -o BatchMode=yes -o ConnectTimeout=8 "$NUC" true || {
  echo "ERROR: cannot reach $NUC." >&2
  echo "       On Thunderbolt use NUC=nuc-tb; over Wi-Fi check the IP." >&2
  exit 1; }

REMOTE_HOME="$(ssh -o BatchMode=yes "$NUC" 'printf "%s" "$HOME"')"
case "$REMOTE_HOME" in
  /*) ;;
  *) echo "ERROR: remote HOME is not an absolute path" >&2; exit 1 ;;
esac
REPO="${REPO:-$REMOTE_HOME/wheelchair_localization_src}"
WS="${WS:-$REMOTE_HOME/livox_static_localization_ws}"
MAPS="${MAPS:-$REMOTE_HOME/wheelchair_localization_maps}"

remote_bash() {
  local encoded_args=""
  local encoded_arg
  local argument
  for argument in "$@"; do
    printf -v encoded_arg '%q' "$argument"
    encoded_args+=" $encoded_arg"
  done
  ssh -o BatchMode=yes "$NUC" "bash -s --$encoded_args"
}

say "creating an isolated map staging directory"
REMOTE_STAGE="$(remote_bash "$MAPS" "$DEST" <<'REMOTE_STAGE_CREATE'
set -euo pipefail
MAPS="$1"
DEST="$2"
[ ! -L "$MAPS" ] || {
  echo "ERROR: remote maps root must not be a symlink" >&2
  exit 1
}
[ ! -e "$MAPS" ] || [ -d "$MAPS" ] || {
  echo "ERROR: remote maps root is not a directory" >&2
  exit 1
}
mkdir -p "$MAPS"
LIVE_DEST="$MAPS/$DEST"
[ ! -L "$LIVE_DEST" ] || {
  echo "ERROR: live map destination must not be a symlink" >&2
  exit 1
}
[ ! -e "$LIVE_DEST" ] || [ -d "$LIVE_DEST" ] || {
  echo "ERROR: live map destination is not a directory" >&2
  exit 1
}
mkdir -p "$LIVE_DEST"
mktemp -d "$MAPS/.incoming-$DEST.XXXXXX"
REMOTE_STAGE_CREATE
)"
if [[ "$REMOTE_STAGE" == *$'\n'* ]]; then
  echo "ERROR: remote staging path contained a newline" >&2
  exit 1
fi
case "$REMOTE_STAGE" in
  "$MAPS"/.incoming-"$DEST".*) ;;
  *) echo "ERROR: invalid remote staging path: $REMOTE_STAGE" >&2; exit 1 ;;
esac

say "copying canonical + runtime maps into staging"
printf -v encoded_stage '%q' "$REMOTE_STAGE"
# --no-xattrs as well as COPYFILE_DISABLE: current macOS stamps
# com.apple.provenance on downloaded files, and bsdtar writes it as a
# LIBARCHIVE.xattr header that GNU tar on the NUC rejects. The receiver then
# exits, the pipe breaks, and the sender reports an unexpected EOF - a
# transfer failure that looks like a network fault and is not.
COPYFILE_DISABLE=1 tar --no-xattrs -C "$MAP_SRC" -cf - \
  mergedmap.ply merged_0707_0725_0p20m_xyzi.pcd traj_lidar.txt |
  ssh -o BatchMode=yes "$NUC" "tar -xf - -C $encoded_stage"

say "verifying staged copies byte for byte"
for local_file in "$PLY" "$RUNTIME_PCD" "$MAP_SRC/traj_lidar.txt"; do
  base="$(basename "$local_file")"
  if command -v sha256sum >/dev/null 2>&1; then
    local_sum="$(sha256sum "$local_file" | awk '{print $1}')"
  else
    local_sum="$(shasum -a 256 "$local_file" | awk '{print $1}')"
  fi
  remote_sum="$(remote_bash "$REMOTE_STAGE/$base" <<'REMOTE_SHA'
sha256sum "$1" | awk '{print $1}'
REMOTE_SHA
)"
  [ "$local_sum" = "$remote_sum" ] || {
    echo "ERROR: SHA-256 mismatch for $base after transfer" >&2
    echo "  local  $local_sum" >&2
    echo "  remote $remote_sum" >&2
    exit 1
  }
  echo "  sha256 $base $local_sum matches"
done

say "updating code, route and band on the NUC"
remote_bash \
  "$REPO" "$WS" "$MAPS" "$DEST" "$EXPECTED_COMMIT" "$REF" \
  "$GIT_REMOTE" "$REMOTE_STAGE" <<'REMOTE_DEPLOY'
set -euo pipefail
REPO="$1"
WS="$2"
MAPS="$3"
DEST="$4"
EXPECTED_COMMIT="$5"
REF="$6"
GIT_REMOTE="$7"
STAGE="$8"

cd "$REPO"
git fetch --all --prune
git checkout "$REF"
git pull --ff-only "$GIT_REMOTE" "$REF"
OBSERVED_COMMIT="$(git rev-parse HEAD)"
[ "$OBSERVED_COMMIT" = "$EXPECTED_COMMIT" ] || {
  echo "ERROR: NUC checkout does not match reviewed commit" >&2
  echo "  expected: $EXPECTED_COMMIT" >&2
  echo "  observed: $OBSERVED_COMMIT" >&2
  exit 1
}
echo "  repo now at: $(git rev-parse --short HEAD) $(git log -1 --format=%s)"
REMOTE_DIRTY="$(git status --porcelain --untracked-files=all -- \
  src/static_livox_localization \
  tools/push_to_nuc.sh \
  tools/deploy_merged_map.sh \
  tools/deploy_fastlio_vn100.sh \
  tools/start_wheelchair_localization.sh \
  routes \
  runtime/record_moving_localization_trial.sh \
  docs/runbooks/livox-moving-localization-ko.md)"
[ -z "$REMOTE_DIRTY" ] || {
  echo "ERROR: NUC deployment inputs are dirty:" >&2
  printf '%s\n' "$REMOTE_DIRTY" >&2
  exit 1
}

for f in routes/20260814_route_algorithm_waypoints.json \
         routes/20260814_route_algorithm_safety_band.json \
         routes/route_2d_map_algorithm.pgm \
         routes/route_2d_map_algorithm.yaml; do
  [ -f "$f" ] || { echo "ERROR: $f missing after pull" >&2; exit 1; }
done
python3 -c "
import json
w = json.load(open('routes/20260814_route_algorithm_waypoints.json'))
b = json.load(open('routes/20260814_route_algorithm_safety_band.json'))
assert all('z' in p for p in w['waypoints']), 'waypoints need z'
assert w.get('reference_point') == 'chair_centre', 'route must be chair-centred'
assert any('left_kind' in s for s in b['stations']), 'band needs edge kinds'
c = b.get('corridor')
assert c, 'band carries no algorithm mask provenance'
assert c['source'] == 'route_2d_map_algorithm.yaml', 'band is not bound to algorithm mask'
print('  route: %d waypoints (with height)' % w['count'])
print('  band : %d stations (algorithm hard-mask authority)' % len(b['stations']))
print('  mask : %s' % c['source'])
"

echo "  deploying the verified map bundle"
DEST_DIR="$MAPS/$DEST" ./tools/deploy_merged_map.sh "$STAGE"

echo "  installing the optional FAST-LIO VN-100 override"
./tools/deploy_fastlio_vn100.sh

# The running code lives in a catkin workspace kept separate from this
# repo, so the package has to be pushed across and rebuilt. Verified
# rather than assumed - the layout differs between machines.
EXPECTED_PKG="$REPO/src/static_livox_localization"
PKG="$WS/src/static_livox_localization"
[ -d "$EXPECTED_PKG" ] || {
  echo "ERROR: reviewed package missing: $EXPECTED_PKG" >&2
  exit 1
}
EXPECTED_PKG_RESOLVED="$(readlink -f -- "$EXPECTED_PKG")"
if [ -L "$PKG" ]; then
  PKG_RESOLVED="$(readlink -f -- "$PKG")"
  [ "$PKG_RESOLVED" = "$EXPECTED_PKG_RESOLVED" ] || {
    echo "ERROR: workspace package symlink is not the reviewed package" >&2
    echo "  expected: $EXPECTED_PKG_RESOLVED" >&2
    echo "  observed: $PKG_RESOLVED" >&2
    exit 1
  }
  echo "  ws package resolves to the reviewed repo package"
elif [ -d "$PKG" ]; then
  PKG_RESOLVED="$(readlink -f -- "$PKG")"
  if [ "$PKG_RESOLVED" = "$EXPECTED_PKG_RESOLVED" ]; then
    echo "  workspace already uses the reviewed repo package"
  else
    echo "  syncing package into $WS"
    rsync -a --delete "$EXPECTED_PKG/" "$PKG/"
  fi
else
  echo "ERROR: $PKG not found - cannot tell how the workspace is wired." >&2
  echo "       Point WS= at the right catkin workspace and re-run." >&2
  exit 1
fi
echo "  building"
cd "$WS"
# ROS's setup.bash reads variables it has not set yet, so it trips the `set -u`
# this script runs under and aborts before catkin is ever invoked. Relax it for
# the source only.
# ROS's setup.bash reads variables it has not set yet, so it trips the `set -u`
# this script runs under and aborts before catkin is ever invoked. Relax it for
# the source only.
set +u
source /opt/ros/noetic/setup.bash
set -u
# Use whichever tool owns the build space. catkin_make refuses a space built by
# catkin_tools and vice versa, and this workspace has been a catkin_tools one
# since 2026-07-15 - so the catkin_make here had never actually run on the
# vehicle, and every deployment had been finished by hand without anyone
# noticing the script stopped short.
if [ -d "$WS/.catkin_tools" ]; then
  BUILD_CMD="catkin build"
else
  BUILD_CMD="catkin_make"
fi
echo "  using $BUILD_CMD (build space owned by ${BUILD_CMD%% *})"
$BUILD_CMD >/tmp/nuc_build.log 2>&1 || {
  echo "ERROR: $BUILD_CMD failed; tail of /tmp/nuc_build.log:" >&2
  tail -25 /tmp/nuc_build.log >&2
  exit 1; }
echo "  build OK"

# The bringup lives in $HOME, outside the checkout, so a pull does not touch
# it. Left behind it launches the previous route and band and skips whatever
# gates were added since - a deployment that verifies clean and still brings
# the vehicle up on superseded configuration.
# trial_0727.sh / go.sh / stop.sh travel with it. They are what gets typed at
# the chair, so leaving a stale copy behind is the same failure one step
# further on: a run brought up on last week's settings by a script that looks
# right.
for script in start_wheelchair_localization.sh trial_0727.sh go.sh go_mpc.sh \
              stop.sh; do
  BRINGUP_SRC="$REPO/tools/$script"
  BRINGUP_DST="$HOME/$script"
  [ -f "$BRINGUP_SRC" ] || {
    echo "ERROR: $BRINGUP_SRC missing" >&2; exit 1; }
  install -m 0755 "$BRINGUP_SRC" "$BRINGUP_DST"
  SRC_SUM="$(sha256sum "$BRINGUP_SRC" | awk '{print $1}')"
  DST_SUM="$(sha256sum "$BRINGUP_DST" | awk '{print $1}')"
  [ "$SRC_SUM" = "$DST_SUM" ] || {
    echo "ERROR: $script did not install cleanly" >&2; exit 1; }
  echo "  installed: $BRINGUP_DST (${SRC_SUM:0:12})"
done

case "$STAGE" in
  "$MAPS"/.incoming-"$DEST".*) ;;
  *) echo "ERROR: refusing to clean unexpected staging path" >&2; exit 1 ;;
esac
find "$STAGE" -mindepth 1 -maxdepth 1 -type f -delete
rmdir "$STAGE"
echo "  staging cleaned"
REMOTE_DEPLOY

say "done"
cat <<'TXT'
Nothing has been started on the vehicle. Next, on the NUC, bring it up in
stages. The field default is the built-in Livox IMU used to record the 0727
route; VN_IMU=1 remains an explicit diagnostic override:

  1) stack only, no driving:
       ./start_wheelchair_localization.sh
     confirm /livox/imu appears before FAST-LIO initialises and that the
     localizer reaches TRACKING against the merged runtime map.

  2) drive it manually with the follower left PAUSED, record the black
     box, and check cross-track error and hold reasons offline before any
     autonomous run.
TXT
