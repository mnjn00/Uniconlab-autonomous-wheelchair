#!/usr/bin/env bash
# Record one self-contained debugging bag of a driving run, then stop by itself.
#
# Records EVERYTHING except a short list of topics that are huge or redundant,
# rather than a hand-written topic list. The VectorNav is not referenced anywhere
# in this repository, so its topic name is unknown here; recording all topics is
# what guarantees it lands in the bag whatever the driver calls it. The same
# holds for /wheel_cmd and anything else published by packages outside this repo.
#
# Usage, on the NUC:
#   ./record_debug_bag.sh                 # 10 minutes
#   DURATION=5m ./record_debug_bag.sh     # something else
#
# Launch it detached if you intend to close the SSH session:
#   setsid nohup ./record_debug_bag.sh > ~/record_debug.log 2>&1 < /dev/null &
set -eo pipefail

DURATION="${DURATION:-10m}"
OUT_DIR="${OUT_DIR:-$HOME/localization_trials}"
# Rough guide from the 2026-07-07 bag: 688 s of /livox/lidar + /livox/imu was
# 2.8 GB uncompressed. Adding /cloud_registered_body roughly doubles the rate,
# and lz4 gives some of it back, so 10 minutes lands around 3-5 GB.
MIN_FREE_MB="${MIN_FREE_MB:-10240}"
# Buffer is raised well above the 256 MB default because this NUC is known to
# saturate; an overrun silently drops messages. Watch the drop check at the end.
BUFFER_MB="${BUFFER_MB:-512}"

# Excluded: the latched full-map preview and the live preview are large and
# reconstructible, the markers are cosmetic, and rosout is noise. Everything
# else is kept.
EXCLUDE='^/fast_lio_icp/(map_preview|live_preview|.*_marker)$|^/rosout(_agg)?$|/compressed|/theora'

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_BASE="$OUT_DIR/debug_${STAMP}"
BAG="${BAG_BASE}.bag"
SUMMARY="${BAG_BASE}.summary.txt"
RECORD_LOG="${BAG_BASE}.record.log"

echo "=== debug bag recorder ==="
echo "duration : $DURATION"
echo "output   : $BAG"

if ! rostopic list > /dev/null 2>&1; then
  echo "ERROR: no ROS master reachable. Source the workspace and start the stack first." >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
FREE_MB="$(df -Pm "$OUT_DIR" | awk 'NR==2 {print $4}')"
echo "free     : ${FREE_MB} MB (need ${MIN_FREE_MB})"
if [ "$FREE_MB" -lt "$MIN_FREE_MB" ]; then
  echo "ERROR: not enough free space in $OUT_DIR." >&2
  exit 1
fi

# Report what is actually live before recording, so a bag full of nothing is
# obvious now rather than after the drive.
echo ""
echo "--- live topics of interest ---"
LIVE="$(rostopic list)"
for t in /livox/lidar /livox/imu /cloud_registered_body /Odometry \
         /cmd_vel /cmd_vel_raw /cmd_vel_gated /wheel_cmd /wheel_status /mode_cmd \
         /fast_lio_icp/pose /fast_lio_icp/localization_diagnostics /tf /tf_static; do
  if echo "$LIVE" | grep -qx "$t"; then echo "  present : $t"; else echo "  MISSING : $t"; fi
done
VN="$(echo "$LIVE" | grep -iE 'vectornav|vn100|vn_100' || true)"
if [ -n "$VN" ]; then
  echo "  present : VectorNav ->"; echo "$VN" | sed 's/^/              /'
else
  echo "  MISSING : no VectorNav-looking topic is being published"
  echo "            (the VN-100 integration is not in this revision of the repo;"
  echo "             start its driver separately if you want it in the bag)"
fi

echo ""
echo "recording for $DURATION, will stop on its own ..."
rosbag record -a --lz4 --duration="$DURATION" -b "$BUFFER_MB" \
  -x "$EXCLUDE" -O "$BAG" > "$RECORD_LOG" 2>&1 &
RECORD_PID=$!

# A SIGINT reaches rosbag directly so the bag still closes cleanly, but reap it
# here too so an interrupted run does not leave an orphan holding the file.
trap 'kill -INT "$RECORD_PID" 2>/dev/null || true; wait "$RECORD_PID" 2>/dev/null || true' INT TERM
wait "$RECORD_PID" || true
trap - INT TERM

# rosbag leaves a .active file if it was killed before writing its index.
if [ -f "${BAG}.active" ]; then
  echo "recording did not close cleanly; reindexing ..." | tee -a "$SUMMARY"
  rosbag reindex "${BAG}.active" >> "$RECORD_LOG" 2>&1 || true
  mv "${BAG}.active" "$BAG" 2>/dev/null || true
fi

{
  echo "=== debug bag summary ==="
  echo "bag      : $BAG"
  echo "finished : $(date '+%Y-%m-%d %H:%M:%S')"
  echo "size     : $(du -h "$BAG" 2>/dev/null | cut -f1)"
  echo ""
  rosbag info "$BAG" 2>&1
  echo ""
  if grep -qi "dropping\|buffer exceeded" "$RECORD_LOG" 2>/dev/null; then
    echo "WARNING: rosbag reported dropped messages. The bag is incomplete."
    echo "         Re-run with a larger BUFFER_MB, or exclude more topics."
    grep -i "dropping\|buffer exceeded" "$RECORD_LOG" | head -5
  else
    echo "no dropped-message warnings in the recorder log."
  fi
} | tee "$SUMMARY"

echo ""
echo "=============================================="
echo " RECORDING FINISHED"
echo " bag     : $BAG"
echo " summary : $SUMMARY"
echo "=============================================="
# Audible cue for an operator who stepped away from the terminal.
printf '\a' 2>/dev/null || true
