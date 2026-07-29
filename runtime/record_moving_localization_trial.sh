#!/usr/bin/env bash
# Record localization evidence only. This script never publishes a motion command.
set -euo pipefail

OUTPUT_DIR="${OUTPUT_DIR:-$HOME/localization_trials}"
mkdir -p "$OUTPUT_DIR"
OUTPUT="${OUTPUT:-$OUTPUT_DIR/moving_localization_$(date +%Y%m%d_%H%M%S)}"

exec rosbag record --lz4 -O "$OUTPUT" \
  /livox/lidar \
  /livox/imu \
  /cloud_registered_body \
  /Odometry \
  /fast_lio_icp/initialpose \
  /fast_lio_icp/pose \
  /fast_lio_icp/localization_diagnostics \
  /fast_lio_icp/map_preview \
  /fast_lio_icp/live_preview \
  /tf \
  /tf_static
