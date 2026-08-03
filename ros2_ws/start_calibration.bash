#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
if [[ -z "${1:-}" ]]; then
  echo "usage: $0 CAMERA_PASSWORD [CLOUD_TOPIC]" >&2
  exit 2
fi
source setup_calibration.bash
export HIKVISION_PASSWORD="$1"
cloud_topic="${2:-/livox/lidar}"
output_dir="${3:-calibration_samples}"
max_samples="${4:-30}"
set --
exec ros2 launch camera_lidar_chessboard_calibration calibration.launch.py \
  cloud_topic:="$cloud_topic" output_dir:="$output_dir" auto_save_max_samples:="$max_samples"
