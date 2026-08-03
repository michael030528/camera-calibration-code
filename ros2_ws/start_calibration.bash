#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
source setup_calibration.bash
if [[ -n "${HIKVISION_PASSWORD:-}" ]]; then
  # Preferred form: keep the password out of process arguments.
  cloud_topic="${1:-/livox/lidar}"
  output_dir="${2:-calibration_samples}"
  max_samples="${3:-30}"
else
  if [[ -z "${1:-}" ]]; then
    echo "set HIKVISION_PASSWORD or use: $0 CAMERA_PASSWORD [CLOUD_TOPIC] [OUTPUT_DIR] [MAX_SAMPLES]" >&2
    exit 2
  fi
  export HIKVISION_PASSWORD="$1"
  cloud_topic="${2:-/livox/lidar}"
  output_dir="${3:-calibration_samples}"
  max_samples="${4:-30}"
fi
set --
exec ros2 launch camera_lidar_chessboard_calibration calibration.launch.py \
  cloud_topic:="$cloud_topic" output_dir:="$output_dir" auto_save_max_samples:="$max_samples"
