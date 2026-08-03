#!/usr/bin/env bash
# Source this file from WSL: source setup_calibration.bash
_CALIB_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$_CALIB_WS/install_merge/setup.bash"
export AMENT_PREFIX_PATH="$_CALIB_WS/install_merge${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
unset _CALIB_WS
