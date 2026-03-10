#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"
CALIB_SRC="$WS/src/zed2i_isaac_vslam/config/SN32835549.conf"
CALIB_DST="/usr/local/zed/settings/SN32835549.conf"

if [ ! -f "$CALIB_SRC" ]; then
  printf 'Missing calibration file: %s\n' "$CALIB_SRC" >&2
  exit 1
fi

sudo mkdir -p /usr/local/zed/settings
sudo ln -sfn "$CALIB_SRC" "$CALIB_DST"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

ros2 launch zed2i_isaac_vslam zed2i_isaac_vslam.launch.py "$@"

# exec zed_setup.sh first
# ./run_zed2i_vslam.sh use_rviz:=true
# check IMU ros2 topic info /zed2i/zed_node/imu/data -v
# ros2 topic echo /visual_slam/tracking/odometry
# ros2 topic echo /visual_slam/tracking/vo_pose