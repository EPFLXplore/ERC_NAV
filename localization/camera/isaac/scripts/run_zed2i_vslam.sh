#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"
LAUNCH_FILE="$WS/src/zed2i_isaac_vslam/launch/zed2i_isaac_vslam.launch.py"
CALIB_SRC="$WS/src/zed2i_isaac_vslam/config/SN32835549.conf"
CALIB_DST="/usr/local/zed/settings/SN32835549.conf"
CAMERA_NAME="${CAMERA_NAME:-zed2i}"
ZED_NODE_NAME="${ZED_NODE_NAME:-zed_node}"
USE_VIO_WATCHER="${USE_VIO_WATCHER:-}"
LAUNCH_ZED_WRAPPER="${LAUNCH_ZED_WRAPPER:-}"
LAUNCH_ARGS=()

for arg in "$@"; do
  case "$arg" in
    camera_name:=*) CAMERA_NAME="${arg#camera_name:=}" ;;
    zed_node_name:=*) ZED_NODE_NAME="${arg#zed_node_name:=}" ;;
    use_vio_watcher:=*) USE_VIO_WATCHER="${arg#use_vio_watcher:=}" ;;
    launch_zed_wrapper:=*) LAUNCH_ZED_WRAPPER="${arg#launch_zed_wrapper:=}" ;;
    native_vio_odom_topic:=*) continue ;;
  esac
  LAUNCH_ARGS+=("$arg")
done

if [ ! -f "$LAUNCH_FILE" ]; then
  printf 'Missing launch file: %s\n' "$LAUNCH_FILE" >&2
  exit 1
fi

if [ ! -f "$CALIB_SRC" ]; then
  printf 'Missing calibration file: %s\n' "$CALIB_SRC" >&2
  exit 1
fi

sudo mkdir -p /usr/local/zed/settings
sudo ln -sfn "$CALIB_SRC" "$CALIB_DST"

set +u
source /opt/ros/humble/setup.bash

if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash" || true
fi

set -u

if ! ros2 pkg prefix zed_wrapper >/dev/null 2>&1; then
  printf 'Could not find the `zed_wrapper` package. Build/source `localization/camera/isaac` first.\n' >&2
  exit 1
fi

if [ -z "$USE_VIO_WATCHER" ]; then
  USE_VIO_WATCHER=false
  LAUNCH_ARGS+=("use_vio_watcher:=false")
fi

if [[ "$USE_VIO_WATCHER" =~ ^(1|true|yes)$ ]]; then
  if WATCHER_PREFIX="$(ros2 pkg prefix zed2i_isaac_vslam 2>/dev/null)"; then
    if [ ! -x "$WATCHER_PREFIX/lib/zed2i_isaac_vslam/vio_watcher" ] && [ -f "$WS/src/zed2i_isaac_vslam/scripts/vio_watcher" ]; then
      printf 'Watcher executable missing in install; using source fallback from %s.\n' \
        "$WS/src/zed2i_isaac_vslam/scripts/vio_watcher"
    elif [ ! -x "$WATCHER_PREFIX/lib/zed2i_isaac_vslam/vio_watcher" ]; then
      printf 'The watcher executable is missing at %s and no source fallback was found. Rebuild `zed2i_isaac_vslam` or use `use_vio_watcher:=false`.\n' \
        "$WATCHER_PREFIX/lib/zed2i_isaac_vslam/vio_watcher" >&2
      exit 1
    fi
  elif [ ! -f "$WS/src/zed2i_isaac_vslam/scripts/vio_watcher" ]; then
    printf 'The watcher was requested but neither the installed executable nor the source script exists.\n' >&2
    exit 1
  fi
fi

ZED_NODE_FQN="/${CAMERA_NAME}/${ZED_NODE_NAME}"
DEFAULT_NATIVE_VIO_ODOM_TOPIC="${ZED_NODE_FQN}/odom"
VSLAM_NODE_FQN="/visual_slam_node"
VSLAM_CONTAINER_FQN="/${CAMERA_NAME}/zed2i_isaac_vslam_container"

if ros2 node list 2>/dev/null | grep -Fqx -- "$VSLAM_NODE_FQN" || \
   ros2 node list 2>/dev/null | grep -Fqx -- "$VSLAM_CONTAINER_FQN"; then
  printf 'Isaac VSLAM already appears to be running (%s or %s).\n' "$VSLAM_NODE_FQN" "$VSLAM_CONTAINER_FQN" >&2
  printf 'Refusing to start a second instance because it would duplicate topics such as /visual_slam/tracking/offset_vo_pose.\n' >&2
  printf 'Stop the existing VSLAM launch first if you want to restart it with different arguments.\n' >&2
  exit 1
fi

if [ -z "$LAUNCH_ZED_WRAPPER" ]; then
  if ros2 node list 2>/dev/null | grep -Fqx -- "$ZED_NODE_FQN" || \
     ros2 topic info "$DEFAULT_NATIVE_VIO_ODOM_TOPIC" >/dev/null 2>&1; then
    printf 'Detected running ZED node %s, reusing it for Isaac VSLAM.\n' "$ZED_NODE_FQN"
    LAUNCH_ARGS+=("launch_zed_wrapper:=false")
  fi
fi

ros2 launch "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}"

# exec zed_setup.sh first
# ./run_zed2i_vslam.sh use_rviz:=true
# ./run_zed2i_vslam.sh use_rviz:=true use_vio_watcher:=true
# If native VIO is already running, the script reuses the existing `/zed2i/zed_node` automatically.
# check IMU ros2 topic info /zed2i/zed_node/imu/data -v
# ros2 topic echo /visual_slam/tracking/odometry
# ros2 topic echo /visual_slam/tracking/vo_pose
