#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$ISAAC_WS/.." && pwd)"
NATIVE_VIO_DIR="$REPO_ROOT/zed2i_native_vio"
NATIVE_LAUNCH_FILE="$NATIVE_VIO_DIR/launch/zed2i_native_vio.launch.py"
VSLAM_SCRIPT="$SCRIPT_DIR/run_zed2i_vslam.sh"
CALIB_SRC="$NATIVE_VIO_DIR/config/SN32835549.conf"
CALIB_DST="/usr/local/zed/settings/SN32835549.conf"
CAMERA_NAME="${CAMERA_NAME:-zed2i}"
ZED_NODE_NAME="${ZED_NODE_NAME:-zed_node}"
NATIVE_ARGS=()
VSLAM_ARGS=()
STARTED_NATIVE_VIO=false

for arg in "$@"; do
  case "$arg" in
    camera_name:=*) CAMERA_NAME="${arg#camera_name:=}" ;;
    zed_node_name:=*) ZED_NODE_NAME="${arg#zed_node_name:=}" ;;
  esac

  case "$arg" in
    camera_name:=*|zed_node_name:=*|serial_number:=*)
      NATIVE_ARGS+=("$arg")
      VSLAM_ARGS+=("$arg")
      ;;
    use_vio_watcher:=*|vio_watcher_reference_odom_topic:=*|vio_watcher_jump_threshold_m:=*|vio_watcher_jump_frequency_window_sec:=*|vio_watcher_jump_frequency_reset_count:=*|vio_watcher_reset_cooldown_sec:=*|vio_watcher_post_reset_check_delay_sec:=*|vio_watcher_post_reset_max_distance_m:=*)
      NATIVE_ARGS+=("$arg")
      VSLAM_ARGS+=("$arg")
      ;;
    native_pose_topic:=*|offset_pose_topic:=*|set_pose_service:=*|vio_watcher_max_pose_frame_gap_sec:=*|vio_watcher_set_pose_retry_count:=*|vio_watcher_set_pose_retry_delay_sec:=*)
      NATIVE_ARGS+=("$arg")
      ;;
    launch_zed_wrapper:=*|native_vio_odom_topic:=*)
      continue
      ;;
    *)
      VSLAM_ARGS+=("$arg")
      ;;
  esac
done

ODOM_TOPIC="/${CAMERA_NAME}/${ZED_NODE_NAME}/odom"
LEFT_GRAY_TOPIC="/${CAMERA_NAME}/${ZED_NODE_NAME}/left_gray/image_rect_gray"
VSLAM_NODE_FQN="/visual_slam_node"
VSLAM_CONTAINER_FQN="/${CAMERA_NAME}/zed2i_isaac_vslam_container"

topic_publisher_count() {
  local topic="$1"
  local topic_info

  if ! topic_info="$(ros2 topic info "$topic" -v 2>/dev/null)"; then
    printf '0\n'
    return 0
  fi

  printf '%s\n' "$topic_info" | python3 -c 'import re, sys; text = sys.stdin.read(); m = re.search(r"Publisher count:\s*(\d+)", text); print(m.group(1) if m else "0")'
}

cleanup() {
  if [ "$STARTED_NATIVE_VIO" = true ] && [ -n "${NATIVE_VIO_PID:-}" ] && kill -0 "$NATIVE_VIO_PID" 2>/dev/null; then
    kill "$NATIVE_VIO_PID" 2>/dev/null || true
    wait "$NATIVE_VIO_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if [ ! -f "$NATIVE_LAUNCH_FILE" ]; then
  printf 'Missing native VIO launch file: %s\n' "$NATIVE_LAUNCH_FILE" >&2
  exit 1
fi

if [ ! -f "$VSLAM_SCRIPT" ]; then
  printf 'Missing VSLAM script: %s\n' "$VSLAM_SCRIPT" >&2
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

if [ -f "$ISAAC_WS/install/setup.bash" ]; then
  source "$ISAAC_WS/install/setup.bash" || true
fi

if [ -f "$REPO_ROOT/install/setup.bash" ]; then
  source "$REPO_ROOT/install/setup.bash" || true
fi
set -u

if ! ros2 pkg prefix zed_wrapper >/dev/null 2>&1; then
  printf 'Could not find the `zed_wrapper` package. Build/source the camera workspaces first.\n' >&2
  exit 1
fi

if ros2 node list 2>/dev/null | grep -Fqx -- "$VSLAM_NODE_FQN" || \
   ros2 node list 2>/dev/null | grep -Fqx -- "$VSLAM_CONTAINER_FQN"; then
  printf 'Isaac VSLAM already appears to be running (%s or %s).\n' "$VSLAM_NODE_FQN" "$VSLAM_CONTAINER_FQN" >&2
  printf 'Refusing to launch another stack instance because it would create duplicate publishers and unstable outputs.\n' >&2
  printf 'Stop the existing VSLAM launch first, then rerun this combined script.\n' >&2
  exit 1
fi

ODOM_PUBLISHER_COUNT="$(topic_publisher_count "$ODOM_TOPIC")"
LEFT_GRAY_PUBLISHER_COUNT="$(topic_publisher_count "$LEFT_GRAY_TOPIC")"

if [ "$ODOM_PUBLISHER_COUNT" -gt 0 ]; then
  printf 'Detected active native VIO odometry on %s. Reusing it.\n' "$ODOM_TOPIC"
elif [ "$LEFT_GRAY_PUBLISHER_COUNT" -gt 0 ]; then
  printf 'A ZED camera node is already publishing %s, but %s has no active publishers.\n' "$LEFT_GRAY_TOPIC" "$ODOM_TOPIC" >&2
  printf 'This usually means VSLAM already owns the camera without ZED positional tracking.\n' >&2
  printf 'Stop that launch first, or use this combined script before starting VSLAM manually.\n' >&2
  exit 1
else
  printf 'Launching native ZED2i VIO first...\n'
  ros2 launch "$NATIVE_LAUNCH_FILE" "${NATIVE_ARGS[@]}" use_rviz:=false &
  NATIVE_VIO_PID=$!
  STARTED_NATIVE_VIO=true

  printf 'Waiting for native odometry topic %s...\n' "$ODOM_TOPIC"
  until [ "$(topic_publisher_count "$ODOM_TOPIC")" -gt 0 ]; do
    sleep 1
    if ! kill -0 "$NATIVE_VIO_PID" 2>/dev/null; then
      wait "$NATIVE_VIO_PID"
    fi
  done
fi

printf 'Launching Isaac VSLAM on the existing ZED node...\n'
"$VSLAM_SCRIPT" launch_zed_wrapper:=false "${VSLAM_ARGS[@]}"

# ./src/localization/camera/isaac/scripts/run_zed2i_native_vio_and_vslam.sh use_rviz:=true use_vio_watcher:=true
# ros2 topic echo /zed2i/zed_node/offset_pose
# ros2 topic echo /visual_slam/tracking/offset_vo_pose
