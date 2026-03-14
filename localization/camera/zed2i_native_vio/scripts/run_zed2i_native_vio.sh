#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$PACKAGE_DIR/../../.." && pwd)"
ISAAC_WS="$REPO_ROOT/localization/camera/isaac"
LAUNCH_FILE="$PACKAGE_DIR/launch/zed2i_native_vio.launch.py"
CALIB_SRC="$PACKAGE_DIR/config/SN32835549.conf"
CALIB_DST="/usr/local/zed/settings/SN32835549.conf"
CAMERA_NAME="${CAMERA_NAME:-zed2i}"
ZED_NODE_NAME="${ZED_NODE_NAME:-zed_node}"

for arg in "$@"; do
    case "$arg" in
        camera_name:=*) CAMERA_NAME="${arg#camera_name:=}" ;;
        zed_node_name:=*) ZED_NODE_NAME="${arg#zed_node_name:=}" ;;
    esac
done

ODOM_TOPIC="/${CAMERA_NAME}/${ZED_NODE_NAME}/odom"
LEFT_GRAY_TOPIC="/${CAMERA_NAME}/${ZED_NODE_NAME}/left_gray/image_rect_gray"

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
    if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

set +u
source /opt/ros/humble/setup.bash

if ! ros2 pkg prefix zed_wrapper >/dev/null 2>&1 && [ -f "$ISAAC_WS/install/setup.bash" ]; then
    source "$ISAAC_WS/install/setup.bash" || true
fi

if [ -f "$REPO_ROOT/install/setup.bash" ]; then
    source "$REPO_ROOT/install/setup.bash" || true
fi

set -u

if ! ros2 pkg prefix zed_wrapper >/dev/null 2>&1; then
    printf 'Could not find the `zed_wrapper` package. Build/source `localization/camera/isaac` first.\n' >&2
    exit 1
fi

if [ -f "$CALIB_SRC" ]; then
    sudo mkdir -p /usr/local/zed/settings
    sudo ln -sfn "$CALIB_SRC" "$CALIB_DST"
fi

ODOM_PUBLISHER_COUNT="$(topic_publisher_count "$ODOM_TOPIC")"
LEFT_GRAY_PUBLISHER_COUNT="$(topic_publisher_count "$LEFT_GRAY_TOPIC")"

if [ "$ODOM_PUBLISHER_COUNT" -gt 0 ]; then
        printf 'Detected active odometry publishers on %s. Reusing the running native VIO node.\n' "$ODOM_TOPIC"
        printf 'Streaming position from %s\n' "$ODOM_TOPIC"
        printf 'Press Ctrl+C to stop the position stream.\n'
        ros2 topic echo "$ODOM_TOPIC" --field pose.pose.position
        exit 0
fi

if [ "$LEFT_GRAY_PUBLISHER_COUNT" -gt 0 ]; then
    printf 'A ZED camera node is already publishing %s, but %s has no active publishers.\n' "$LEFT_GRAY_TOPIC" "$ODOM_TOPIC" >&2
    printf 'This usually means Isaac VSLAM started the camera with ZED positional tracking disabled (`pos_tracking_enabled: false`).\n' >&2
    printf 'Native VIO cannot start on top of that because the camera is already in use.\n' >&2
    printf 'Start native VIO first, then run Isaac VSLAM with `launch_zed_wrapper:=false`, or enable ZED positional tracking in the VSLAM ZED config.\n' >&2
    exit 1
fi

printf 'Launching native ZED2i VIO...\n'
ros2 launch "$LAUNCH_FILE" "$@" &
LAUNCH_PID=$!

printf 'Waiting for odometry topic %s...\n' "$ODOM_TOPIC"
until [ "$(topic_publisher_count "$ODOM_TOPIC")" -gt 0 ]; do
    sleep 1
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        wait "$LAUNCH_PID"
    fi
done

printf 'Streaming position from %s\n' "$ODOM_TOPIC"
printf 'Press Ctrl+C to stop the VIO launch and the position stream.\n'
ros2 topic echo "$ODOM_TOPIC" --field pose.pose.position
