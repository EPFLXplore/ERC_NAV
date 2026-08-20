#!/usr/bin/env bash
# Start the autonomous LiDAR pitch/yaw calibration on the NAV Jetson.
#
# Run this on the Jetson host from the ERC_NAV repository:
#   ./script.sh
#
# It opens a tmux session with three windows:
#   nav-shell   - the NAV Docker container
#   glim        - builds required packages, then starts GLIM on raw LiDAR data
#   calibration - runs the 3 m test drive and saves the calibration YAML
#
# The calibration node records 50 GLIM samples before and after a successful
# Nav2 goal, then updates sensors/lidar/config/lidar_calibration.yaml.

set -euo pipefail

SESSION_NAME="lidar-calibration"
NAV_CONTAINER="nav_humble_jetson"
GLIM_CONTAINER="glim_humble_jetson"
CONTAINER_WORKSPACE="/home/xplore/dev_ws"
WAIT_TIMEOUT_SEC=180
SKIP_BUILD=false

usage() {
    cat <<'EOF'
Usage: ./script.sh [--skip-build]

Starts the NAV, GLIM, and autonomous LiDAR calibration processes in tmux.

  --skip-build  Do not rebuild the required ROS packages before startup.
  -h, --help    Show this help text.
EOF
}

while (($#)); do
    case "$1" in
        --skip-build)
            SKIP_BUILD=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown option: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

for command in docker tmux; do
    command -v "$command" >/dev/null 2>&1 || {
        printf 'Required command is unavailable: %s\n' "$command" >&2
        exit 1
    }
done

for launcher in \
    "$SCRIPT_DIR/docker_humble_jetson/run_with_imu.sh" \
    "$SCRIPT_DIR/docker_humble_glim/run_glim.sh"; do
    [[ -f "$launcher" ]] || {
        printf 'Missing launcher: %s\n' "$launcher" >&2
        exit 1
    }
done

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    printf 'tmux session "%s" already exists. Attach with: tmux attach -t %s\n' \
        "$SESSION_NAME" "$SESSION_NAME" >&2
    exit 1
fi

for container in "$NAV_CONTAINER" "$GLIM_CONTAINER"; do
    if docker container inspect "$container" >/dev/null 2>&1; then
        printf 'Container "%s" already exists. Stop/remove it before starting calibration.\n' \
            "$container" >&2
        exit 1
    fi
done

shell_quote() {
    printf '%q' "$1"
}

nav_command="cd $(shell_quote "$SCRIPT_DIR/docker_humble_jetson") && exec ./run_with_imu.sh"

# Building in the NAV container updates the shared workspace volume before the
# GLIM container sources it. This keeps the two containers on matching code.
glim_command=$(cat <<EOF
set -euo pipefail
deadline=\$((SECONDS + ${WAIT_TIMEOUT_SEC}))
until docker container inspect --format '{{.State.Running}}' ${NAV_CONTAINER} 2>/dev/null | grep -qx true; do
    if (( SECONDS >= deadline )); then
        echo 'Timed out waiting for the NAV container.' >&2
        exit 1
    fi
    sleep 1
done

$(if [[ "$SKIP_BUILD" == true ]]; then
    printf '%s\n' "echo 'Skipping workspace build.'"
else
    cat <<BUILD
echo 'Building calibration packages in the NAV container...'
docker exec -it ${NAV_CONTAINER} bash -lc \\
    'source /opt/ros/humble/setup.bash && cd ${CONTAINER_WORKSPACE} && colcon build --packages-select path_planning wheels_control glim_starter'
BUILD
fi)

cd $(shell_quote "$SCRIPT_DIR/docker_humble_glim")
exec ./run_glim.sh
EOF
)

# This window waits for both containers, preserves GLIM vertical motion for
# pitch estimation, then starts the already autonomous ROS calibration node.
calibration_command=$(cat <<EOF
set -euo pipefail

wait_for_container() {
    local container="\$1"
    local deadline=\$((SECONDS + ${WAIT_TIMEOUT_SEC}))
    until docker container inspect --format '{{.State.Running}}' "\$container" 2>/dev/null | grep -qx true; do
        if (( SECONDS >= deadline )); then
            echo "Timed out waiting for container \$container" >&2
            exit 1
        fi
        sleep 1
    done
}

wait_for_glim_node() {
    local deadline=\$((SECONDS + ${WAIT_TIMEOUT_SEC}))
    until docker exec ${GLIM_CONTAINER} bash -lc \
        'source /opt/ros/humble/setup.bash && source ${CONTAINER_WORKSPACE}/install/setup.bash && ros2 node list 2>/dev/null | grep -qx /glim_odom_publisher_node'; do
        if (( SECONDS >= deadline )); then
            echo 'Timed out waiting for GLIM odometry republisher.' >&2
            exit 1
        fi
        sleep 1
    done
}

wait_for_container ${NAV_CONTAINER}
wait_for_container ${GLIM_CONTAINER}
wait_for_glim_node

echo 'Enabling GLIM Z output for pitch calibration...'
docker exec ${GLIM_CONTAINER} bash -lc \
    'source /opt/ros/humble/setup.bash && source ${CONTAINER_WORKSPACE}/install/setup.bash && ros2 param set /glim_odom_publisher_node zero_z false'

docker exec -it ${NAV_CONTAINER} bash -lc '
    source /opt/ros/humble/setup.bash
    source ${CONTAINER_WORKSPACE}/install/setup.bash
    ros2 launch path_planning lidar_calib.launch.py
'
EOF
)

calibration_tmux_command="exec bash -lc $(shell_quote "$calibration_command")"
glim_tmux_command="exec bash -lc $(shell_quote "$glim_command")"

tmux new-session -d -s "$SESSION_NAME" -n nav-shell
tmux send-keys -t "$SESSION_NAME:nav-shell" "$nav_command" C-m
tmux new-window -d -t "$SESSION_NAME" -n glim
tmux send-keys -t "$SESSION_NAME:glim" "$glim_tmux_command" C-m
tmux new-window -d -t "$SESSION_NAME" -n calibration
tmux send-keys -t "$SESSION_NAME:calibration" "$calibration_tmux_command" C-m

printf 'Started LiDAR calibration in tmux session "%s".\n' "$SESSION_NAME"
printf 'Attach with: tmux attach -t %s\n' "$SESSION_NAME"
printf 'Watch calibration with: tmux select-window -t %s:calibration\n' "$SESSION_NAME"
printf 'Stop all three processes with: tmux kill-session -t %s\n' "$SESSION_NAME"
