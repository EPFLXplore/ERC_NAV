#!/bin/bash

XAUTH=/tmp/.docker.xauth
USERNAME=xplore
CONTAINER_NAME=nav_humble_jetson
IMAGE_NAME=ghcr.io/epflxplore/nav:humble-jetson
DOCKER_COMMAND="sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; source src/docker_humble_jetson/attach.sh; ros2 launch wheels_control manual_stack.launch.py pub_urdf:=False"

# Function to check if a Docker container is running
is_container_running() {
    if [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == "true" ]; then
        echo "true"
    else
        echo "false"
    fi
}

# Prepare Xauthority data
echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""

# Get the current working directory and parent directory
current_dir=$(pwd)
parent_dir=$(dirname "$current_dir")

# Check if the container is running
container_status=$(is_container_running "$CONTAINER_NAME")


# # tune kernel receive buffer size
# echo -e "${BOLD_BLUE}Tuning kernel UDP socket buffers for CycloneDDS.${NC}"
# echo -e "${BOLD_BLUE}Warning: net.core.rmem_max is raised to 2 GiB (host-wide cap for socket receive buffers; uses kernel RAM when apps request large buffers).${NC}"
# sudo sysctl -w net.core.rmem_max=2147483647         # 2 GiB
# # CycloneDDS requests >= 1 MiB. Give it plenty of headroom.
# sudo sysctl -w net.core.wmem_max=33554432
# # Reasonable defaults for sockets that do not explicitly request a size.
# sudo sysctl -w net.core.rmem_default=8388608
# sudo sysctl -w net.core.wmem_default=8388608
# # sudo sysctl -w net.core.rmem_default=2147483647     # 2 GiB
# # normal jetson default is 212992


################################# OAK1W CAMERA devrules setup ############################

MOVIDIUS_UDEV_RULE='SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"'
MOVIDIUS_UDEV_RULE_FILE=/etc/udev/rules.d/80-movidius.rules

movidius_udev_rule_ok() {
  [[ -f "$MOVIDIUS_UDEV_RULE_FILE" ]] || return 1
  sudo grep -Fxq "$MOVIDIUS_UDEV_RULE" "$MOVIDIUS_UDEV_RULE_FILE" 2>/dev/null
}

if movidius_udev_rule_ok; then
  echo -e "${BOLD_GREEN}Movidius/OAK USB udev rule is already set up on the host (${MOVIDIUS_UDEV_RULE_FILE}).${NC}"
else
  echo -e "${RED}Movidius/OAK USB udev rule is not on the host (missing or different line); applying ${MOVIDIUS_UDEV_RULE_FILE} and reloading udev…${NC}" >&2
  if printf '%s\n' "$MOVIDIUS_UDEV_RULE" | sudo tee "$MOVIDIUS_UDEV_RULE_FILE" >/dev/null &&
     sudo udevadm control --reload-rules && sudo udevadm trigger &&
     movidius_udev_rule_ok; then
    echo -e "${BOLD_GREEN}Movidius/OAK USB udev rule installed and udev reloaded successfully.${NC}"
  else
    echo -e "${RED}Movidius/OAK USB udev rule setup failed (tee, udevadm, or verification).${NC}" >&2
  fi
fi

# USB runtime PM: force power/control=on for Movidius (OAK1W / MyriadX) so the kernel does not autosuspend them.
MOVIDIUS_USB_VID="03e7"
movidius_usb_pm_failures=0
movidius_usb_pm_nodes=0
for d in /sys/bus/usb/devices/*; do
  [[ -r "$d/idVendor" ]] || continue
  [[ "$(tr -d '[:space:]' < "$d/idVendor" 2>/dev/null)" == "$MOVIDIUS_USB_VID" ]] || continue
  ctrl="$d/power/control"
  [[ -f "$ctrl" ]] || continue
  movidius_usb_pm_nodes=$((movidius_usb_pm_nodes + 1))
  if ! echo on | sudo tee "$ctrl" >/dev/null 2>&1; then
    movidius_usb_pm_failures=$((movidius_usb_pm_failures + 1))
  fi
done
if [[ "$movidius_usb_pm_nodes" -eq 0 ]]; then
  echo -e "${BOLD_ORANGE}Movidius USB: no idVendor=${MOVIDIUS_USB_VID} sysfs nodes (unplugged or not enumerated); skipped power/control=on.${NC}"
elif [[ "$movidius_usb_pm_failures" -eq 0 ]]; then
  echo -e "${BOLD_GREEN}Movidius USB: set power/control=on on ${movidius_usb_pm_nodes} device node(s) (idVendor ${MOVIDIUS_USB_VID}) — USB autosuspend disabled for these nodes.${NC}"
else
  echo -e "${RED}Movidius USB: failed to set power/control=on on ${movidius_usb_pm_failures} of ${movidius_usb_pm_nodes} device node(s) (check sudo / sysfs).${NC}" >&2
fi

OAK1W_USB_VIDPID="03e7:2485"
OAK1W_USB_EXPECTED=3
oak1w_usb_count="$(lsusb -d "$OAK1W_USB_VIDPID" 2>/dev/null | wc -l | tr -d '[:space:]')"
[[ -n "$oak1w_usb_count" ]] || oak1w_usb_count=0
if [[ "$oak1w_usb_count" -eq "$OAK1W_USB_EXPECTED" ]]; then
  echo -e "${BOLD_GREEN}OAK1W / MyriadX: ${oak1w_usb_count} USB device(s) detected (expected ${OAK1W_USB_EXPECTED} × ID ${OAK1W_USB_VIDPID}).${NC}"
else
  echo -e "${RED}WARNING: OAK1W / MyriadX USB: expected ${OAK1W_USB_EXPECTED} devices (ID ${OAK1W_USB_VIDPID}), found ${oak1w_usb_count}.${NC}" >&2
fi

################################# END OAK1W CAMERA ############################################




if [ "$container_status" == "false" ]; then
    echo "Container $CONTAINER_NAME is not running. Starting a new container..."
    docker run -i \
        --name $CONTAINER_NAME \
        --rm \
        --privileged \
        --net=host \
        -e DISPLAY=unix$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=$XAUTH \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $XAUTH:$XAUTH \
        -v /run/user/1000/at-spi:/run/user/1000/at-spi \
        -v /dev:/dev \
        -v /run/jtop.sock:/run/jtop.sock \
        -v $parent_dir:/home/$USERNAME/dev_ws/src \
        -v nav_humble_jetson_home_volume:/home/$USERNAME \
        -v ~/Documents/ERC_NAV/docker_humble_jetson/cyclonedds_no_imu.xml:/home/xplore/cyclonedds.xml:ro \
        -e CYCLONEDDS_URI="file:///home/xplore/cyclonedds.xml" \
        --add-host=os-122140001125.local:169.254.55.220 \
        $IMAGE_NAME \
        /bin/bash -c "$DOCKER_COMMAND"
else
    echo "Container $CONTAINER_NAME is already running. Attaching to it..."
    docker exec -it $CONTAINER_NAME $DOCKER_COMMAND
fi
