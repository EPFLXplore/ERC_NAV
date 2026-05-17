# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# If still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
# Always create the file if missing
if [ ! -f $XAUTH ]; then
    touch $XAUTH
fi

# Always (re)populate the Xauthority with the real SSH xauth cookie
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Make sure it’s readable
chmod a+r $XAUTH

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."


# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

USERNAME=xplore


################################# IMU NETWORK SETUP ##########################################
RED='\033[0;31m'
BOLD_GREEN='\033[1;32m'
# Orange (256-color); falls back reasonably on basic terminals
BOLD_ORANGE='\033[1;38;5;208m'
BOLD_BLUE='\033[1;34m'
NC='\033[0m'
################################# END IMU NETWORK SETUP ##########################################

################################# WIFI ACTIVE WARNING ##########################################
# Example active WiFi (nmcli): "S24PlusArno 1 … d5ed70ae-d5f3-4018-8f8a-4497bd1a0df8 … wifi … wlx3460f90fdc9c"
if sudo nmcli -t -f TYPE,STATE device status 2>/dev/null | awk -F: '$1 == "wifi" && $2 ~ /^connected/ { exit 0 } END { exit 1 }'; then
  echo -e "${BOLD_ORANGE}watch out: the is a hotspot active !!!!${NC}"
fi
################################# END WIFI ACTIVE WARNING ##########################################

################################# JETSON CLOCKS ##########################################

JETSON_CPU_TARGET_KHZ=1497600
JETSON_CPU_MIN_FREQ_SYSFS=/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
JETSON_CPU_MAX_FREQ_SYSFS=/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq

if [[ -r "$JETSON_CPU_MIN_FREQ_SYSFS" && -r "$JETSON_CPU_MAX_FREQ_SYSFS" ]]; then
  jetson_cpu_min_khz=$(tr -d '[:space:]' < "$JETSON_CPU_MIN_FREQ_SYSFS")
  jetson_cpu_max_khz=$(tr -d '[:space:]' < "$JETSON_CPU_MAX_FREQ_SYSFS")
  if [[ -n "$jetson_cpu_min_khz" && -n "$jetson_cpu_max_khz" ]] &&
    (( jetson_cpu_min_khz >= JETSON_CPU_TARGET_KHZ && jetson_cpu_max_khz >= JETSON_CPU_TARGET_KHZ )); then
    echo -e "${BOLD_GREEN}CPU cpufreq already pinned at ${jetson_cpu_max_khz} kHz (min=${jetson_cpu_min_khz} kHz, target ${JETSON_CPU_TARGET_KHZ} kHz per jetson_clocks) — skipping jetson_clocks.${NC}"
  else
    echo -e "${BOLD_ORANGE}CPU cpufreq min=${jetson_cpu_min_khz:-?} max=${jetson_cpu_max_khz:-?} kHz (target min/max >= ${JETSON_CPU_TARGET_KHZ} kHz) — running jetson_clocks…${NC}"
    sudo jetson_clocks
  fi
else
  echo -e "${BOLD_ORANGE}Cannot read ${JETSON_CPU_MIN_FREQ_SYSFS} / ${JETSON_CPU_MAX_FREQ_SYSFS} — running jetson_clocks…${NC}"
  sudo jetson_clocks
fi
################################# END JETSON CLOCKS ##########################################

################################# OUSTER LIDAR (Docker /etc/hosts) ############################
# Rover convention: LiDAR static 169.254.55.x/24 (same /24 as other onboard hosts; driver uses
# udp_dest 169.254.55.231 on this Jetson).
# One-time on sensor: from any reachable address, run (see ouster_set_rover_ipv4.sh):
#   docker_humble_jetson/ouster_set_rover_ipv4.sh <current-sensor-ipv4>
# That PUTs Ouster's persistent static override (survives power cycles). Pick OUSTER_IPV4
# unused on the rover (avoid gateway .1, this Jetson .231, other stacks).
OUSTER_HOSTNAME="os-122609000655.local"
OUSTER_IPV4="169.254.55.180"

# tune kernel receive buffer size
echo -e "${BOLD_BLUE}Warning: net.core.rmem_max is raised to 2 GiB (host-wide cap for socket receive buffers; uses kernel RAM when apps request large buffers).${NC}"
sudo sysctl -w net.core.rmem_max=2147483647         # 2 GiB
# sudo sysctl -w net.core.rmem_default=2147483647     # 2 GiB
# normal jetson default is 212992

# tune IP fragmentation settings
echo -e "${BOLD_BLUE}Warning: net.ipv4.ipfrag_time=3 shortens IPv4 fragment reassembly timeout (host-wide; can drop late fragments on lossy paths).${NC}"
sudo sysctl -w net.ipv4.ipfrag_time=3                 # 3s
echo -e "${BOLD_BLUE}Warning: net.ipv4.ipfrag_high_thresh raised to 128 MiB (host-wide IPv4 defrag memory before aggressive eviction).${NC}"
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB



if ping -c 1 -W 2 "${OUSTER_IPV4}" &>/dev/null; then
  echo -e "${BOLD_GREEN}LiDAR Connected ! (${OUSTER_IPV4})${NC}"
else
  echo -e "${RED}WARNING: LiDAR not connected ! ${NC}"
fi
################################# END OUSTER LIDAR ############################################


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

################################# MAXON EPOS4 USB reset ################################
#might fix CAN bus issues when lsusb works fine.
MAXON_USB_VIDPID="24e7:3b01"

maxon_set_power_no_suspend() {
  local d="$1"
  local ctrl="$d/power/control"
  local as="$d/power/autosuspend_delay_ms"
  [[ -f "$ctrl" ]] || return 1
  if ! echo on | sudo tee "$ctrl" >/dev/null 2>&1; then
    echo -e "${RED}Maxon EPOS4: could not set power/control=on on ${d}${NC}" >&2
    return 1
  fi
  [[ -f "$as" ]] && echo -1 | sudo tee "$as" >/dev/null 2>&1
  return 0
}

maxon_soft_reset_one() {
  local d="$1"
  local auth="$d/authorized"
  if [[ ! -f "$auth" ]]; then
    echo -e "${BOLD_ORANGE}Maxon EPOS4: no ${auth} — skip authorized reset for ${d}.${NC}"
    return 1
  fi
  echo -e "${BOLD_ORANGE}Maxon EPOS4: USB soft-reset ${d} (authorized 0→1)…${NC}"
  echo 0 | sudo tee "$auth" >/dev/null
  sleep 0.5
  echo 1 | sudo tee "$auth" >/dev/null
  return 0
}

maxon_found_before=$(lsusb -d "$MAXON_USB_VIDPID" 2>/dev/null | wc -l | tr -d '[:space:]')
[[ -n "$maxon_found_before" ]] || maxon_found_before=0

if [[ "$maxon_found_before" -eq 0 ]]; then
  echo -e "${RED}WARNING: Maxon EPOS4 not found (lsusb -d ${MAXON_USB_VIDPID}). Plug in USB-CAN / EPOS4 before starting the stack.${NC}" >&2
else
  echo -e "${BOLD_GREEN}Maxon EPOS4: ${maxon_found_before} device(s) — soft-reset + USB autosuspend off.${NC}"
  maxon_reset_count=0
  for d in /sys/bus/usb/devices/*; do
    [[ -r "$d/idVendor" && -r "$d/idProduct" ]] || continue
    vid=$(tr -d '[:space:]' < "$d/idVendor" 2>/dev/null)
    pid=$(tr -d '[:space:]' < "$d/idProduct" 2>/dev/null)
    [[ "${vid}:${pid}" == "$MAXON_USB_VIDPID" ]] || continue
    if maxon_soft_reset_one "$d"; then
      maxon_reset_count=$((maxon_reset_count + 1))
    fi
  done
  echo -e "${BOLD_ORANGE}Maxon EPOS4: waiting 2s for re-enumeration…${NC}"
  sleep 2

  maxon_pm_ok=0
  for d in /sys/bus/usb/devices/*; do
    [[ -r "$d/idVendor" && -r "$d/idProduct" ]] || continue
    vid=$(tr -d '[:space:]' < "$d/idVendor" 2>/dev/null)
    pid=$(tr -d '[:space:]' < "$d/idProduct" 2>/dev/null)
    [[ "${vid}:${pid}" == "$MAXON_USB_VIDPID" ]] || continue
    if maxon_set_power_no_suspend "$d"; then
      maxon_pm_ok=$((maxon_pm_ok + 1))
    fi
  done
  if [[ "$maxon_pm_ok" -gt 0 ]]; then
    echo -e "${BOLD_GREEN}Maxon EPOS4: power/control=on (+ autosuspend -1) on ${maxon_pm_ok} node(s).${NC}"
  fi

  maxon_found_after=$(lsusb -d "$MAXON_USB_VIDPID" 2>/dev/null | wc -l | tr -d '[:space:]')
  [[ -n "$maxon_found_after" ]] || maxon_found_after=0
  if [[ "$maxon_found_after" -ge 1 ]]; then
    echo -e "${BOLD_GREEN}Maxon EPOS4: lsusb OK (${maxon_found_after} × ${MAXON_USB_VIDPID}).${NC}"
  else
    echo -e "${RED}WARNING: Maxon EPOS4 disappeared after reset — try another USB port, cable, or powered hub.${NC}" >&2
  fi
fi
################################# END MAXON EPOS4 USB ##########################################





################################# DOCKER RUN ##########################################

docker run -it \
    --name nav_humble_jetson \
    --rm \
    --privileged \
    --net=host \
    --runtime=nvidia \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --env GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11 \
    --env GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
    --env GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $XAUTH:$XAUTH \
    -v /run/user/1000/at-spi:/run/user/1000/at-spi \
    -v /dev:/dev \
    -v /run/jtop.sock:/run/jtop.sock \
    -v $parent_dir:/home/xplore/dev_ws/src \
    -v nav_humble_jetson_home_volume:/home/xplore \
    -v ~/Documents/ERC_NAV/docker_humble_jetson/cyclonedds_no_imu.xml:/home/xplore/cyclonedds.xml:ro \
    -v /home/xplore-nav/Documents/photos_competition:/home/xplore/dev_ws/photos_competition \
    -e CYCLONEDDS_URI="file:///home/xplore/cyclonedds.xml" \
    --add-host="${OUSTER_HOSTNAME}:${OUSTER_IPV4}" \
    ghcr.io/epflxplore/nav:humble-jetson \
    /bin/bash -c "sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; export PYTHONPATH=/home/xplore/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/xplore/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; /bin/bash"
