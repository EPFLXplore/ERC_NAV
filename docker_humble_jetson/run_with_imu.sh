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
 
# enx0a76f8b03d57 — profile "Wired connection 2" (UUID is stable; name can vary)
# IMU USB-Ethernet Connection Setup:
NM_CONN_UUID="f78435e8-8845-3d65-82c4-4647f998b7e3"
IMU_IPV4_ADDR="169.254.56.222/24"
IMU_IPV4_ROUTE="169.254.56.221/32"
RED='\033[0;31m'
BOLD_GREEN='\033[1;32m'
# Orange (256-color); falls back reasonably on basic terminals
BOLD_ORANGE='\033[1;38;5;208m'
BOLD_BLUE='\033[1;34m'
NC='\033[0m'

if ! sudo nmcli connection show "$NM_CONN_UUID" &>/dev/null; then
  echo -e "${RED}IMU NETWORK PROFILE NOT FOUND IN NETWORKMANAGER (UUID $NM_CONN_UUID). IS THE IMU PLUGGED IN OR WAS THIS PROFILE NEVER CREATED? ABORTING.${NC}" >&2
  exit 1
fi

IMU_IFACE=$(sudo nmcli -g connection.interface-name connection show "$NM_CONN_UUID" 2>/dev/null | head -n1)
if [[ -z "$IMU_IFACE" ]] || ! ip link show "$IMU_IFACE" &>/dev/null; then
  echo -e "${RED}IMU USB-ETHERNET INTERFACE NOT PRESENT (EXPECTED INTERFACE FROM NM PROFILE: '${IMU_IFACE:-<none>}'). PLUG IN THE IMU AND RETRY. ABORTING.${NC}" >&2
  exit 1
fi

IMU_USB_HOST_IP=$(ip -4 -o addr show "$IMU_IFACE" 2>/dev/null | awk '{print $4}' | head -n1 | cut -d/ -f1)
[[ -n "$IMU_USB_HOST_IP" ]] || IMU_USB_HOST_IP="${IMU_IPV4_ADDR%%/*}"
IMU_DEVICE_IP="${IMU_IPV4_ROUTE%%/*}"
echo "IMU USB-Ethernet interface ${IMU_IFACE}: this host ${IMU_USB_HOST_IP}, IMU ${IMU_DEVICE_IP}"

imu_nm_already_configured() {
  local method addr routes gw v6 auto
  method=$(sudo nmcli -g ipv4.method connection show "$NM_CONN_UUID" 2>/dev/null) || return 1
  addr=$(sudo nmcli -g ipv4.addresses connection show "$NM_CONN_UUID" 2>/dev/null | head -n1)
  addr="${addr%% *}"
  routes=$(sudo nmcli -g ipv4.routes connection show "$NM_CONN_UUID" 2>/dev/null)
  gw=$(sudo nmcli -g ipv4.gateway connection show "$NM_CONN_UUID" 2>/dev/null | head -n1)
  v6=$(sudo nmcli -g ipv6.method connection show "$NM_CONN_UUID" 2>/dev/null)
  auto=$(sudo nmcli -g connection.autoconnect connection show "$NM_CONN_UUID" 2>/dev/null | head -n1)
  [[ "$method" == "manual" ]] || return 1
  [[ "$addr" == "$IMU_IPV4_ADDR" ]] || return 1
  echo "$routes" | grep -qF "$IMU_IPV4_ROUTE" || return 1
  [[ -z "$gw" || "$gw" == "--" ]] || return 1
  [[ "$v6" == "disabled" ]] || return 1
  [[ "$auto" == "yes" ]] || return 1
  return 0
}

if imu_nm_already_configured; then
  echo -e "${BOLD_GREEN}IMU detected by NetworkManager and the profile is already configured correctly — nothing to change; IMU network setup is OK.${NC}"
else
  echo -e "${BOLD_ORANGE}IMU detected by NetworkManager, but the connection profile is not set to the required values; applying nmcli changes…${NC}"
  sudo nmcli connection modify "$NM_CONN_UUID" \
    ipv4.method manual \
    ipv4.addresses "$IMU_IPV4_ADDR" \
    ipv4.gateway "" \
    ipv4.routes "$IMU_IPV4_ROUTE" \
    ipv6.method disabled \
    connection.autoconnect yes
  echo -e "${BOLD_GREEN}IMU setup finished: nmcli profile updated; NetworkManager configuration is correct and IMU network is OK.${NC}"
fi
################################# END IMU NETWORK SETUP ##########################################

################################# WIFI ACTIVE WARNING ##########################################
# Example active WiFi (nmcli): "S24PlusArno 1 … d5ed70ae-d5f3-4018-8f8a-4497bd1a0df8 … wifi … wlx3460f90fdc9c"
if nmcli -t -f TYPE,DEVICE connection show 2>/dev/null | awk -F: '($1 == "wifi" || $1 == "802-11-wireless") && $2 != "" && $2 != "--" { found = 1 } END { exit !found }'; then
  echo -e "${BOLD_ORANGE}watch out: there is a wifi network active !!!!${NC}"
fi
################################# END WIFI ACTIVE WARNING ##########################################

# ################################# JETSON CLOCKS ##########################################

# JETSON_CPU_TARGET_KHZ=1497600
# JETSON_CPU_MIN_FREQ_SYSFS=/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
# JETSON_CPU_MAX_FREQ_SYSFS=/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq

# if [[ -r "$JETSON_CPU_MIN_FREQ_SYSFS" && -r "$JETSON_CPU_MAX_FREQ_SYSFS" ]]; then
#   jetson_cpu_min_khz=$(tr -d '[:space:]' < "$JETSON_CPU_MIN_FREQ_SYSFS")
#   jetson_cpu_max_khz=$(tr -d '[:space:]' < "$JETSON_CPU_MAX_FREQ_SYSFS")
#   if [[ -n "$jetson_cpu_min_khz" && -n "$jetson_cpu_max_khz" ]] &&
#     (( jetson_cpu_min_khz >= JETSON_CPU_TARGET_KHZ && jetson_cpu_max_khz >= JETSON_CPU_TARGET_KHZ )); then
#     echo -e "${BOLD_GREEN}CPU cpufreq already pinned at ${jetson_cpu_max_khz} kHz (min=${jetson_cpu_min_khz} kHz, target ${JETSON_CPU_TARGET_KHZ} kHz per jetson_clocks) — skipping jetson_clocks.${NC}"
#   else
#     echo -e "${BOLD_ORANGE}CPU cpufreq min=${jetson_cpu_min_khz:-?} max=${jetson_cpu_max_khz:-?} kHz (target min/max >= ${JETSON_CPU_TARGET_KHZ} kHz) — running jetson_clocks…${NC}"
#     sudo jetson_clocks
#   fi
# else
#   echo -e "${BOLD_ORANGE}Cannot read ${JETSON_CPU_MIN_FREQ_SYSFS} / ${JETSON_CPU_MAX_FREQ_SYSFS} — running jetson_clocks…${NC}"
#   sudo jetson_clocks
# fi
# ################################# END JETSON CLOCKS ##########################################


################################# JETSON MAXN + CLOCKS ##########################################

if ! command -v nvpmodel >/dev/null 2>&1; then
  echo -e "${RED}nvpmodel is not installed or not in PATH.${NC}"
  exit 1
fi

if ! command -v jetson_clocks >/dev/null 2>&1; then
  echo -e "${RED}jetson_clocks is not installed or not in PATH.${NC}"
  exit 1
fi

# Ask for sudo once.
sudo -v || exit 1

get_nvpmodel_id() {
  sudo nvpmodel -q 2>/dev/null | awk 'END { print $1 }'
}

get_nvpmodel_name() {
  sudo nvpmodel -q 2>/dev/null |
    sed -n 's/^NV Power Mode: //p'
}

nvpmodel_id=$(get_nvpmodel_id)
nvpmodel_name=$(get_nvpmodel_name)

if [[ "$nvpmodel_id" != "0" ]]; then
  echo -e "${BOLD_ORANGE}Current power mode: ${nvpmodel_name:-unknown} (${nvpmodel_id:-unknown}).${NC}"
  echo -e "${BOLD_ORANGE}Switching Jetson to MAXN mode...${NC}"

  nvpmodel_output=$(sudo nvpmodel -m 0 2>&1)
  nvpmodel_status=$?

  [[ -n "$nvpmodel_output" ]] && echo "$nvpmodel_output"

  if (( nvpmodel_status != 0 )); then
    echo -e "${RED}Failed to switch the Jetson to MAXN mode.${NC}"
    exit 1
  fi

  # Read the mode again instead of assuming that the command succeeded.
  nvpmodel_id=$(get_nvpmodel_id)
  nvpmodel_name=$(get_nvpmodel_name)

  if [[ "$nvpmodel_id" != "0" ]]; then
    echo -e "${RED}MAXN was requested, but the active mode is still ${nvpmodel_name:-unknown} (${nvpmodel_id:-unknown}).${NC}"
    echo -e "${RED}A reboot may be required. Docker will not be started.${NC}"
    exit 1
  fi

  echo -e "${BOLD_GREEN}Jetson successfully switched to MAXN mode.${NC}"
else
  echo -e "${BOLD_GREEN}Jetson is already in MAXN mode.${NC}"
fi

echo -e "${BOLD_ORANGE}Pinning CPU, GPU and EMC clocks...${NC}"

if ! sudo /usr/bin/jetson_clocks --fan; then
  echo -e "${RED}jetson_clocks failed.${NC}"
  exit 1
fi

echo -e "${BOLD_GREEN}MAXN and jetson_clocks are active.${NC}"

################################# END JETSON MAXN + CLOCKS ##########################################



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
echo -e "${BOLD_BLUE}Tuning kernel UDP socket buffers for Ouster LiDAR and CycloneDDS.${NC}"
echo -e "${BOLD_BLUE}Warning: net.core.rmem_max is raised to 2 GiB (host-wide cap for socket receive buffers; uses kernel RAM when apps request large buffers).${NC}"
sudo sysctl -w net.core.rmem_max=2147483647         # 2 GiB
# CycloneDDS requests >= 1 MiB. Give it plenty of headroom.
sudo sysctl -w net.core.wmem_max=33554432
# Reasonable defaults for sockets that do not explicitly request a size.
sudo sysctl -w net.core.rmem_default=8388608
sudo sysctl -w net.core.wmem_default=8388608
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
    -v ~/Documents/ERC_NAV/docker_humble_jetson/cyclonedds_with_imu.xml:/home/xplore/cyclonedds.xml:ro \
    -v /home/xplore-nav/Documents/photos_competition:/home/xplore/dev_ws/photos_competition \
    -e CYCLONEDDS_URI="file:///home/xplore/cyclonedds.xml" \
    --add-host="${OUSTER_HOSTNAME}:${OUSTER_IPV4}" \
    ghcr.io/epflxplore/nav:humble-jetson \
    /bin/bash -c "sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; export PYTHONPATH=/home/xplore/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/xplore/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; /bin/bash"