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
    -v ~/Documents/ERC_NAV/docker_humble_jetson/cyclonedds.xml:/home/xplore/cyclonedds.xml:ro \
    -v /home/xplore-nav/Documents/photos_competition:/home/xplore/dev_ws/photos_competition \
    -e CYCLONEDDS_URI="file:///home/xplore/cyclonedds.xml" \
    --add-host="${OUSTER_HOSTNAME}:${OUSTER_IPV4}" \
    ghcr.io/epflxplore/nav:humble-jetson \
    /bin/bash -c "sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; export PYTHONPATH=/home/xplore/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/xplore/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; /bin/bash"
