#!/usr/bin/env bash

set -euo pipefail

#CUDA
sudo apt update && sudo apt-get install -y cuda-cudart-12-6 cuda-libraries-12-6 libcublas-12-2 libnpp-12-2
sudo apt install -y gnupg software-properties-common
curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
  | gpg --dearmor | sudo tee /usr/share/keyrings/nvidia-jetson.gpg >/dev/null

echo "deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/x86_64/jammy r36.2 main" \
  | sudo tee /etc/apt/sources.list.d/nvidia-jetson-vpi.list

sudo apt update
sudo apt install -y libnvvpi3 vpi3-dev vpi3-samples ros-humble-zed-msgs

# ZED

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"
INSTALL_SCRIPT="$WS/scripts/install-zed-x86_64.sh"
CALIB_SRC="$WS/src/zed2i_isaac_vslam/config/SN32835549.conf"
CALIB_DST="/usr/local/zed/settings/SN32835549.conf"

cd "$WS/src"

sudo chmod +x "$INSTALL_SCRIPT"
"$INSTALL_SCRIPT"

sudo chmod -R a+rX /usr/local/zed
sudo mkdir -p /usr/local/zed/settings
sudo ln -sfn "$CALIB_SRC" "$CALIB_DST"

cd "$WS"

rosdep install --from-paths src/zed-ros2-wrapper src/zed2i_isaac_vslam --ignore-src -r -y

rm -rf build install log
export ZED_DIR=/usr/local/zed
export CMAKE_PREFIX_PATH="/usr/local/zed:${CMAKE_PREFIX_PATH:-}"

colcon build --symlink-install --packages-up-to zed2i_isaac_vslam --cmake-args -DCMAKE_BUILD_TYPE=Release
# /usr/local/zed/tools/ZED_Explorer
