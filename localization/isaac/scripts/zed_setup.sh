#!/usr/bin/env bash

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

WS="/home/xplore/dev_ws/src/localization/isaac"
INSTALL_SCRIPT=$WS/scripts/install-zed-x86_64.sh

cd "$WS/src"

sudo chmod +x "$INSTALL_SCRIPT"
"$INSTALL_SCRIPT"

sudo chmod -R a+rX /usr/local/zed

cd "$WS"

rosdep install --from-paths src/zed-ros2-wrapper --ignore-src -r -y

rm -rf build install log
export ZED_DIR=/usr/local/zed
export CMAKE_PREFIX_PATH=/usr/local/zed:$CMAKE_PREFIX_PATH

colcon build --symlink-install --packages-up-to zed_wrapper --cmake-args -DCMAKE_BUILD_TYPE=Release
curl -L "http://calib.stereolabs.com/?SN=32835549" | sudo tee /usr/local/zed/settings/SN32835549.conf > /dev/null
# /usr/local/zed/tools/ZED_Explorer
