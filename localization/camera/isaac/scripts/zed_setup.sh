#!/usr/bin/env bash
set -euo pipefail
ROS_DISTRO="${ROS_DISTRO:-humble}"
JETSON_L4T_RELEASE="${JETSON_L4T_RELEASE:-r36.4}"
JETSON_SOC_REPO="${JETSON_SOC_REPO:-t234}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

# CUDA
sudo apt update
sudo apt install -y gnupg software-properties-common

# Prefer CUDA 12.6 packages when available, otherwise fall back to the latest
# CUDA 12.x package names present in apt metadata.
CUDA_CUDART_PKG="cuda-cudart-12-6"
CUDA_LIBRARIES_PKG="cuda-libraries-12-6"
if ! apt-cache show "$CUDA_CUDART_PKG" >/dev/null 2>&1; then
  CUDA_CUDART_PKG="$(apt-cache search '^cuda-cudart-12-[0-9]+$' | awk '{print $1}' | sort -V | tail -n1)"
  CUDA_LIBRARIES_PKG="$(apt-cache search '^cuda-libraries-12-[0-9]+$' | awk '{print $1}' | sort -V | tail -n1)"
fi

CUDA_PKGS=()
if [ -n "${CUDA_CUDART_PKG:-}" ]; then
  CUDA_PKGS+=("$CUDA_CUDART_PKG")
fi
if [ -n "${CUDA_LIBRARIES_PKG:-}" ]; then
  CUDA_PKGS+=("$CUDA_LIBRARIES_PKG")
fi
for pkg in libcublas-12-2 libnpp-12-2; do
  if apt-cache show "$pkg" >/dev/null 2>&1; then
    CUDA_PKGS+=("$pkg")
  fi
done
if [ "${#CUDA_PKGS[@]}" -gt 0 ]; then
  sudo apt-get install -y "${CUDA_PKGS[@]}"
else
  echo "No CUDA 12.x apt packages found in configured repositories; continuing without explicit CUDA package install."
fi

# Install VPI/ZED dependencies only when available in apt metadata.
OPTIONAL_PKGS=(libnvvpi3 vpi3-dev vpi3-samples ros-humble-zed-msgs)
AVAILABLE_OPTIONAL_PKGS=()
for pkg in "${OPTIONAL_PKGS[@]}"; do
  if apt-cache show "$pkg" >/dev/null 2>&1; then
    AVAILABLE_OPTIONAL_PKGS+=("$pkg")
  else
    echo "Package not found in apt repositories, skipping: $pkg"
  fi
done
if [ "${#AVAILABLE_OPTIONAL_PKGS[@]}" -gt 0 ]; then
  sudo apt-get install -y "${AVAILABLE_OPTIONAL_PKGS[@]}"
else
  echo "No optional VPI/ZED apt packages found; continuing."
fi

# ZED
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"

CALIB_SRC="$WS/src/zed2i_isaac_vslam/config/SN32835549.conf"
CALIB_DST="/usr/local/zed/settings/SN32835549.conf"

sudo chmod -R a+rX /usr/local/zed
sudo mkdir -p /usr/local/zed/settings
sudo ln -sfn "$CALIB_SRC" "$CALIB_DST"

cd "$WS"

# Initialize rosdep database in fresh containers before dependency resolution.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

if ! rosdep install --from-paths src/zed-ros2-wrapper src/zed2i_isaac_vslam --ignore-src -r -y; then
  echo "rosdep could not resolve all dependencies for ZED wrapper packages; continuing with best-effort prebuild."
fi

sudo rm -rf build install log

export ZED_DIR=/usr/local/zed
export CMAKE_PREFIX_PATH="/usr/local/zed:${CMAKE_PREFIX_PATH:-}"

# Always build with --packages-up-to to resolve dependencies (zed_components, zed_wrapper, etc.)
if ! colcon build --symlink-install --packages-up-to zed2i_isaac_vslam --cmake-args -DCMAKE_BUILD_TYPE=Release; then
  echo "ZED workspace build failed; check errors above."
  exit 1
fi

# /usr/local/zed/tools/ZED_Explorer