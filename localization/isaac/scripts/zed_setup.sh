#!/usr/bin/env bash

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

# /usr/local/zed/tools/ZED_Explorer
