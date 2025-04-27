#!/bin/bash

# This script runs the GLIM node using your custom config path
# Make sure you have sourced ROS2 before (or inside docker it's automatic)

# Set the config path
CONFIG_PATH="/home/xplore/dev_ws/src/localization/lidar/glim/config"

# Run the GLIM node
ros2 run glim_ros glim_rosnode --ros-args -p config_path:="$CONFIG_PATH"
