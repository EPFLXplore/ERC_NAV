#!/usr/bin/env bash

WS="/home/xplore/dev_ws/src/localization/isaac"

cd "$WS"

sudo apt-get install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-stereo-image-proc ros-humble-isaac-ros-zed

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
    launch_fragments:=zed_stereo_rect,visual_slam pub_frame_rate:=30.0 \
    base_frame:=zed2_camera_center camera_optical_frames:="['zed2_left_camera_optical_frame', 'zed2_right_camera_optical_frame']" \
    interface_specs_file:="$WS/isaac_ros_assets/isaac_ros_visual_slam/zed2_quickstart_interface_specs.json"

# rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam --share)/rviz/default.cfg.rviz
