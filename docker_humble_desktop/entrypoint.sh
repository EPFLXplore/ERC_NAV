#!/bin/bash
# entrypoint.sh

# ROS 2 Humble
source /opt/ros/humble/setup.bash

# # Gazebo environment
source /usr/share/gazebo/setup.sh

# OpenGL / GPU
export LIBGL_ALWAYS_INDIRECT=0
export LD_LIBRARY_PATH=/usr/lib/nvidia:$LD_LIBRARY_PATH

# Dummy audio to avoid ALSA errors
export SDL_AUDIODRIVER=dummy

# Change ownership of home directory (optional)
sudo chown -R $USERNAME:$USERNAME /home/$USERNAME

# Launch interactive shell
exec /bin/bash

