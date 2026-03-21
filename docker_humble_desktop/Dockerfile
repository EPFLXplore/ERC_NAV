###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop-nvidia

ARG DEBIAN_FRONTEND=noninteractive


# Keys fix
RUN rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates gnupg2 lsb-release software-properties-common

## ros 2
RUN mkdir -p /usr/share/keyrings \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor --batch --yes --no-tty \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
       > /etc/apt/sources.list.d/ros2.list

## Isaac
RUN mkdir -p /usr/share/keyrings \
    && curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
       | gpg --dearmor --batch --yes --no-tty \
         -o /usr/share/keyrings/isaac-ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/isaac-ros-archive-keyring.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0" \
       > /etc/apt/sources.list.d/isaac-ros.list

# Install ROS 2 Nav packages
RUN add-apt-repository ppa:borglab/gtsam-release-4.1
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-libg2o \
    ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-isaac-ros-visual-slam \
    python3-zmq \
    libyaml-cpp-dev \
    lcov \
    libtins-dev \
    net-tools \
    iproute2 \
    dnsmasq \
    libgtsam-dev \
    libgtsam-unstable-dev 


# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart

# Add Lidar Host
# RUN echo "169.254.55.220  os-122140001125.local" >> /etc/hosts
# Can be replaced with the --add-host argument https://docs.docker.com/reference/cli/docker/container/run/#add-host

USER $USERNAME

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY --chown=xplore:xplore . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Install ROS 2 Nav dependencies
RUN rosdep update \
    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r

ENV ZED_DIR=/usr/local/zed
ENV CMAKE_PREFIX_PATH=/usr/local/zed

# Add the source of the project to the .bashrc
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Install additional Python packages
RUN pip install opencv-python --no-cache-dir
# For Aruco
RUN pip install opencv-contrib-python==4.6.0.66 
RUN pip install depthai --no-cache-dir
RUN pip install pygame_gui --no-cache-dir
RUN pip install pygame --no-cache-dir
RUN pip install transforms3d --no-cache-dir
RUN pip install pyserial --no-cache-dir
RUN pip install open3d --no-cache-dir
# Fix numpy version
RUN pip uninstall numpy -y
RUN pip install numpy==1.22.4 --no-cache-dir

# Pre-install ZED SDK and build the Isaac ZED workspace in the image
RUN sudo chmod +x /home/$USERNAME/dev_ws/src/localization/camera/isaac/scripts/zed_setup.sh \
    && /bin/bash /home/$USERNAME/dev_ws/src/localization/camera/isaac/scripts/zed_setup.sh

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*
