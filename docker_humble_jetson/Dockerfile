###########################################
# Xplore Navigation image
# Last built: June 29th 2026
# Image Size: 21GB
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-jetson

ARG DEBIAN_FRONTEND=noninteractive

#new ros2 humble apt source setup for signing the packages
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN apt update && apt install ros2-apt-source  
RUN rm /etc/apt/sources.list.d/ros2.list
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg


RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    python3 \
    python3-setuptools \
    python3-dev \
    python3-pip \
    python3-pybind11 \
    v4l-utils \
    libxinerama-dev \
    libxcursor-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


#####################################


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
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-libg2o \
    ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-librealsense2*\
    ros-${ROS_DISTRO}-realsense2-*\
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2* \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2-ros \
    python3-zmq \
    libyaml-cpp-dev \
    lcov \
    libtins-dev \
    net-tools \
    iproute2 \
    dnsmasq \
    libgtsam-dev \
    libgtsam-unstable-dev \
    libgeographic-dev


#for liorf:
    #libgtsam-dev
    #libgtsam-unstable-dev
    #libgeographic-dev

RUN apt-get update && apt-get install -y \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-pcl-conversions \
    ros-humble-ament-copyright \
    ros-humble-ament-flake8 \
    ros-humble-ament-pep257 \
    ros-humble-ros2launch \
    ros-humble-pcl-msgs \
    iputils-ping

#very useful tools for debugging the network and the usb devices on the jetson, can be removed without any issues
RUN apt-get install -y --no-install-recommends \
    iperf3 \
    net-tools \
    nmap \
    usbutils \
    v4l-utils \
    iftop \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Add USB rules for oak-d
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart  


# Add Lidar Host
# RUN echo "169.254.55.220  os-122140001125.local" >> /etc/hosts
# Can be replaced with the --add-host argument https://docs.docker.com/reference/cli/docker/container/run/#add-host


#Setting up the CycloneDDS Config to allow ROS2 to communicate with both the Olive X1 IMU USB-Etherner itnerface and the Jetson's Ethernet interface
# COPY docker_humble_jetson/cyclonedds.xml /home/${USERNAME}/cyclonedds.xml
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
#     CYCLONEDDS_URI="file:///home/${USERNAME}/cyclonedds.xml"
#^ this is done at runtime in the run.sh

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

# sudo is required for this package (documentation)
RUN sudo pip install -U jetson-stats

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src


COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Install ROS 2 dependencies #skip keys otherwise wont build
RUN rosdep update
RUN rosdep install -r -y -i --from-path src --rosdistro $ROS_DISTRO \
  --skip-keys="gazebo_ros gazebo_ros2_control ros2_control ros2_controllers controller_manager joint_state_broadcaster position_controllers velocity_controllers rviz2"

# Add the source of the project to the .bashrc
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc


# ENV VARIABLES
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
ENV GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins

# Install additional Python packages
RUN pip install "numpy<2" --no-cache-dir
# RUN pip install opencv-python --no-cache-dir
#RUN pip install pyrealsense2 -> does not work
# For Aruco
RUN pip install opencv-contrib-python==4.6.0.66 --no-cache-dir
RUN pip install depthai==2.30 --no-cache-dir
# RUN pip install pygame_gui --no-cache-dir #no longer used
# RUN pip install pygame --no-cache-dir     #no longer used
RUN pip install transforms3d --no-cache-dir
RUN pip install pyserial --no-cache-dir

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*
