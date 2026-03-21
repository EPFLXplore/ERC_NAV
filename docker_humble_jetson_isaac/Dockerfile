###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-jetson

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

# installing pyrealsense2 (doesnt work with pip)
# Install dependencies for building librealsense with Python bindings
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
    && apt-get clean && rm -rf /var/lib/apt/lists/*


#####################################
#Librealsense build form source

# Clone the librealsense repository with submodules
RUN git clone https://github.com/IntelRealSense/librealsense.git /librealsense \
    && cd /librealsense \
    && git submodule update --init --recursive


# Set up udev rules (optional but recommended for device access)
WORKDIR /librealsense
RUN ./scripts/setup_udev_rules.sh

# Build and install librealsense with Python bindings
WORKDIR /librealsense/build
RUN cmake ../ \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_WITH_CUDA:bool=true \
    -DBUILD_EXAMPLES=true \
    -DCMAKE_BUILD_TYPE=release && \
    make uninstall && make clean

RUN make VERBOSE=1 -j$(nproc) pyrealsense2 && make install

ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages
ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages/pyrealsense2:$PYTHONPATH

#need to copy the pyrealsense2.cpython-310-aarch64-linux-gnu.so files to
#/usr/local/lib/python3.10/dist-packages/pyrealsense2/
RUN mkdir -p /usr/local/lib/python3.10/dist-packages/pyrealsense2 && \
    cp /librealsense/build/release/pyrealsense2.cpython-310-aarch64-linux-gnu.so* /usr/local/lib/python3.10/dist-packages/pyrealsense2/

#may also need to copy the __init__.py file from librealsense/?? to the same place
RUN cp /librealsense/wrappers/python/pyrealsense2/__init__.py /usr/local/lib/python3.10/dist-packages/pyrealsense2/

#this line doesnt work
#RUN udevadm control --reload-rules && udevadm trigger


###############################################################

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
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2* \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    python3-zmq \
    libyaml-cpp-dev \
    lcov \
    libtins-dev \
    net-tools \
    iproute2 \
    dnsmasq \
    libgtsam-dev \
    libgtsam-unstable-dev \
    libgeographic-dev \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-isaac-ros-visual-slam \

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

USER $USERNAME

# sudo is required for this package (documentation)
RUN sudo pip install -U jetson-stats

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src


COPY --chown=xplore:xplore . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Install ROS 2 Nav dependencies
#CAUTION : the exec depend of gazebo-ros in xplore_description's package.xml will cause the build to crash !!
# will give the error: can't locate ros-humble-gazebo-ros

RUN rosdep update
RUN rosdep install -r -y -i --from-path src --rosdistro $ROS_DISTRO --skip-keys="ros-humble-gazebo-ros"


# Add the source of the project to the .bashrc
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc


# ENV VARIABLES
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
ENV GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
ENV ZED_DIR=/usr/local/zed
ENV CMAKE_PREFIX_PATH=/usr/local/zed

# Install additional Python packages
RUN pip install opencv-python --no-cache-dir
#RUN pip install pyrealsense2 -> does not work
# For Aruco
RUN pip install opencv-contrib-python==4.6.0.66 --no-cache-dir
RUN pip install depthai --no-cache-dir
RUN pip install pygame_gui --no-cache-dir
RUN pip install pygame --no-cache-dir
RUN pip install transforms3d --no-cache-dir
RUN pip install pyserial --no-cache-dir

# Pre-install ZED SDK and build the Isaac ZED workspace in the image
RUN sudo chmod +x /home/$USERNAME/dev_ws/src/localization/camera/isaac/scripts/zed_setup.sh \
    && /bin/bash /home/$USERNAME/dev_ws/src/localization/camera/isaac/scripts/zed_setup.sh /home/$USERNAME/dev_ws/src/localization/camera/isaac/scripts/install-zed-aarch64.sh

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*
