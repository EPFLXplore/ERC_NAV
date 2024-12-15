###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-jetson

ARG DEBIAN_FRONTEND=noninteractive

################################################################
#                       realsense
RUN sudo apt update && sudo apt autoremove -y && sudo apt clean -y

RUN sudo apt-get -y update && sudo apt-get -y upgrade \
    && echo "[INFO] Upgrades are done!" \
    && sleep 2


ENV REALSENSE_BASE=/root

ENV REALSENSE_DIR=$REALSENSE_BASE/librealsense

# clone librealsense SKD
RUN git clone https://github.com/IntelRealSense/librealsense.git $REALSENSE_DIR \
    && cd $REALSENSE_DIR \
    && mkdir build
 
RUN sudo apt-get -y install python3 python3-dev libssl-dev libxinerama-dev libxcursor-dev libcanberra-gtk-module libcanberra-gtk3-module
RUN apt-get update && apt-get install -y \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*
RUN echo "[INFO] pyrealsense2 bindings build has been started!"

# Give the CUDA path to CMake
RUN sed -i '3iset(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)' $REALSENSE_DIR/CMakeLists.txt
# Set the flags so that librealsense is compatible with python

RUN cmake $REALSENSE_DIR/ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DFORCE_RSUSB_BACKEND=ON

RUN sudo make uninstall && sudo make clean

RUN echo "[INFO] Building is starting, it will take a long time like half an hour or more!"
RUN sleep 2
RUN sudo make -j$(($(nproc)-1)) && sudo make install


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
    ros-${ROS_DISTRO}-libg2o \
    ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-librealsense2*\
    ros-${ROS_DISTRO}-realsense2-*\
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2* \
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


# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idProduct}=="2485", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
#this udev rule restart doesn't work
RUN /etc/init.d/udev restart    


# Add Lidar Host
# RUN echo "169.254.55.220  os-122140001125.local" >> /etc/hosts
# Can be replaced with the --add-host argument https://docs.docker.com/reference/cli/docker/container/run/#add-host

USER $USERNAME

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Install ROS 2 Nav dependencies
#this causes the build to crash !!
#ERROR: can't locate ros-humble-gazebo-ros
#RUN rosdep update \
#    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r

RUN rosdep update \
    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r --skip-keys "ros-humble-gazebo-ros"


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

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*
