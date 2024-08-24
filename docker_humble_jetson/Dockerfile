###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-jetson

ARG DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Nav packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-libg2o \
    ros-${ROS_DISTRO}-depthai-ros \
    python3-zmq \
    libyaml-cpp-dev \
    lcov

# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
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
RUN rosdep update \
    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r

# Add the source of the project to the .bashrc
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*