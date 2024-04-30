###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive


RUN apt-get update && apt-get upgrade -y
RUN apt-get install libtins-dev -y
RUN apt install net-tools -y
RUN apt-get install iproute2 -y
RUN apt install dnsmasq -y
RUN add-apt-repository ppa:borglab/gtsam-release-4.1 -y
RUN apt install libgtsam-dev libgtsam-unstable-dev -y
RUN python3 -m pip install pyserial


USER $USERNAME

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*