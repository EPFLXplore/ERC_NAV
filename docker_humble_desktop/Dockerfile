###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

USER $USERNAME

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*