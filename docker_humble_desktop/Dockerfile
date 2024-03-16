###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive


RUN apt-get update && apt-get upgrade -y
#RUN apt-get install libtins-dev  
RUN apt install net-tools 
#RUN apt-get install iproute2 
#RUN apt install dnsmasq 


USER $USERNAME

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*