#!/bin/bash

XAUTH=/tmp/.docker.xauth
USERNAME=xplore
CONTAINER_NAME=nav_humble_jetson
IMAGE_NAME=ghcr.io/epflxplore/nav:humble-jetson
DOCKER_COMMAND="sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; source src/docker_humble_jetson/attach.sh; ros2 launch wheels_control manual_stack.launch.py pub_urdf:=True"

# Function to check if a Docker container is running
is_container_running() {
    if [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == "true" ]; then
        echo "true"
    else
        echo "false"
    fi
}

# Prepare Xauthority data
echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""

# Get the current working directory and parent directory
current_dir=$(pwd)
parent_dir=$(dirname "$current_dir")

# Check if the container is running
container_status=$(is_container_running "$CONTAINER_NAME")

if [ "$container_status" == "false" ]; then
    echo "Container $CONTAINER_NAME is not running. Starting a new container..."
    docker run -i \
        --name $CONTAINER_NAME \
        --rm \
        --privileged \
        --net=host \
        -e DISPLAY=unix$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=$XAUTH \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $XAUTH:$XAUTH \
        -v /run/user/1000/at-spi:/run/user/1000/at-spi \
        -v /dev:/dev \
        -v /run/jtop.sock:/run/jtop.sock \
        -v $parent_dir:/home/$USERNAME/dev_ws/src \
        -v nav_humble_jetson_home_volume:/home/$USERNAME \
        --add-host=os-122140001125.local:169.254.55.220 \
        $IMAGE_NAME \
        /bin/bash -c "$DOCKER_COMMAND"
else
    echo "Container $CONTAINER_NAME is already running. Attaching to it..."
    docker exec -it $CONTAINER_NAME $DOCKER_COMMAND
fi
