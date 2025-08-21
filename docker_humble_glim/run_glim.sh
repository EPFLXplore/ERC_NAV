# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# If still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."

# Always create the file if missing
if [ ! -f $XAUTH ]; then
    touch $XAUTH
fi

# Always (re)populate the Xauthority with the real SSH xauth cookie
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Make sure it’s readable
chmod a+r $XAUTH

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

USERNAME=xplore

docker run -it \
    --name glim_humble_jetson \
    --rm \
    --privileged \
    --net=host \
    --ipc=host \
    --pid=host \
    --runtime=nvidia \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $XAUTH:$XAUTH \
    -v /run/user/1000/at-spi:/run/user/1000/at-spi \
    -v /dev:/dev \
    -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
    -v /usr/local/cuda:/usr/local/cuda:ro \
    -v $parent_dir:/home/xplore/dev_ws/src \
    -v nav_humble_jetson_home_volume:/home/xplore \
    -w /home/xplore/dev_ws/ \
    glim-humble-jetson \
    /bin/bash -c "sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; /bin/bash"
