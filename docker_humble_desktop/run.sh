# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# If still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

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
echo "Running docker..."

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

USERNAME=xplore

docker run -it \
    --name nav_humble_desktop \
    --rm \
    --gpus all \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e XDG_RUNTIME_DIR=/tmp/runtime-xplore \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e LIBGL_ALWAYS_INDIRECT=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $XAUTH:$XAUTH \
    -v /run/user/1000/at-spi:/run/user/1000/at-spi \
    -v /dev:/dev \
    -v $parent_dir:/home/xplore/dev_ws/src \
    -v nav_humble_desktop_home_volume:/home/xplore \
    -v ${current_dir}/entrypoint.sh:/entrypoint.sh \
    --entrypoint /entrypoint.sh \
    ghcr.io/epflxplore/nav:humble-desktop
