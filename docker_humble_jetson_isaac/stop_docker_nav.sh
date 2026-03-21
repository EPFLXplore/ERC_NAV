# Function to check if a Docker container is running
is_container_running() {
    if [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == "true" ]; then
        echo "true"
    else
        echo "false"
    fi
}

CONTAINER_NAME=nav_humble_jetson

container_status=$(is_container_running "$CONTAINER_NAME")

if [ "$container_status" == "true" ]; then
    docker stop $CONTAINER_NAME
fi