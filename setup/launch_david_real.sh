#!/bin/bash
DOCKER_DIR="/home/bjarke/rob5/david_rob5/docker"
COMPOSE_ARGS="-f docker_real-compose.yml"
CONTAINER="david_docker_real"
LAUNCH_CMD="ros2 launch david_launch dual_urdriver.launch.py left_robot_ip:=192.168.57.101 right_robot_ip:=192.168.57.102 launch_rviz:=true"

# No keyboard needed: the terminal stays open until you close it, and closing
# it stops the driver and the container.
cleanup() {
    trap - EXIT HUP TERM INT
    # setsid: keep running even though the terminal is going away.
    setsid bash -c "cd '$DOCKER_DIR' && docker compose $COMPOSE_ARGS down" >/dev/null 2>&1 &
    exit 0
}
trap cleanup EXIT HUP TERM INT

hold() {
    echo ""
    echo "$1"
    echo "Close this window to stop everything."
    while true; do sleep 3600; done
}

cd "$DOCKER_DIR" || hold "Could not cd to $DOCKER_DIR."

# Allow the container to open RViz on this display (harmless if not needed).
command -v xhost >/dev/null 2>&1 && xhost +local:docker >/dev/null 2>&1

echo "Starting container..."
docker compose $COMPOSE_ARGS up -d || hold "Failed to start the container."

if [ -z "$CONTAINER" ]; then
    CONTAINER=$(docker compose $COMPOSE_ARGS ps -q | head -n1)
    [ -n "$CONTAINER" ] || hold "Could not find the container started by compose."
fi

echo "Launching in container $CONTAINER..."
# bash -i so the container's .bashrc runs, as in a manual docker exec ... bash.
docker exec -it "$CONTAINER" bash -ic "source install/setup.bash && $LAUNCH_CMD"

hold "Driver exited (exit code $?)."
