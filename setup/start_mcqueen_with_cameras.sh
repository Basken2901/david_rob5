#!/usr/bin/env bash
# Run on the Docker host after the desktop launcher starts DAVID.
# Start cameras remotely, then execute the supplied McQueen command in Docker.
set -euo pipefail

if [[ "$#" -eq 0 ]]; then
    echo "Usage: DAVID_ONBOARD_SSH=user@onboard setup/start_mcqueen_with_cameras.sh <McQueen command and arguments>" >&2
    exit 2
fi
: "${DAVID_ONBOARD_SSH:?Set DAVID_ONBOARD_SSH=user@onboard (passwordless SSH required)}"

CONTAINER="${DAVID_DOCKER_CONTAINER:-david_docker_real}"
SERVICE="david-rgbd-cameras.service"
SSH_OPTIONS=(-o BatchMode=yes -o ConnectTimeout=8)

remote_service() {
    ssh "${SSH_OPTIONS[@]}" "$DAVID_ONBOARD_SSH" systemctl --user "$@" "$SERVICE"
}

if [[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]]; then
    echo "Container $CONTAINER is not running. Start DAVID with the desktop icon first." >&2
    exit 1
fi

echo "Starting onboard camera publishers..."
remote_service start

cleanup() {
    status=$?
    trap - EXIT INT TERM
    if ! remote_service stop; then
        echo "WARNING: Could not stop cameras; check onboard service manually." >&2
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# systemd can report a successful start even if a process fails immediately.
sleep 2
if ! remote_service is-active --quiet; then
    echo "Camera service exited; inspect onboard: systemctl --user status $SERVICE" >&2
    exit 1
fi

echo "Cameras running. Launching McQueen in $CONTAINER..."
docker_args=(-i)
if [[ -t 0 && -t 1 ]]; then
    docker_args=(-it)
fi

docker exec "${docker_args[@]}" "$CONTAINER" \
    bash -ic 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && exec "$@"' \
    david-mcqueen "$@"
