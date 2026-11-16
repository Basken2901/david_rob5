#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
CONFIG="${DAVID_CAMERA_CONFIG:-$HOME/.config/david/cameras.env}"

if [[ ! -f "$CONFIG" ]]; then
    echo "Camera configuration missing: $CONFIG" >&2
    exit 1
fi

set -a
# shellcheck source=/dev/null
source "$CONFIG"
set +a

: "${DAVID_LEFT_SERIAL:?Set DAVID_LEFT_SERIAL in cameras.env}"
: "${DAVID_RIGHT_SERIAL:?Set DAVID_RIGHT_SERIAL in cameras.env}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
    echo "Build onboard: cd $WS_DIR && colcon build --packages-select david_sensor" >&2
    exit 1
fi
# shellcheck source=/dev/null
source "$WS_DIR/install/setup.bash"

exec ros2 launch david_sensor dual_rgbd.launch.py \
    left_serial:="$DAVID_LEFT_SERIAL" \
    right_serial:="$DAVID_RIGHT_SERIAL"
