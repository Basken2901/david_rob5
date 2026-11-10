#!/bin/bash

# Installs a desktop launcher for the David dual UR driver.
#
# Keep this script in your setup folder and run it once:
#   ./install_david_launcher.sh [docker_dir]
#
# docker_dir is the folder that contains docker_real-compose.yml. If you leave
# it out, DOCKER_DIR below is used.
#
# It creates:
#   <setup folder>/launch_david.sh          - starts the container + UR driver
#   ~/Desktop/David UR Driver.desktop       (+ an application grid entry)
#
# Touch-only use: tap the icon, a terminal opens and everything starts.
# Close the terminal to stop the driver and the container.

SCRIPT_DIR=$(dirname "$(realpath "$0")")

# ---- Settings you may want to change ---------------------------------------
DOCKER_DIR="${1:-$SCRIPT_DIR/../docker}"     # folder with the compose file
COMPOSE_FILE="docker_real-compose.yml"
CONTAINER="david_docker_real"
LEFT_ROBOT_IP="192.168.57.101"
RIGHT_ROBOT_IP="192.168.57.102"
LAUNCH_RVIZ="true"
LAUNCH_CMD="ros2 launch david_launch dual_urdriver.launch.py left_robot_ip:=$LEFT_ROBOT_IP right_robot_ip:=$RIGHT_ROBOT_IP launch_rviz:=$LAUNCH_RVIZ"
# -----------------------------------------------------------------------------

DOCKER_DIR=$(realpath "$DOCKER_DIR" 2>/dev/null || echo "$DOCKER_DIR")
RUN_SCRIPT="$SCRIPT_DIR/launch_david.sh"
APP_DIR="$HOME/.local/share/applications"
DESKTOP_DIR=$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")
ICON="$SCRIPT_DIR/icon.png"   # optional; launcher works without it

if [ ! -f "$DOCKER_DIR/$COMPOSE_FILE" ]; then
    echo "Error: $DOCKER_DIR/$COMPOSE_FILE not found."
    echo "Pass the docker folder as an argument:  ./install_david_launcher.sh /path/to/docker"
    exit 1
fi

# ---- Generate the script that does the actual work --------------------------
cat > "$RUN_SCRIPT" <<EOF
#!/bin/bash
DOCKER_DIR="$DOCKER_DIR"
COMPOSE_FILE="$COMPOSE_FILE"
CONTAINER="$CONTAINER"
LAUNCH_CMD="$LAUNCH_CMD"
EOF

cat >> "$RUN_SCRIPT" <<'EOF'

# No keyboard needed: the terminal stays open until you close it, and closing
# it stops the driver and the container.
cleanup() {
    trap - EXIT HUP TERM INT
    # setsid: keep running even though the terminal is going away.
    setsid bash -c "cd '$DOCKER_DIR' && docker compose -f '$COMPOSE_FILE' down" >/dev/null 2>&1 &
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

echo "Starting container ($COMPOSE_FILE)..."
docker compose -f "$COMPOSE_FILE" up -d || hold "Failed to start the container."

echo "Launching UR driver..."
# bash -i so the container's .bashrc runs, as in a manual docker exec ... bash.
docker exec -it "$CONTAINER" bash -ic "source install/setup.bash && $LAUNCH_CMD"

hold "Driver exited (exit code $?)."
EOF
chmod +x "$RUN_SCRIPT"

# ---- Terminal emulator ------------------------------------------------------
TERMINAL_EXEC=""
for term in gnome-terminal konsole xfce4-terminal x-terminal-emulator; do
    if command -v "$term" >/dev/null 2>&1; then
        TERMINAL_EXEC="$term"
        break
    fi
done

case "$TERMINAL_EXEC" in
    gnome-terminal) EXEC_LINE="gnome-terminal -- $RUN_SCRIPT" ;;
    xfce4-terminal) EXEC_LINE="xfce4-terminal -x $RUN_SCRIPT" ;;
    "")             EXEC_LINE="$RUN_SCRIPT" ;;
    *)              EXEC_LINE="$TERMINAL_EXEC -e $RUN_SCRIPT" ;;
esac
[ -n "$TERMINAL_EXEC" ] && TERM_FLAG=false || TERM_FLAG=true

# ---- .desktop entry ---------------------------------------------------------
mkdir -p "$APP_DIR" "$DESKTOP_DIR"
FILE="david-ur-driver.desktop"

cat > "$APP_DIR/$FILE" <<EOF
[Desktop Entry]
Type=Application
Name=David UR Driver
Comment=Start the $CONTAINER container and the dual UR driver
Exec=$EXEC_LINE
Icon=$ICON
Terminal=$TERM_FLAG
Categories=Utility;
EOF

cp "$APP_DIR/$FILE" "$DESKTOP_DIR/$FILE"
chmod +x "$DESKTOP_DIR/$FILE"
if ! env -u GIO_USE_VFS -u GIO_MODULE_DIR gio set "$DESKTOP_DIR/$FILE" metadata::trusted true 2>/dev/null; then
    echo "Note: could not mark the icon as trusted - right-click it and choose 'Allow Launching'."
fi

command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database "$APP_DIR" 2>/dev/null

echo "Installed:"
echo "  $DESKTOP_DIR/$FILE  (tap to start)"
echo "  $RUN_SCRIPT"
echo "  docker folder: $DOCKER_DIR"
echo ""
echo "If the icon shows a red cross / raw filename, right-click it and choose 'Allow Launching'."