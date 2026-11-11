#!/bin/bash

# Installs desktop launchers for the David dual UR driver:
#   David UR Driver - real robots (docker_real-compose.yml, real IPs)
#   David UR Sim    - mock hardware (default compose file)
#
# Keep this script in your setup folder and run it once:
#   ./install_david_launcher.sh [docker_dir]
#
# docker_dir is the folder that contains the compose files. If you leave it
# out, DOCKER_DIR below is used.
#
# Touch-only use: tap an icon, a terminal opens and everything starts.
# Close the terminal to stop the driver and the container.

SCRIPT_DIR=$(dirname "$(realpath "$0")")

# ---- Settings you may want to change ---------------------------------------
DOCKER_DIR="${1:-$SCRIPT_DIR/../docker}"     # folder with the compose files

# Real robots
REAL_COMPOSE_FILE="docker_real-compose.yml"
REAL_CONTAINER="david_docker_real"
LEFT_ROBOT_IP="192.168.57.101"
RIGHT_ROBOT_IP="192.168.57.102"
REAL_CMD="ros2 launch david_launch dual_urdriver.launch.py left_robot_ip:=$LEFT_ROBOT_IP right_robot_ip:=$RIGHT_ROBOT_IP launch_rviz:=true"

# Simulation (mock hardware)
SIM_COMPOSE_FILE=""      # empty = plain "docker compose up -d" (default compose file)
SIM_CONTAINER=""         # empty = auto-detect the container started by compose
SIM_CMD="ros2 launch david_launch dual_urdriver.launch.py left_use_mock_hardware:=true right_use_mock_hardware:=true launch_rviz:=true"
# -----------------------------------------------------------------------------

DOCKER_DIR=$(realpath "$DOCKER_DIR" 2>/dev/null || echo "$DOCKER_DIR")
APP_DIR="$HOME/.local/share/applications"
DESKTOP_DIR=$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")
ICON="$SCRIPT_DIR/icon.png"   # optional; launchers work without it

if [ ! -d "$DOCKER_DIR" ]; then
    echo "Error: docker folder $DOCKER_DIR not found."
    echo "Pass it as an argument:  ./install_david_launcher.sh /path/to/docker"
    exit 1
fi
if [ ! -f "$DOCKER_DIR/$REAL_COMPOSE_FILE" ]; then
    echo "Warning: $DOCKER_DIR/$REAL_COMPOSE_FILE not found - the real launcher will fail."
fi
has_default_compose=false
for f in docker-compose.yml docker-compose.yaml compose.yml compose.yaml; do
    [ -f "$DOCKER_DIR/$f" ] && has_default_compose=true
done
if [ -z "$SIM_COMPOSE_FILE" ] && [ "$has_default_compose" = false ]; then
    echo "Warning: no default compose file (docker-compose.yml / compose.yml) in $DOCKER_DIR - the sim launcher will fail."
fi

# ---- Terminal emulator ------------------------------------------------------
TERMINAL_EXEC=""
for term in gnome-terminal konsole xfce4-terminal x-terminal-emulator; do
    if command -v "$term" >/dev/null 2>&1; then
        TERMINAL_EXEC="$term"
        break
    fi
done
[ -n "$TERMINAL_EXEC" ] && TERM_FLAG=false || TERM_FLAG=true

mkdir -p "$APP_DIR" "$DESKTOP_DIR"

# Writes the run script, then a desktop entry that opens it in a terminal.
# Args: run_script desktop_file name comment compose_file container cmd
install_launcher() {
    local run_script="$1" file="$2" name="$3" comment="$4"
    local compose_file="$5" container="$6" cmd="$7"
    local compose_args=""
    [ -n "$compose_file" ] && compose_args="-f $compose_file"

    cat > "$run_script" <<EOF
#!/bin/bash
DOCKER_DIR="$DOCKER_DIR"
COMPOSE_ARGS="$compose_args"
CONTAINER="$container"
LAUNCH_CMD="$cmd"
EOF

    cat >> "$run_script" <<'EOF'

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
EOF
    chmod +x "$run_script"

    local exec_line
    case "$TERMINAL_EXEC" in
        gnome-terminal) exec_line="gnome-terminal -- $run_script" ;;
        xfce4-terminal) exec_line="xfce4-terminal -x $run_script" ;;
        "")             exec_line="$run_script" ;;
        *)              exec_line="$TERMINAL_EXEC -e $run_script" ;;
    esac

    cat > "$APP_DIR/$file" <<EOF
[Desktop Entry]
Type=Application
Name=$name
Comment=$comment
Exec=$exec_line
Icon=$ICON
Terminal=$TERM_FLAG
Categories=Utility;
EOF

    cp "$APP_DIR/$file" "$DESKTOP_DIR/$file"
    chmod +x "$DESKTOP_DIR/$file"
    if ! env -u GIO_USE_VFS -u GIO_MODULE_DIR gio set "$DESKTOP_DIR/$file" metadata::trusted true 2>/dev/null; then
        echo "Note: could not mark $file as trusted - right-click the icon and choose 'Allow Launching'."
    fi
}

# Remove the single launcher from earlier versions of this script.
rm -f "$SCRIPT_DIR/launch_david.sh" "$DESKTOP_DIR/david-ur-driver.desktop" "$APP_DIR/david-ur-driver.desktop"

install_launcher "$SCRIPT_DIR/launch_david_real.sh" "david-ur-real.desktop" \
    "David UR Driver" "Start the real-robot container and the dual UR driver" \
    "$REAL_COMPOSE_FILE" "$REAL_CONTAINER" "$REAL_CMD"

install_launcher "$SCRIPT_DIR/launch_david_sim.sh" "david-ur-sim.desktop" \
    "David UR Sim" "Start the simulation container and the dual UR driver with mock hardware" \
    "$SIM_COMPOSE_FILE" "$SIM_CONTAINER" "$SIM_CMD"

command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database "$APP_DIR" 2>/dev/null

echo "Installed (tap to start):"
echo "  $DESKTOP_DIR/david-ur-real.desktop   David UR Driver"
echo "  $DESKTOP_DIR/david-ur-sim.desktop    David UR Sim"
echo "Run scripts are in $SCRIPT_DIR"
echo "docker folder: $DOCKER_DIR"
echo ""
echo "If an icon shows a red cross / raw filename, right-click it and choose 'Allow Launching'."