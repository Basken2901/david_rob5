#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DIR="$HOME/.config/systemd/user"
CONFIG_DIR="$HOME/.config/david"
SERVICE="$UNIT_DIR/david-rgbd-cameras.service"

mkdir -p "$UNIT_DIR" "$CONFIG_DIR"
if [[ ! -f "$CONFIG_DIR/cameras.env" ]]; then
    cp "$SCRIPT_DIR/cameras.env.example" "$CONFIG_DIR/cameras.env"
    chmod 600 "$CONFIG_DIR/cameras.env"
fi

cat > "$SERVICE" <<EOF
[Unit]
Description=DAVID dual RGB-D camera publishers

[Service]
Type=simple
ExecStart=/bin/bash $SCRIPT_DIR/run_cameras.sh
Restart=no
KillMode=control-group
TimeoutStopSec=10

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
echo "Installed $SERVICE (not enabled or started)."
echo "Set left/right serials in $CONFIG_DIR/cameras.env before starting."
