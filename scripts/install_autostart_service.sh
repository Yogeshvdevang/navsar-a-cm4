#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.."; pwd)"
RUNNER="$ROOT_DIR/scripts/start_navisar.sh"
SERVICE_NAME="navisar.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
CTRL_SERVICE_NAME="navisar-control.service"
CTRL_SERVICE_PATH="/etc/systemd/system/$CTRL_SERVICE_NAME"
RUN_USER="${SUDO_USER:-$USER}"
CTRL_RUNNER="$ROOT_DIR/tools/navisar_controller.py"

if [ ! -f "$RUNNER" ]; then
  echo "Runner script not found: $RUNNER" >&2
  exit 1
fi

tmpfile="$(mktemp)"
cat >"$tmpfile" <<EOF
[Unit]
Description=NAVISAR Autostart Service
After=network-online.target graphical.target display-manager.service systemd-user-sessions.service
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$ROOT_DIR
Environment=PYTHONUNBUFFERED=1
Environment=NAVISAR_DASHBOARD_OPEN=0
Environment=NAVISAR_DASHBOARD_HOST=0.0.0.0
Environment=NAVISAR_REQUIRE_DISPLAY=1
Environment=NAVISAR_DISPLAY_WAIT_SECONDS=180
Environment=QT_QPA_PLATFORM=xcb
Environment=DISPLAY=:0
Environment=XAUTHORITY=/home/$RUN_USER/.Xauthority
ExecStart=/usr/bin/env bash $RUNNER
Restart=always
RestartSec=8

[Install]
WantedBy=graphical.target
EOF

echo "Installing $SERVICE_PATH ..."
sudo cp "$tmpfile" "$SERVICE_PATH"
rm -f "$tmpfile"

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"

ctrl_tmpfile="$(mktemp)"
cat >"$ctrl_tmpfile" <<EOF
[Unit]
Description=NAVISAR Controller Service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$ROOT_DIR
Environment=PYTHONUNBUFFERED=1
Environment=NAVISAR_SERVICE_NAME=$SERVICE_NAME
Environment=NAVISAR_CONTROL_HOST=0.0.0.0
Environment=NAVISAR_CONTROL_PORT=8770
Environment=NAVISAR_DASHBOARD_PORT=8765
ExecStart=/usr/bin/env python3 $CTRL_RUNNER
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "Installing $CTRL_SERVICE_PATH ..."
sudo cp "$ctrl_tmpfile" "$CTRL_SERVICE_PATH"
rm -f "$ctrl_tmpfile"
sudo systemctl daemon-reload
sudo systemctl enable "$CTRL_SERVICE_NAME"
sudo systemctl restart "$CTRL_SERVICE_NAME"

echo "Autostart enabled."
echo "Check status with:"
echo "  sudo systemctl status $SERVICE_NAME"
echo "  sudo journalctl -u $SERVICE_NAME -f"
echo "Controller status:"
echo "  sudo systemctl status $CTRL_SERVICE_NAME"
echo "  sudo journalctl -u $CTRL_SERVICE_NAME -f"
