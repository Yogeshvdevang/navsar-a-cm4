#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.."; pwd)"
RUNNER="$ROOT_DIR/scripts/start_navisar.sh"
SERVICE_NAME="navisar.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
RUN_USER="${SUDO_USER:-$USER}"

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

# Remove the legacy controller service if it was previously installed.
LEGACY_CTRL_SERVICE_NAME="navisar-control.service"
LEGACY_CTRL_SERVICE_PATH="/etc/systemd/system/$LEGACY_CTRL_SERVICE_NAME"
if sudo test -f "$LEGACY_CTRL_SERVICE_PATH"; then
  echo "Removing legacy $LEGACY_CTRL_SERVICE_NAME ..."
  sudo systemctl stop "$LEGACY_CTRL_SERVICE_NAME" || true
  sudo systemctl disable "$LEGACY_CTRL_SERVICE_NAME" || true
  sudo rm -f "$LEGACY_CTRL_SERVICE_PATH"
  sudo systemctl daemon-reload
fi

echo "Autostart enabled."
echo "Check status with:"
echo "  sudo systemctl status $SERVICE_NAME"
echo "  sudo journalctl -u $SERVICE_NAME -f"
