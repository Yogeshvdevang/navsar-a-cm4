#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.."; pwd)"
VENV_DIR="$ROOT_DIR/venv"
PYTHON_BIN="python3"
PIP_BIN="pip3"
BOOT_CONFIG="/boot/firmware/config.txt"
BOOT_CONFIG_FALLBACK="/boot/config.txt"
CFG_START="# --- NAVISAR CM4 CONFIG START ---"
CFG_END="# --- NAVISAR CM4 CONFIG END ---"

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required but not installed." >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required but not installed." >&2
  exit 1
fi

if [ ! -f "$BOOT_CONFIG" ] && [ -f "$BOOT_CONFIG_FALLBACK" ]; then
  BOOT_CONFIG="$BOOT_CONFIG_FALLBACK"
fi

if [ ! -f "$BOOT_CONFIG" ]; then
  echo "Boot config file not found at /boot/firmware/config.txt or /boot/config.txt" >&2
  exit 1
fi

echo "[1/6] Updating boot config for camera/UART/LED..."
tmp_cfg="$(mktemp)"
cp "$BOOT_CONFIG" "$tmp_cfg"

if grep -Fq "$CFG_START" "$tmp_cfg"; then
  awk -v start="$CFG_START" -v end="$CFG_END" '
    $0 == start { in_block=1; next }
    $0 == end { in_block=0; next }
    !in_block { print }
  ' "$tmp_cfg" > "${tmp_cfg}.clean"
  mv "${tmp_cfg}.clean" "$tmp_cfg"
fi

cat >> "$tmp_cfg" <<EOF

$CFG_START
[all]
# --- VO Drone CM4 UART Config ---

# Camera
dtoverlay=ov5647,cam0

# 1. Enable UART for Primary GPS (GPIO 14/15)
# Disables Bluetooth to free up the high-quality PL011 UART for GPS
dtoverlay=disable-bt

# 2. Enable UART on CM4, GPIO 4/5 maps to UART3 (was UART2 on Pi 5)
dtoverlay=uart3

# 3. Enable UART on CM4, GPIO 8/9 maps to UART4 (was UART3 on Pi 5)
dtoverlay=uart4

# 4. Enable UART on CM4, GPIO 12/13 maps to UART5 (was UART4 on Pi 5)
dtoverlay=uart5

# 5. Disable audio to protect MAVLink
# GPIO 12/13 are default audio pins; this prevents audio drivers from interfering
dtparam=audio=off

# 6. General UART enable
enable_uart=1

# Custom power LED
dtparam=act_led_gpio=6
dtparam=act_led_trigger=mmc0
dtparam=act_led_activelow=on
$CFG_END
EOF

sudo cp "$BOOT_CONFIG" "${BOOT_CONFIG}.bak.navisar"
sudo cp "$tmp_cfg" "$BOOT_CONFIG"
rm -f "$tmp_cfg"

echo "[2/6] Installing system packages..."
sudo apt update
sudo apt install -y \
  libcamera-apps \
  python3-libcamera \
  python3-picamera2 \
  python3-venv

echo "[3/6] Creating virtual environment (if missing)..."
if [ ! -d "$VENV_DIR" ]; then
  python3 -m venv --system-site-packages "$VENV_DIR"
fi

if [ -x "$VENV_DIR/bin/python" ]; then
  PYTHON_BIN="$VENV_DIR/bin/python"
fi
if [ -x "$VENV_DIR/bin/pip" ]; then
  PIP_BIN="$VENV_DIR/bin/pip"
fi

echo "[4/6] Installing Python dependencies..."
"$PIP_BIN" install --upgrade pip
"$PIP_BIN" install -r "$ROOT_DIR/requirements.txt"
"$PIP_BIN" install --upgrade --force-reinstall "numpy<2" "opencv-python<4.11"
"$PIP_BIN" install smbus2

echo "[5/6] Making runner script executable..."
chmod +x "$ROOT_DIR/scripts/start_navisar.sh"

echo "[6/6] Installing and starting autostart services..."
bash "$ROOT_DIR/scripts/install_autostart_service.sh"

echo "Done. Verify with:"
echo "  sudo systemctl status navisar.service"
echo "Reboot required for camera/UART overlay changes:"
echo "  sudo reboot"
