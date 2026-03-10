# NAVSAR-A Raspberry Pi CM4 Setup

This guide is for Raspberry Pi CM4 and covers:
- Camera + UART + LED boot config
- Full manual installation steps
- One-command bootstrap installation

## 1) Edit Boot Config (`config.txt`)

Open terminal and run:

```bash
sudo nano /boot/firmware/config.txt
```

Add the following block at the end of the file:

```ini
[all]
# --- VO Drone CM4 UART Config ---

# Camera
dtoverlay=ov9281,cam0

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

# 6. General UART enable (required for serial console/primary UART)
enable_uart=1

# Custom LED for power
dtparam=act_led_gpio=6
dtparam=act_led_trigger=mmc0
dtparam=act_led_activelow=on
```

Save and exit nano.

## 2) Full Manual Setup

Run these commands:

```bash
# 1) Clone project
cd ~
git clone <OUR-REPO-URL> NAVSAR-A
cd NAVSAR-A

# 2) Install system deps
sudo apt update
sudo apt install -y libcamera-apps python3-libcamera python3-picamera2 python3-venv

# 3) Python env
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
pip install --upgrade --force-reinstall "numpy<2" "opencv-python<4.11"
pip install smbus2

# 4) Install autostart services
bash scripts/install_autostart_service.sh

# 5) Verify
sudo systemctl status navisar.service
sudo systemctl status navisar-control.service
```

## 3) One-Command Alternative (Recommended)

Instead of manual install steps above, run:

```bash
# 1) Clone project
cd ~
git clone <OUR-REPO-URL> NAVSAR-A
cd NAVSAR-A

# 2) Install everything + create autostart services
bash scripts/bootstrap_rpi_autostart.sh
```

`bootstrap_rpi_autostart.sh` does all of this:
- Updates boot `config.txt` with camera/UART/LED block
- Installs system dependencies
- Creates `venv` and installs Python packages
- Installs and starts `navisar.service` and `navisar-control.service`

## 4) Reboot

After either method, reboot once so boot overlay/UART changes are applied:

```bash
sudo reboot
```

## 5) Service Control Commands

Use these commands to manage `navisar.service`:

```bash
sudo systemctl stop navisar.service  # for STOP the service
sudo systemctl start navisar.service  # for START the service
sudo systemctl is-active navisar.service  # for checking its STATUS
```
