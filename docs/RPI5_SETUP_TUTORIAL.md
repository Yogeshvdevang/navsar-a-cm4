# NAVISAR Raspberry Pi 5 Setup Tutorial

This guide is for bringing up this project on a fresh Raspberry Pi 5 from zero to a working dashboard and autostart service.

It is written against the current repository scripts:

- `scripts/bootstrap_rpi_autostart.sh`
- `scripts/install_autostart_service.sh`
- `scripts/start_navisar.sh`

## 1. What This Setup Will Do

By the end of this tutorial, your Raspberry Pi 5 will be able to:

- run the NAVISAR pipeline
- expose the dashboard on port `8765`
- start NAVISAR automatically on boot

## 2. Hardware and Software Assumptions

This tutorial assumes:

- Raspberry Pi 5
- Raspberry Pi OS installed and booting normally
- internet access on the Pi for package installation
- camera connected and supported by the project
- Pixhawk connected if you want MAVLink mode
- optional GPS and optical-flow hardware connected to the UARTs used by this repo

Current repo defaults expect these ports in [`config/pixhawk.yaml`](/home/psyc2/navsar-a-cm4/config/pixhawk.yaml):

- Pixhawk MAVLink: `/dev/ttyACM0`
- Optical flow input: `/dev/ttyAMA3`
- GPS input: `/dev/ttyAMA5`
- GPS output to flight controller: `/dev/ttyAMA0`

Current camera default in [`config/camera.yaml`](/home/psyc2/navsar-a-cm4/config/camera.yaml) is:

- `model: ov5647`

If your hardware differs, the project can still work, but you must update the YAML config before expecting it to run correctly.

## 3. Recommended OS Prep

On the new Pi 5, start with a normal terminal session and run:

```bash
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv
```

If you want to confirm the machine identity and IP:

```bash
hostname
hostname -I
```

## 4. Clone the Repository

Choose one permanent location and keep the repo there. This matters because the systemd service stores the absolute project path.

Recommended:

```bash
cd /home/$USER
git clone <YOUR-REPO-URL> navsar-a-cm4
cd navsar-a-cm4
```

Important:

- do not keep multiple clones unless you really need them
- if you later move the folder, reinstall the services
- if you already have an older clone elsewhere, stop its services before using the new one

## 5. Choose Your Setup Method

You have two valid ways to install the project:

1. Recommended: run the bootstrap script and let the repo configure everything
2. Manual: install dependencies and services step by step

For most new Pi 5 setups, use the bootstrap method.

## 6. Fastest Setup: Bootstrap Script

From the repo root:

```bash
bash scripts/bootstrap_rpi_autostart.sh
```

This script currently does all of the following:

- updates `/boot/firmware/config.txt` or `/boot/config.txt`
- installs camera and Python system packages
- creates `venv/` with `--system-site-packages`
- installs `requirements.txt`
- reinstalls pinned NumPy and OpenCV versions
- installs `smbus2`
- marks the runner script executable
- installs and starts `navisar.service`

After the script finishes:

```bash
sudo reboot
```

The reboot is required so camera and UART boot-overlay changes actually take effect.

## 7. What the Bootstrap Script Changes

The bootstrap script appends a managed block into the Pi boot config. The current repo version enables:

- camera overlay
- UART overlays
- `enable_uart=1`
- audio disable on the UART-sharing pins
- activity LED settings

The current script writes an `ov5647` camera overlay. If your Pi 5 is using `ov9281` instead, change both of these after installation:

1. boot config overlay
2. [`config/camera.yaml`](/home/psyc2/navsar-a-cm4/config/camera.yaml)

Example camera config values supported by the repo:

- `opencv`
- `ov9281`
- `ov5647`

## 8. Manual Setup Path

If you do not want to use the bootstrap script, do this instead.

### 8.1 Install system packages

```bash
sudo apt update
sudo apt install -y \
  libcamera-apps \
  python3-libcamera \
  python3-picamera2 \
  python3-venv
```

### 8.2 Create the Python environment

```bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
pip install --upgrade --force-reinstall "numpy<2" "opencv-python<4.11"
pip install smbus2
```

### 8.3 Install the autostart services

```bash
bash scripts/install_autostart_service.sh
```

### 8.4 Reboot

```bash
sudo reboot
```

## 9. First Configuration Review After Clone

Before testing on the new Pi 5, review these files:

- [`config/camera.yaml`](/home/psyc2/navsar-a-cm4/config/camera.yaml)
- [`config/pixhawk.yaml`](/home/psyc2/navsar-a-cm4/config/pixhawk.yaml)

At minimum, verify:

- camera model matches your real camera
- camera width and height are okay for your sensor
- Pixhawk device path is correct
- GPS input port is correct
- optical-flow port is correct
- GPS output port is correct
- desired `output_mode` is selected

The current default `output_mode` is `optical_flow_mavlink`.

## 10. Service Names and What They Do

This repo installs one service:

### `navisar.service`

This is the main NAVISAR runtime. It launches [`scripts/start_navisar.sh`](/home/psyc2/navsar-a-cm4/scripts/start_navisar.sh), which runs:

```bash
PYTHONPATH="$ROOT_DIR/src" python -m navisar.main
```

It also uses a lock file:

- `.navisar.lock`

That prevents multiple copies of the main pipeline from running at the same time from the same repo.

## 11. How to Check the Services

After reboot, verify the service:

```bash
sudo systemctl status navisar.service
```

To confirm they are enabled on boot:

```bash
sudo systemctl is-enabled navisar.service
```

To watch live logs:

```bash
sudo journalctl -u navisar.service -f
```

## 12. How to Open the UI

Once the services are up, open these URLs in a browser.

From the Pi itself:

- `http://127.0.0.1:8765/`
- `http://127.0.0.1:8765/gui.html`

From another device on the same network:

- `http://<PI_IP>:8765/`
- `http://<PI_IP>:8765/gui.html`

To find the Pi IP:

```bash
hostname -I
```

## 13. How to Start and Stop the Project

To stop the main NAVISAR runtime:

```bash
sudo systemctl stop navisar.service
```

To start it again:

```bash
sudo systemctl start navisar.service
```

To disable autostart completely:

```bash
sudo systemctl disable navisar.service
```

To restart after changing config:

```bash
sudo systemctl restart navisar.service
```

## 14. How to Confirm What Is Actually Running

These commands are very useful on a new Pi:

```bash
ps -ef | grep -i navisar | grep -v grep
ss -ltnp | grep 8765
```

Expected behavior:

- `8765` is the dashboard/server from `navisar.main`

## 15. How to Run Manually Without systemd

If you want to test the project before enabling autostart:

```bash
cd /home/$USER/navsar-a-cm4
source venv/bin/activate
PYTHONPATH=src python -m navisar.main
```

Manual runs are useful for:

- first boot testing
- checking camera errors directly in the terminal
- confirming config changes before restarting the service

Stop a manual foreground run with:

```bash
Ctrl+C
```

## 16. Common Hardware Checks

### Check USB Pixhawk device

```bash
ls /dev/ttyACM*
```

### Check UART devices

```bash
ls /dev/ttyAMA*
```

### Check camera stack

```bash
libcamera-hello --list-cameras
```

If the camera is not detected there, NAVISAR will usually fail too.

## 17. When You Need to Change Ports

If the new Pi 5 wiring does not match the repo defaults, edit [`config/pixhawk.yaml`](/home/psyc2/navsar-a-cm4/config/pixhawk.yaml).

Most common fields to adjust:

- `device`
- `gps_input.port`
- `optical_flow.port`
- `gps_output.port`
- `gps_passthrough.input_port`
- `gps_passthrough.output_port`

Then restart:

```bash
sudo systemctl restart navisar.service
```

## 18. When You Need to Change Camera Type

If your new Pi 5 is using a different camera than the current repo default:

1. edit [`config/camera.yaml`](/home/psyc2/navsar-a-cm4/config/camera.yaml)
2. update the camera overlay in `/boot/firmware/config.txt`
3. reboot

Then check:

```bash
libcamera-hello --list-cameras
sudo systemctl restart navisar.service
sudo journalctl -u navisar.service -n 100 --no-pager
```

## 19. If the Service Fails to Start

Use this sequence:

```bash
sudo systemctl status navisar.service
sudo journalctl -u navisar.service -n 200 --no-pager
```

Most common causes are:

- wrong camera model in config
- wrong serial port names
- hardware not connected
- boot overlays not applied because the Pi was not rebooted
- repo moved after service installation

## 20. If You Move the Repo Folder Later

Because the systemd service stores the repo path, moving the folder can break startup.

If you move the project, do this from the new repo location:

```bash
bash scripts/install_autostart_service.sh
sudo systemctl restart navisar.service
```

## 21. Recommended Bring-Up Checklist for a Brand-New Pi 5

Use this order:

1. Flash Raspberry Pi OS and complete first boot
2. Install `git`, `python3`, and `python3-venv`
3. Clone this repo into its permanent folder
4. Run `bash scripts/bootstrap_rpi_autostart.sh`
5. Reboot
6. Verify `navisar.service`
7. Open `http://<PI_IP>:8765/gui.html`
8. If hardware is different, update YAML config and restart the service

## 22. Short Version

If your Pi 5 hardware matches the current repo defaults, this is the shortest reliable install flow:

```bash
cd /home/$USER
git clone <YOUR-REPO-URL> navsar-a-cm4
cd navsar-a-cm4
bash scripts/bootstrap_rpi_autostart.sh
sudo reboot
```

After reboot:

```bash
sudo systemctl status navisar.service
hostname -I
```

Then open:

- `http://<PI_IP>:8765/gui.html`
