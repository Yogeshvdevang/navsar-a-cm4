# NAVISAR-A Quick Guide

This guide is a practical, end-to-end reference for running the project, switching modes, and opening the HTML GUI.

## 1) Setup (one-time)

If you already installed dependencies, skip to **Run**.

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## 2) Run

Run the main pipeline (loads configs from `config/`):

```bash
PYTHONPATH=src python -m navisar.main
```

## 3) Open the GUI

After the program starts, open:

```
http://127.0.0.1:8765/gui.html
```

The data endpoint used by the GUI is:

```
http://127.0.0.1:8765/data
```

## 4) Modes (output_mode)

Mode is configured in `config/pixhawk.yaml`:

```yaml
output_mode: optical_flow_gps_port
```

Supported values:
- `optical_flow_gps_port` (optical flow -> GPS output to Pixhawk)
- `optical_flow_mavlink` (optical flow -> MAVLink OPTICAL_FLOW_RAD)
- `gps_mavlink` (VO -> GPS_INPUT)
- `gps_port` (VO -> serial GPS output)
- `odometry` (VO -> MAVLink ODOMETRY)
- `optical_flow_then_vo` (auto switch under/over distance threshold)

After changing `output_mode`, **restart** the program.

## 5) Auto mode (8m switch)

Configure the auto mode in `config/pixhawk.yaml`:

```yaml
output_mode: optical_flow_then_vo

optical_flow_vo:
  switch_m: 8.0
  optical_flow_output_mode: optical_flow_gps_port
  vo_output_mode: gps_mavlink
```

## 6) Optical Flow Port

Configured in `config/pixhawk.yaml`:

```yaml
optical_flow:
  enabled: true
  port: /dev/ttyAMA3
  baud: 115200
```

## 7) Pixhawk / MAVLink Port

Configured in `config/pixhawk.yaml`:

```yaml
device: /dev/ttyAMA0
baud: 115200
```

## 8) HTML Mode Switching (Live Buttons)

The GUI supports live mode switching via a local endpoint:

```
POST http://127.0.0.1:8765/mode
```

The GUI buttons call this endpoint.  
If you **start in optical-only mode**, VO buttons are disabled because the VO pipeline is not running.

## 9) UBX / NMEA Output

Configured in `config/pixhawk.yaml`:

```yaml
gps_output:
  enabled: true
  format: ubx   # or nmea
  port: /dev/ttyAMA0
  baud: 230400
  rate_hz: 10
```

## 10) Troubleshooting

- **GUI blank**: ensure the program is running and `http://127.0.0.1:8765/data` returns JSON.
- **No optical flow plot**: check `optical_flow.enabled: true` and port is correct.
- **VO mode not switching**: start with a VO-capable mode (`gps_mavlink`, `odometry`, or `optical_flow_then_vo`).

## 11) Key Files

- `config/pixhawk.yaml` – runtime settings (ports, modes, GPS output)
- `simulation/gui.html` – custom GUI
- `src/navisar/main.py` – main pipeline and dashboard server

## Appendix: `config.txt` (CM4 UART / Camera)

Add these lines to your Pi config:

```
sudo nano /boot/firmware/config.tx
```
[all]

# --- VO Drone CM4 UART Config ---

# 1. Enable UART for Primary GPS (GPIO 14/15)
# Disables Bluetooth to free up the high-quality PL011 UART for GPS
dtoverlay=disable-bt

# 2. Enable UART for Pixhawk (GPIO 4/5)
# On CM4, GPIO 4/5 maps to UART3 (was UART2 on Pi 5)
dtoverlay=uart3

# 3. Enable UART for Optical Flow (GPIO 8/9)
# On CM4, GPIO 8/9 maps to UART4 (was UART3 on Pi 5)
dtoverlay=uart4

# 4. Enable UART for Mavlink (GPIO 12/13)
# On CM4, GPIO 12/13 maps to UART5 (was UART4 on Pi 5)
dtoverlay=uart5

# 5. Disable Audio to protect Mavlink
# GPIO 12/13 are default audio pins; this prevents audio drivers from interfering
dtparam=audio=off

# 6. General UART Enable (Required for serial console/primary UART)
enable_uart=1
dtoverlay=ov9281,cam0
dtparam=act_led_gpio=6
dtparam=act_led_trigger=mmc0
dtparam=act_led_activelow=on
```
