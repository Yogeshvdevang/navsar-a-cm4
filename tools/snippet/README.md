# Odometry

A collection of standalone Python scripts for reading sensors and experimenting with optical-flow/visual-odometry pipelines. The focus is on quick bring-up and live telemetry from common robotics/autopilot stacks (MAVLink) and serial sensors (NMEA GPS, MicoLink optical-flow).

This repo is not a single package; each file is meant to be run directly and tuned for your hardware.

## What's inside

- Optical flow over serial (MicoLink) and MAVLink
- Visual odometry using OpenCV (USB camera or Raspberry Pi camera)
- MAVLink telemetry readers for IMU, barometer, distance sensor (LiDAR), compass
- NMEA GPS parsing utilities

## Requirements

- Python 3.8+ (3.10+ recommended)
- Serial access for your devices (Linux: make sure your user is in the `dialout` group)

Python packages (install as needed by the script you run):

- `pyserial`
- `pymavlink`
- `opencv-python`
- `numpy`
- `picamera2` (Raspberry Pi camera scripts)

Example setup:

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install pyserial pymavlink opencv-python numpy
```

On Raspberry Pi for `picamera2`, follow the system-specific install instructions for your OS image.

## Quick start

Pick the script that matches your hardware and run it directly.

### Optical flow (MicoLink)
Reads an MTF-01 style optical flow sensor over serial.

```bash
python3 of.py --protocol micolink --port /dev/ttyUSB0 --baud 115200
```

### Optical flow (MAVLink)
Listen to MAVLink optical flow messages from an autopilot.

```bash
python3 of.py --protocol mavlink --port /dev/ttyACM0 --baud 115200
```

### MTF-01 heartbeat + reader (MicoLink)
Sends heartbeats to keep the sensor active while decoding data.

```bash
python3 oftical_flow.py /dev/ttyUSB0 115200 100
```

Arguments are `PORT BAUDRATE FREQUENCY_HZ` (frequency `0` = maximum speed).

### Visual odometry with LiDAR height (USB camera)
Uses optical flow from a USB camera and scales motion by MAVLink `DISTANCE_SENSOR` height.

```bash
python3 vo.py
```

### Visual odometry (USB camera, different tuning)
Similar to `vo.py` but with different parameters and overlays.

```bash
python3 odo.py
```

### Visual odometry (Raspberry Pi camera)
Monocular VO with `picamera2` and essential matrix estimation.

```bash
python3 pi_vo_new.py
```

### IMU velocity integration (MAVLink)
Integrates IMU acceleration to estimate velocities and prints at a set rate.

```bash
python3 imu.py
```

Environment variables for `imu.py`:

- `MAVLINK_DEVICE` (default: `/dev/ttyACM0`)
- `MAVLINK_BAUD` (default: `115200`)
- `MAVLINK_HEARTBEAT_TIMEOUT_S` (default: `5.0`)
- `MAVLINK_IMU_RATE_HZ` (default: `50.0`)
- `MAVLINK_PRINT_INTERVAL_S` (default: `0.1`)
- `IMU_BIAS_CALIB_S` (default: `3.0`)
- `IMU_VEL_DAMPING` (default: `0.98`)

### Pixhawk compass (MAVLink)
Reads the Pixhawk built-in magnetometer from MAVLink `RAW_IMU`, `SCALED_IMU*`, and `HIGHRES_IMU`.

```bash
python3 pixhawk_compass_mavlink.py --device /dev/ttyACM0 --baud 115200
```

### Barometer (MAVLink)
Read pressure/temperature and optionally compute altitude.

```bash
python3 baro.py --port /dev/ttyACM0 --baud 115200 --compute-alt
```

### GPS (NMEA over serial)
Parse NMEA GGA/RMC sentences and print lat/lon/alt.

```bash
python3 gps.py
```

## Script map

Use this as a quick guide to what each file does.

- `of.py`: Optical flow reader that supports both MicoLink and MAVLink.
- `oftical_flow.py`: MTF-01 optical flow reader with heartbeat thread (MicoLink).
- `vo.py`: USB camera VO + MAVLink distance sensor height scaling + overlays.
- `odo.py`: USB camera VO + MAVLink distance sensor height scaling (alternate tuning).
- `pi_vo_new.py`: Raspberry Pi camera VO with essential matrix and pose recovery.
- `pi_vo.py`: Earlier Raspberry Pi VO variant.
- `cam.py`, `cam_wid.py`: Camera capture utilities / variants.
- `imu.py`: MAVLink IMU reader with simple bias calibration and velocity integration.
- `pixhawk_compass_mavlink.py`: standalone Pixhawk built-in compass reader over MAVLink.
- `baro.py`: MAVLink barometer reader with optional altitude conversion.
- `gps.py`: NMEA GPS parsing over serial.
- `gps_a2.py`, `gps_auto.py`, `gps_compass.py`, `mav_gps.py`, `read_gps.py`: GPS utilities and MAVLink/GPS parsing helpers.
- `fakeg.py`, `temp.py`: Local experiments or placeholders.
- `vo-new.py`: Alternative VO tuning/experiment.

If you are unsure, start with `of.py`, `vo.py`, `imu.py`, or `pixhawk_compass_mavlink.py` and then branch out.

## Configuration notes

- Serial ports and baud rates are hard-coded in some scripts. Update them near the top of each file to match your setup.
- MAVLink scripts assume an autopilot that is already streaming the relevant messages.
- Optical flow scaling in `vo.py`/`odo.py` depends on `FOCAL_LENGTH` and the height source. You will likely need to tune these for your camera.

## Troubleshooting

- Port busy or permission denied: close other tools and ensure your user is in `dialout` (or the equivalent group on your OS).
- No MAVLink heartbeat: verify the correct port/baud and that the autopilot is powered.
- No optical flow data: verify the sensor protocol, baud rate, and wiring; try `oftical_flow.py` if the device requires heartbeats.
- OpenCV window not showing: ensure you are running in a desktop session with GUI support.

## Safety and handling

These scripts talk to live hardware. If you are running them on a vehicle, keep it secured and be ready to cut power.

## License

No license file is included. Add one if you plan to distribute this code.
