# Visual Odometry Drone

This repository implements a monocular visual odometry pipeline for a drone and integrates with a Pixhawk via MAVLink. The core flow reads camera frames, tracks features, estimates pixel motion, converts that to metric motion using altitude (LiDAR or fallback), and fuses it with GPS when available. It can also stream odometry and synthetic GPS back to the flight controller.
```bash
NAVISAR_DASHBOARD_OPEN=0 PYTHONPATH=src python -m navisar.main
http://127.0.0.1:8765
http://127.0.0.1:8765/gui.html

```

## What is implemented

- Monocular VO with feature tracking, RANSAC gating, and motion smoothing.
- Optional LiDAR altitude via MAVLink.
- GPS/odometry source selection with drift monitoring.
- MAVLink telemetry send/receive for attitude, GPS, and odometry.
- Live OpenCV visualization of tracked features and motion.

## Quick start

1) Create a Python environment and install requirements.

```bash
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

2) Run visual odometry (camera required).

```bash
PYTHONPATH=src python -m navisar.main
```

Windows PowerShell:

```powershell
$env:PYTHONPATH = "src"
python -m navisar.main
```

3) Optional: monitor raw Pixhawk MAVLink data.

```bash
PYTHONPATH=src python tools/mavlink_sniffer.py
```

Note: set `PYTHONPATH=src` for any direct `tools/` or `scripts/` runs.


## Dashboard

Open the dashboard in a browser at `http://127.0.0.1:8765/`.
Auto-open is disabled by default to avoid launching terminal browsers in headless shells.
Set `NAVISAR_DASHBOARD_OPEN=1` to auto-open in a browser.

parameters
```bash
sudo nano /boot/firmware/config.txt
```

```bash
dtoverlay=ov5647
dtoverlay=ov5647,cam0

dtparam=fan_temp0=60000
dtparam=fan_temp1=70000


dtparam=fan_temp2=80000
dtparam=fan_temp3=85000

dtparam=uart0=on
enable_uart=1
dtoverlay=uart3  # GPIO4/5 (optical flow)
dtoverlay=uart5  # GPIO12/13 (GPS input)
```

UART mapping used in this setup:
- GPIO12/13 → `/dev/ttyAMA5` (GPS input to RPi)
- GPIO4/5 → `/dev/ttyAMA3` (optical flow to RPi)
- GPIO14/15 → `/dev/ttyAMA0` (GPS output to Pixhawk)
## RPi Picamera2 (OV9281/OV5647) setup

If you use a Picamera2-backed CSI camera (`model: ov9281` or `model: ov5647` in `config/camera.yaml`), install
Picamera2 and its system dependencies on the RPi:

```bash
sudo apt update
sudo apt install -y libcap-dev python3-picamera2
```

Then install the Python package in your venv:

```bash
pip install picamera2
```

## Fake GPS (NMEA) injection

Provide a home location in `data/home_locations/site_A.yaml`, then run:

```bash
PYTHONPATH=src python -m navisar.pixhawk.gps_injector --port /dev/ttyAMA0 --baud 115200 --rate 5
```

## VPS -> GPS over MAVLink

Set `output_mode: vps_gps` in `config/pixhawk.yaml` to send smoothed fake GPS to the Pixhawk via `GPS_INPUT`.
Tune `config/pixhawk.yaml` under `vps_gps:` for send rate and smoothing.

## RPi GPS input + Pixhawk NMEA output (dual USB-TTL)

If you wire a real GPS (NEO-7M) into the RPi and a separate USB-TTL into the Pixhawk GPS port,
configure `config/pixhawk.yaml`:

- `gps_input` for the NEO-7M (used to set home origin at startup)
- `gps_output` with `format: nmea` for the Pixhawk GPS port

### Wiring and ports

RPi side:
- NEO-7M GPS (TX/RX) → USB-TTL → RPi USB (`/dev/ttyUSB0`, 9600 baud)

Pixhawk side:
- RPi USB-TTL → Pixhawk GPS port (TX/RX, GND, 5V as needed) (`/dev/ttyUSB1`, 9600 baud)

### Setup checklist

1) Plug both USB-TTL adapters into the RPi.
2) Confirm device paths (`/dev/ttyUSB0`, `/dev/ttyUSB1`) with `ls /dev/ttyUSB*`.
3) Set `gps_input` and `gps_output` in `config/pixhawk.yaml`:

```yaml
gps_input:
  enabled: true
  port: /dev/ttyUSB0
  baud: 9600
  format: auto
  init_wait_s: 60
  min_fix_type: 3

gps_output:
  enabled: true
  format: nmea
  port: /dev/ttyUSB1
  baud: 9600
  rate_hz: 5
  fix_quality: 1
  min_sats: 14
  max_sats: 20
  update_s: 7
  print: true
```

4) Set `output_mode: vps_gps` to enable fake GPS output.
5) Run:

```bash
PYTHONPATH=src python -m navisar.main
```

## Output modes

`output_mode` is a manual choice:

- `odometry`: send MAVLink ODOMETRY only (VO as local position/velocity).
- `vps_gps`: send fake GPS only (MAVLink GPS_INPUT or NMEA depending on `gps_output`).
- `gps_serial`: read GPS serial for monitoring/debug only.
- `reserved`: legacy; avoid using unless you intend to customize switching logic.

## How it works (data flow)

1) Camera frames arrive from `src/navisar/sensors/camera.py`.
2) `src/navisar/vps/feature_tracking.py` detects features on a grid and tracks them with optical flow.
3) `src/navisar/vps/pose_estimator.py` estimates pixel translation using RANSAC and converts to meters using height.
4) `src/navisar/vps/height_estimator.py` pulls altitude from `src/navisar/sensors/lidar.py` (MAVLink DISTANCE_SENSOR) or fallback.
5) `src/navisar/vps/visual_odometry.py` integrates motion, applies gating (inliers, flow MAD, exposure, zero motion), and renders the UI.
6) `src/navisar/navigation/state_estimator.py` chooses between GPS and odometry based on drift and timeout.
7) `src/navisar/pixhawk/mavlink_client.py` sends odometry/GPS back to the Pixhawk and reads GPS/attitude for yaw compensation.d

## End-to-end runtime flow (step by step)

1) `python -m navisar.main` loads YAML configs (`config/camera.yaml`, `config/vio.yaml`, `config/pixhawk.yaml`).
2) `build_vo_pipeline()` instantiates:
   - A camera driver (OpenCV or OV9281).
   - `FeatureTracker` with grid and quality thresholds.
   - A motion estimator (RANSAC affine by default, median flow optional).
   - `PoseEstimator` with camera intrinsics.
   - `LidarHeightEstimator` (if MAVLink available) and a `HeightEstimator` wrapper.
3) If MAVLink is enabled:
   - Connects to Pixhawk and waits for heartbeat.
   - Requests ATTITUDE at `ATTITUDE_RATE_HZ` for yaw compensation.
4) The VO loop (`VisualOdometry.run`) continuously:
   - Reads a frame, converts to grayscale.
   - Optional undistortion if `dist_coeffs` are provided in `config/camera.yaml`.
   - Tracks features (optical flow), rejects outliers with RANSAC.
   - Converts pixel flow to meters using height.
   - Applies gating (min inliers, flow MAD, exposure, min height).
   - Integrates X/Y/Z and optionally renders a debug window.
5) The main loop:
   - Optionally reads real GPS (NMEA) to set a local origin or validate drift.
   - Chooses the active source (GPS vs odometry) using `PositionSourceSelector`.
   - Emits MAVLink ODOMETRY and/or GPS_INPUT depending on `output_mode`.
   - Optionally emits NMEA/UBX on a serial port for the Pixhawk GPS input.

## Outputs and where they go

- **MAVLink ODOMETRY**: sent via `MavlinkInterface.send_odometry()` when odometry is trusted.
- **MAVLink GPS_INPUT**: sent when `output_mode: vps_gps` is enabled.
- **Serial NMEA/UBX**: emitted via `NmeaSerialEmitter` / `UbxSerialEmitter` if `gps_output.enabled` is true.
- **Debug window**: OpenCV UI from `VisualOdometry.run()` shows tracked features and motion text.

## Camera calibration and distortion

If you see strong lens distortion, calibrate and supply real intrinsics:

1) Run the checkerboard calibration helper:
   ```bash
   PYTHONPATH=src python tools/camera_calibration.py --width 640 --height 400
   ```
2) Paste the printed `intrinsics` into `config/camera.yaml` (including `dist_coeffs`).
3) The VO pipeline will undistort frames before tracking.

## Configuration

YAML configs in `config/` now override defaults in `src/navisar/main.py`.

Key constants:
- Camera intrinsics: `FX`, `FY`, `CX`, `CY`, `IMG_WIDTH`, `IMG_HEIGHT`
- Feature tracking: `MIN_FEATURES`, `MAX_FEATURES`, `REDETECT_INTERVAL`, `GRID_ROWS`, `GRID_COLS`
- Motion gating: `METRIC_THRESHOLD`, `MIN_INLIERS`, `MIN_INLIER_RATIO`, `MAX_FLOW_MAD_PX`, `MIN_FLOW_PX`
- Zero motion detection: `ZERO_MOTION_WINDOW`, `ZERO_MOTION_MEAN_M`, `ZERO_MOTION_STD_M`
- MAVLink rates: `ATTITUDE_RATE_HZ`, `ODOMETRY_SEND_INTERVAL_S`, `ODOM_GPS_SEND_INTERVAL_S`

Environment variables:
- `MAVLINK_DEVICE` (default `/dev/ttyACM0`)
- `MAVLINK_BAUD` (default `115200`)
- `LIDAR_DISTANCE_DIVISOR` (default `100.0`)
- `GPS_ORIGIN_LAT`, `GPS_ORIGIN_LON`, `GPS_ORIGIN_ALT` (manual local origin)

Raw MAVLink monitor environment variables (used by `tools/mavlink_sniffer.py`):
- `MAVLINK_HEARTBEAT_TIMEOUT_S`
- `MAVLINK_PRINT_INTERVAL_S`
- `MAVLINK_GPS_RAW_RATE_HZ`
- `MAVLINK_GLOBAL_POS_RATE_HZ`
- `MAVLINK_LIDAR_RATE_HZ`

## Main runtime behavior

`src/navisar/main.py` wires the pipeline together:

- Initializes camera, feature tracker, pose estimator, height estimator (LiDAR if available).
- Requests ATTITUDE messages for yaw compensation.
- Tracks motion and integrates X/Y/Z in the VO frame.
- Prints GPS and LiDAR values (rate-limited).
- Chooses GPS or odometry as the active position source, based on drift and timeout.
- Sends MAVLink `GPS_INPUT` and `ODOMETRY` messages when odometry is trusted.

Coordinate notes:
- VO motion is integrated in a local ENU-like frame (`x` and `y` from optical flow).
- MAVLink odometry is sent in NED by swapping axes and negating Z.

## Repo layout

- `src/navisar/main.py`: end-to-end VO + LiDAR + MAVLink pipeline.
- `src/navisar/sensors/`: camera, IMU, LiDAR readers.
- `src/navisar/vps/`: feature tracking, pose estimation, visual odometry loop.
- `src/navisar/navigation/`: GPS/odometry source selection and local frame conversions.
- `src/navisar/core/`, `src/navisar/gnss_monitor/`, `src/navisar/pixhawk/`: placeholder modules for future expansion.
- `tests/`: unit/integration test placeholders and logs.
- `tools/mavlink_sniffer.py`: prints MAVLink GPS and LiDAR messages.
- `tools/xy_drift.py`: alternate VO runner for simple position drift logging.

## Known gaps and placeholders

Several directories (docs, config, src/navisar/core, src/navisar/gnss_monitor, src/navisar/pixhawk, tests) contain placeholder files only. They are reserved for future work and do not implement functionality yet.

`tools/xy_drift.py` now unpacks `build_vo_pipeline()` correctly; it expects the VO instance and ignores the MAVLink interface.

## Troubleshooting

- Camera not opening: verify `CAMERA_INDEX` and that no other app is holding the camera.
- No MAVLink data: check `MAVLINK_DEVICE`, baud rate, and Pixhawk connection.
- No LiDAR readings: confirm the MAVLink `DISTANCE_SENSOR` stream is enabled.
- Drifty VO: tune `MIN_INLIERS`, `MIN_INLIER_RATIO`, and `MAX_FLOW_MAD_PX`.
