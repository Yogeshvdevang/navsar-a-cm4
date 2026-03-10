"""Entry point for the VO + barometer pipeline and MAVLink/GPS outputs."""

import os
import sys
import time
import math
import json
import csv
import re
import socket
import subprocess
import threading
import multiprocessing
import http.server
import webbrowser
import tempfile
import zipfile
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np
import cv2
from pymavlink import mavutil
import yaml


from navisar.sensors.camera import SharedCamera, create_camera_driver
from navisar.sensors.compass import (
    CompassReader,
    heading_from_milligauss,
    load_calibration_file,
)
from navisar.sensors.gps_serial import (
    GpsSerialReader,
    find_gps_port_and_baud,
    probe_nmea_on_port,
)
from navisar.sensors.barometer import BarometerHeightEstimator
from navisar.sensors.optical_flow import MTF01OpticalFlowReader
from navisar.pixhawk.gps_output import FakeGpsEmitter, NmeaSerialEmitter, UbxSerialEmitter
from navisar.pixhawk.mavlink_client import MavlinkInterface
from navisar.modes.gps_mavlink import GpsMavlinkMode
from navisar.modes.gps_port import GpsPortMode
from navisar.modes.gps_passthrough import GpsPassthroughMode
from navisar.modes.odometry import OdometryMode
from navisar.modes.optical_flow_gps_port import OpticalFlowGpsPortMode
from navisar.modes.optical_gps_port_imu import OpticalGpsPortImuMode
from navisar.modes.optical_flow_mavlink import OpticalFlowMavlinkMode
from navisar.navigation.state_estimator import PositionSourceSelector
from navisar.fusion.sensor_fusion import SensorFusion
from navisar.vps.feature_tracking import FeatureTracker
from navisar.vps.height_estimator import HeightEstimator
from navisar.vps.pose_estimator import PoseEstimator
from navisar.vps.algorithms.median_flow import MedianFlowEstimator
from navisar.vps.algorithms.ransac_affine import RansacAffineEstimator
from navisar.vps.visual_odometry import VisualOdometry
from navisar.vps.median_flow_vo import MedianFlowVO
from navisar.vps import vio_imu
from navisar.gnss_monitor.spoof_detector import SpoofDetector
from navisar.gnss_monitor.spoof_reporter import SpoofReporter, SpoofReportConfig
from navisar.vps.visual_slam import VisualSlam, SlamConfig
from navisar.vps.orbslam3_runner import OrbSlam3Runner, OrbSlam3Config

_CONFIG_WRITE_LOCK = threading.Lock()
_PLOTLY_JS_CACHE = {"bytes": None, "loaded": False}


def _get_plotly_js_bytes():
    """Return vendored Plotly JS bytes from the local Python environment."""
    if _PLOTLY_JS_CACHE["loaded"]:
        return _PLOTLY_JS_CACHE["bytes"]
    data = None
    try:
        import plotly  # local dependency, not network

        js_path = Path(plotly.__file__).resolve().parent / "package_data" / "plotly.min.js"
        if js_path.exists():
            data = js_path.read_bytes()
    except Exception:
        data = None
    _PLOTLY_JS_CACHE["bytes"] = data
    _PLOTLY_JS_CACHE["loaded"] = True
    return data

# ================= CONFIG =================
CAMERA_INDEX = 0
MIN_FEATURES = 40
MAX_FEATURES = 300
REDETECT_INTERVAL = 10  # frames
RANSAC_REPROJ_THRESH = 3.0
METRIC_THRESHOLD = 0.02  # meters
MIN_INLIERS = 50
GRID_ROWS = 6
GRID_COLS = 8
CELL_MAX_FEATURES = 30
CELL_TEXTURE_THRESHOLD = 12.0
CORNER_QUALITY_LEVEL = 0.2
MIN_FLOW_PX = 0.4
MIN_HEIGHT_M = 0.1
MIN_INLIER_RATIO = 0.5
MAX_FLOW_MAD_PX = 1.2
EXPOSURE_MIN_MEAN = 10.0
EXPOSURE_MAX_MEAN = 245.0
MOTION_CONFIRM_FRAMES = 3
MOTION_SMOOTH_WINDOW = 5
ZERO_MOTION_WINDOW = 8
ZERO_MOTION_MEAN_M = 0.004
ZERO_MOTION_STD_M = 0.002

# --- CAMERA INTRINSICS ---
IMG_WIDTH = 640
IMG_HEIGHT = 480
FX = 525.0
FY = 525.0
CX = IMG_WIDTH / 2.0
CY = IMG_HEIGHT / 2.0
K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], dtype=np.float64)
DIST_COEFFS = None

# --- SCALE (MONOCULAR) ---
USE_BAROMETER = True
ALTITUDE_M = 1.0

# --- MAVLINK ---
USE_MAVLINK = True
MAVLINK_DEVICE = os.getenv("MAVLINK_DEVICE", "/dev/ttyACM0")
MAVLINK_BAUD = int(os.getenv("MAVLINK_BAUD", "115200"))

# --- GPS/ODOMETRY SELECTION ---
GPS_DRIFT_THRESHOLD_M = 5.0
GPS_TIMEOUT_S = 2.0
GPS_MIN_FIX_TYPE = 3
ODOM_GPS_SEND_INTERVAL_S = 0.2
ODOM_GPS_FIX_TYPE = 3
ODOM_GPS_SATS = 10
ODOMETRY_SEND_INTERVAL_S = 0.04
ATTITUDE_RATE_HZ = 30.0
BARO_RATE_HZ = 25.0
IMU_RATE_HZ = 30.0
OUTPUT_MODE = "gps_mavlink" # odometry, gps_mavlink, gps_port, gps_passthrough, optical_flow_mavlink, optical_flow_gps_port, optical_gps_port_imu, optical_flow_then_vo
VIO_MODE = "vo"  # vo, vio_imu
FAKE_GPS_SMOOTH_ALPHA = 0.2
FAKE_GPS_MAX_STEP_M = 1.5
GPS_SERIAL_FORMAT = "auto"
GPS_OUTPUT_PORT = "/dev/ttyUSB1"
GPS_OUTPUT_BAUD = 9600
GPS_OUTPUT_RATE_HZ = 5.0
GPS_OUTPUT_FIX_QUALITY = 1
GPS_OUTPUT_MIN_SATS = 14
GPS_OUTPUT_MAX_SATS = 20
GPS_OUTPUT_UPDATE_S = 7.0
SLAM_ENABLED = False
SLAM_BACKEND = "opencv"
SLAM_CAMERA_INDEX = 1
SLAM_MAX_FEATURES = 800
SLAM_DRAW_MATCHES = 120
SLAM_MOTION_SCALE = 1.0
SLAM_TRAJECTORY_SIZE = 600
SLAM_TRAJECTORY_SCALE = 50.0
SLAM_FRAME_DELAY_S = 0.01
SLAM_WINDOW_NAME = "Visual SLAM"
SLAM_TRAJECTORY_WINDOW = "SLAM Trajectory"
SPOOF_MAX_SPEED_MPS = 25.0
SPOOF_CONSECUTIVE = 3
SPOOF_COOLDOWN_S = 2.0

# --- MANUAL GPS ORIGIN (optional) ---
GPS_ORIGIN_LAT = os.getenv("GPS_ORIGIN_LAT")
GPS_ORIGIN_LON = os.getenv("GPS_ORIGIN_LON")
GPS_ORIGIN_ALT = os.getenv("GPS_ORIGIN_ALT")

# --- SERIAL OUTPUT ---
PRINT_BARO_VALUES = True
PRINT_SENSOR_VALUES = True
PRINT_INTERVAL_S = 0.5

# --- OPTICAL FLOW ---
OPTICAL_FLOW_ENABLED = False
OPTICAL_FLOW_PORT = "/dev/ttyUSB0"
OPTICAL_FLOW_BAUD = 115200
OPTICAL_FLOW_RATE_HZ = 100.0
OPTICAL_FLOW_HEARTBEAT_S = 0.6
OPTICAL_FLOW_PRINT = False
OPTICAL_FLOW_MAV_SEND_INTERVAL_S = 0.05
OPTICAL_FLOW_MAV_PRINT = False
# --- DASHBOARD ---
DASHBOARD_ENABLED = os.getenv("NAVISAR_DASHBOARD_ENABLED", "1").lower() not in (
    "0",
    "false",
    "no",
)
DASHBOARD_HOST = os.getenv("NAVISAR_DASHBOARD_HOST", "0.0.0.0")
DASHBOARD_PORT = int(os.getenv("NAVISAR_DASHBOARD_PORT", "8765"))
DASHBOARD_OPEN_BROWSER = os.getenv("NAVISAR_DASHBOARD_OPEN", "0").lower() not in (
    "0",
    "false",
    "no",
)
NAVISAR_SERVICE_NAME = os.getenv("NAVISAR_SERVICE_NAME", "navisar.service")


def _can_auto_open_browser():
    """Return True only when a GUI browser is likely available."""
    if not DASHBOARD_OPEN_BROWSER:
        return False
    browser_pref = (os.getenv("BROWSER") or "").lower()
    if any(name in browser_pref for name in ("w3m", "lynx", "links", "elinks", "www-browser")):
        return False
    if os.name == "nt" or sys.platform == "darwin":
        return True
    return bool(os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY") or os.getenv("MIR_SOCKET"))


def _repo_root():
    """Return the repository root path."""
    return Path(__file__).resolve().parents[2]


def _discover_local_ipv4_addresses():
    """Return a sorted list of non-loopback local IPv4 addresses."""
    addresses = set()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            addresses.add(sock.getsockname()[0])
    except Exception:
        pass
    try:
        host_name = socket.gethostname()
        for info in socket.getaddrinfo(host_name, None, socket.AF_INET, socket.SOCK_STREAM):
            ip = info[4][0]
            if ip and not ip.startswith("127."):
                addresses.add(ip)
    except Exception:
        pass
    return sorted(addresses)


def _load_yaml(path):
    """Load a YAML file into a dict, defaulting to empty."""
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data


def _load_configs():
    """Load camera/VIO/pixhawk config files with fallback defaults."""
    root = _repo_root()
    config_dir = root / "config"
    # Config files override the constants above; missing files fall back to defaults.
    return {
        "camera": _load_yaml(config_dir / "camera.yaml"),
        "vio": _load_yaml(config_dir / "vio.yaml"),
        "pixhawk": _load_yaml(config_dir / "pixhawk.yaml"),
    }


def _persist_pixhawk_runtime_settings(
    pixhawk_config_path,
    mode_state,
    gps_format_state,
    altitude_offset_state,
):
    """Persist runtime dashboard selections into config/pixhawk.yaml."""
    path = Path(pixhawk_config_path)
    with _CONFIG_WRITE_LOCK:
        cfg = _load_yaml(path)
        if not isinstance(cfg, dict):
            cfg = {}
        mode = str(mode_state.get()).strip().lower()
        gps_format = _normalize_gps_format(gps_format_state.get())
        offset_m = float(altitude_offset_state.get())

        cfg["output_mode"] = mode
        gps_output_cfg = cfg.get("gps_output")
        if not isinstance(gps_output_cfg, dict):
            gps_output_cfg = {}
        gps_output_cfg["format"] = gps_format
        cfg["gps_output"] = gps_output_cfg
        cfg["altitude_offset_m"] = round(offset_m, 6)

        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(cfg, handle, sort_keys=False)
    return {
        "output_mode": mode,
        "gps_format": gps_format,
        "altitude_offset_m": round(offset_m, 6),
    }


def _persist_calibration_tuning(
    pixhawk_config_path,
    lat_scale,
    lon_scale,
    alt_offset_m,
    vo_scale,
    vo_lat_scale,
    vo_lon_scale,
):
    """Persist calibration tuning values into config/pixhawk.yaml."""
    path = Path(pixhawk_config_path)
    with _CONFIG_WRITE_LOCK:
        cfg = _load_yaml(path)
        if not isinstance(cfg, dict):
            cfg = {}
        calibration_cfg = cfg.get("calibration")
        if not isinstance(calibration_cfg, dict):
            calibration_cfg = {}
        tuning_cfg = calibration_cfg.get("optical_gps_tuning")
        if not isinstance(tuning_cfg, dict):
            tuning_cfg = {}

        tuning_cfg["lat_scale"] = round(float(lat_scale), 6)
        tuning_cfg["lon_scale"] = round(float(lon_scale), 6)
        tuning_cfg["alt_offset_m"] = round(float(alt_offset_m), 6)
        tuning_cfg["vo_scale"] = round(float(vo_scale), 6)
        tuning_cfg["vo_lat_scale"] = round(float(vo_lat_scale), 6)
        tuning_cfg["vo_lon_scale"] = round(float(vo_lon_scale), 6)
        calibration_cfg["optical_gps_tuning"] = tuning_cfg
        cfg["calibration"] = calibration_cfg

        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(cfg, handle, sort_keys=False)
    return {
        "lat_scale": tuning_cfg["lat_scale"],
        "lon_scale": tuning_cfg["lon_scale"],
        "alt_offset_m": tuning_cfg["alt_offset_m"],
        "vo_scale": tuning_cfg["vo_scale"],
        "vo_lat_scale": tuning_cfg["vo_lat_scale"],
        "vo_lon_scale": tuning_cfg["vo_lon_scale"],
    }


def _persist_optical_flow_scale_profiles(pixhawk_config_path, scale_state):
    """Persist optical-flow scale profiles into config/pixhawk.yaml."""
    payload = scale_state.snapshot()
    path = Path(pixhawk_config_path)
    with _CONFIG_WRITE_LOCK:
        cfg = _load_yaml(path)
        if not isinstance(cfg, dict):
            cfg = {}
        optical_cfg = cfg.get("optical_flow")
        if not isinstance(optical_cfg, dict):
            optical_cfg = {}
        optical_cfg["scale_profiles"] = payload["profiles"]
        optical_cfg["active_scale_profile"] = payload["active"]
        cfg["optical_flow"] = optical_cfg

        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(cfg, handle, sort_keys=False)
    return {
        "scale_profiles": payload["profiles"],
        "active_scale_profile": payload["active"],
    }


def _persist_gps_origin(pixhawk_config_path, lat, lon, alt_m):
    """Persist GPS origin into config/pixhawk.yaml."""
    path = Path(pixhawk_config_path)
    with _CONFIG_WRITE_LOCK:
        cfg = _load_yaml(path)
        if not isinstance(cfg, dict):
            cfg = {}
        cfg["gps_origin"] = {
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(alt_m) if alt_m is not None else None,
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(cfg, handle, sort_keys=False)
    return {
        "lat": float(lat),
        "lon": float(lon),
        "alt_m": float(alt_m) if alt_m is not None else None,
    }


def _run_service_command(action, service_name):
    """Run a systemd command for the NAVISAR service."""
    cmd = ["systemctl", action, service_name]
    try:
        proc = subprocess.run(
            ["sudo", "-n", *cmd],
            capture_output=True,
            text=True,
            check=False,
        )
    except Exception as exc:
        return False, str(exc)

    out = (proc.stdout or "").strip()
    err = (proc.stderr or "").strip()
    if proc.returncode == 0:
        return True, out or err
    if "a password is required" in (err or "").lower():
        return (
            False,
            "sudo passwordless rule is required for service control "
            "(see setup steps in documentation).",
        )
    return False, err or out or f"exit code {proc.returncode}"


def _safe_float(value):
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _sanitize_csv_key(value):
    text = str(value).strip()
    text = text.replace(" ", "_")
    text = re.sub(r"[^0-9a-zA-Z_]+", "_", text)
    text = re.sub(r"_+", "_", text).strip("_")
    return text or "value"


def _flatten_to_csv_row(value, prefix="", out=None):
    if out is None:
        out = {}
    if isinstance(value, dict):
        for k, v in value.items():
            key = _sanitize_csv_key(k)
            next_prefix = key if not prefix else f"{prefix}_{key}"
            _flatten_to_csv_row(v, next_prefix, out)
        return out
    if isinstance(value, (list, tuple)):
        if not value:
            out[prefix] = ""
            return out
        for i, item in enumerate(value):
            next_prefix = f"{prefix}_{i}"
            _flatten_to_csv_row(item, next_prefix, out)
        return out
    if isinstance(value, (float, int)):
        if isinstance(value, float) and not math.isfinite(value):
            out[prefix] = ""
        else:
            out[prefix] = value
        return out
    if isinstance(value, bool):
        out[prefix] = int(value)
        return out
    if value is None:
        out[prefix] = ""
        return out
    out[prefix] = value
    return out


def _build_sensor_csv_flat_payload(payload):
    flat = {}
    if not isinstance(payload, dict):
        return flat

    raw_sections = {
        "sensors",
        "compass",
        "raw",
    }

    for top_key, top_val in payload.items():
        top_key_safe = _sanitize_csv_key(top_key)
        if top_key_safe in raw_sections and isinstance(top_val, dict):
            if top_key_safe == "sensors":
                for sensor_name, sensor_payload in top_val.items():
                    sensor_key = _sanitize_csv_key(sensor_name)
                    _flatten_to_csv_row(sensor_payload, f"raw_{sensor_key}", flat)
            else:
                _flatten_to_csv_row(top_val, f"raw_{top_key_safe}", flat)
        else:
            _flatten_to_csv_row(top_val, f"syn_{top_key_safe}", flat)

    return _add_csv_aliases(flat)


def _add_csv_aliases(flat):
    """Add stable alias columns for commonly used CSV sensor/output groups."""
    if not isinstance(flat, dict):
        return flat

    def _copy_prefixed(src_prefix, dst_prefix):
        for key, value in list(flat.items()):
            if not key.startswith(src_prefix):
                continue
            alias_key = f"{dst_prefix}{key[len(src_prefix):]}"
            if alias_key not in flat:
                flat[alias_key] = value

    # Raw sensor aliases.
    _copy_prefixed("raw_gps_input_", "raw_gps_parameter_")
    _copy_prefixed("raw_barometer_", "raw_barometer_parameter_")
    _copy_prefixed("raw_imu_", "raw_imu_parameter_")
    _copy_prefixed("raw_attitude_", "raw_attitude_parameter_")
    _copy_prefixed("raw_optical_flow_", "raw_optical_flow_parameter_")
    _copy_prefixed("raw_compass_", "raw_compass_parameter_")
    _copy_prefixed("raw_raw_", "raw_camera_drift_parameter_")

    # Camera drift aliases used by dashboards/scripts.
    if "syn_camera_dx" in flat and "syn_camera_drift_x" not in flat:
        flat["syn_camera_drift_x"] = flat["syn_camera_dx"]
    if "syn_camera_dy" in flat and "syn_camera_drift_y" not in flat:
        flat["syn_camera_drift_y"] = flat["syn_camera_dy"]
    if "syn_camera_dz" in flat and "syn_camera_drift_z" not in flat:
        flat["syn_camera_drift_z"] = flat["syn_camera_dz"]

    # Final serialized output sent to Pixhawk GPS port.
    _copy_prefixed("syn_outputs_gps_port_", "syn_pixhawk_port_parameter_")
    if "syn_outputs_gps_port" in flat and "syn_pixhawk_port_parameter" not in flat:
        flat["syn_pixhawk_port_parameter"] = flat["syn_outputs_gps_port"]

    return flat


def _normalise_csv_value(value):
    if isinstance(value, (dict, list, tuple)):
        try:
            return json.dumps(value, separators=(",", ":"))
        except Exception:
            return ""
    return value


def _build_six_parameters_payload(payload, default_timestamp=None):
    """Normalize final GPS output payload into a strict six-parameter view."""
    if not isinstance(payload, dict):
        return None
    lat = _safe_float(payload.get("lat"))
    lon = _safe_float(payload.get("lon"))
    alt = _safe_float(payload.get("alt_m"))
    vel_n = _safe_float(payload.get("vel_n_mps"))
    if vel_n is None:
        vel_n = _safe_float(payload.get("vn"))
    vel_e = _safe_float(payload.get("vel_e_mps"))
    if vel_e is None:
        vel_e = _safe_float(payload.get("ve"))
    vel_d = _safe_float(payload.get("vel_d_mps"))
    if vel_d is None:
        vel_d = _safe_float(payload.get("vd"))

    heading = _safe_float(payload.get("heading_deg"))
    course = _safe_float(payload.get("course_deg"))
    fix_type = payload.get("fix_type")
    sats = payload.get("satellites")
    if sats is None:
        sats = payload.get("satellites_visible")

    ts = (
        payload.get("ubx_timestamp_utc")
        or payload.get("timestamp_utc")
        or payload.get("time_s")
        or default_timestamp
    )
    ubx_hex = payload.get("ubx_pvt_hex")
    if ubx_hex is None and isinstance(payload.get("ubx"), dict):
        ubx_hex = payload.get("ubx", {}).get("pvt_hex")
    if ubx_hex is None:
        ubx_hex = payload.get("raw_hex")

    return {
        "timestamp": ts,
        "latitude": lat,
        "longitude": lon,
        "altitude_m": alt,
        "velocity_north_mps": vel_n,
        "velocity_east_mps": vel_e,
        "velocity_down_mps": vel_d,
        "heading_deg": heading,
        "course_deg": course,
        "fix_type": fix_type,
        "satellites": sats,
        "hdop": _safe_float(payload.get("hdop")),
        "vdop": _safe_float(payload.get("vdop")),
        "pdop": _safe_float(payload.get("pdop")),
        "horizontal_accuracy_m": _safe_float(payload.get("horizontal_accuracy_m")),
        "vertical_accuracy_m": _safe_float(payload.get("vertical_accuracy_m")),
        "speed_accuracy_mps": _safe_float(payload.get("speed_accuracy_mps")),
        "ubx_message_hex": ubx_hex,
    }


def _capture_startup_baro_offset_max(barometer_driver, duration_s=3.0, poll_s=0.05):
    """Sample barometer for a short window and return the max observed height."""
    if barometer_driver is None:
        return None, 0
    duration_s = max(0.0, float(duration_s))
    poll_s = max(0.01, float(poll_s))
    deadline = time.time() + duration_s
    max_height_m = None
    sample_count = 0
    while time.time() < deadline:
        try:
            barometer_driver.update()
        except Exception:
            pass
        height_m = getattr(barometer_driver, "current_m", None)
        if height_m is None and hasattr(barometer_driver, "get_height_m"):
            try:
                height_m = barometer_driver.get_height_m()
            except Exception:
                height_m = None
        height_m = _safe_float(height_m)
        if height_m is not None and math.isfinite(height_m):
            sample_count += 1
            if max_height_m is None or height_m > max_height_m:
                max_height_m = height_m
        time.sleep(poll_s)
    return max_height_m, sample_count


def _vo_speed_accuracy(inlier_ratio, flow_mad_px):
    """Estimate speed accuracy (m/s) from VO quality metrics."""
    base = 0.5
    ratio = _safe_float(inlier_ratio)
    mad = _safe_float(flow_mad_px)
    if ratio is None or mad is None:
        return base
    ratio_penalty = max(0.0, 0.5 - ratio) * 4.0
    mad_penalty = max(0.0, mad - 1.0) * 0.6
    return max(0.2, min(3.0, base + ratio_penalty + mad_penalty))


def _heading_from_velocity(vx_e, vy_n, min_speed_mps=0.03):
    """Return ENU course heading degrees when speed exceeds threshold."""
    vx = _safe_float(vx_e)
    vy = _safe_float(vy_n)
    if vx is None or vy is None:
        return None
    speed = math.hypot(vx, vy)
    if speed < float(min_speed_mps):
        return None
    return (math.degrees(math.atan2(vx, vy)) + 360.0) % 360.0


def _smooth_heading_deg(prev_deg, new_deg, alpha, max_delta_deg=None):
    """Smooth heading with wrap-around handling."""
    if prev_deg is None:
        return new_deg
    if new_deg is None:
        return prev_deg
    alpha = max(0.0, min(1.0, float(alpha)))
    delta = (new_deg - prev_deg + 540.0) % 360.0 - 180.0
    if max_delta_deg is not None:
        max_delta = float(max_delta_deg)
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
    return (prev_deg + alpha * delta) % 360.0


class DashboardState:
    """Thread-safe storage for dashboard telemetry."""
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {
            "status": "starting",
            "timestamp": None,
        }

    def update(self, data):
        with self._lock:
            self._data.update(data)

    def snapshot(self):
        with self._lock:
            return dict(self._data)


class ModeState:
    """Thread-safe storage for the active output mode."""
    def __init__(self, mode):
        self._lock = threading.Lock()
        self._mode = mode

    def set(self, mode):
        with self._lock:
            self._mode = mode

    def get(self):
        with self._lock:
            return self._mode


class OpticalFlowScaleProfileState:
    """Thread-safe storage for optical-flow scale profile matrix."""

    FEATURE_MODES = ("high", "low")
    LIGHTING_MODES = ("high", "med", "low")
    ALTITUDE_BANDS = ("A", "B", "C")

    def __init__(self, profiles=None, active_profile=None, fallback_scale=1.0):
        self._lock = threading.Lock()
        self._fallback_scale = self._coerce_scale(fallback_scale)
        self._profiles = self._normalize_profiles(
            profiles if isinstance(profiles, dict) else None,
            self._fallback_scale,
        )
        self._active_profile = self._normalize_active_profile(active_profile)

    @staticmethod
    def _coerce_scale(value):
        try:
            scale = float(value)
        except (TypeError, ValueError):
            return 1.0
        if not (scale >= 0.0):
            return 0.0
        return max(0.0, min(20.0, scale))

    @classmethod
    def _normalize_profiles(cls, profiles, fallback_scale):
        normalized = {}
        for feature in cls.FEATURE_MODES:
            feature_data = {}
            src_feature = profiles.get(feature, {}) if isinstance(profiles, dict) else {}
            if not isinstance(src_feature, dict):
                src_feature = {}
            for lighting in cls.LIGHTING_MODES:
                feature_data[lighting] = {}
                src_lighting = src_feature.get(lighting, {})
                if not isinstance(src_lighting, dict):
                    src_lighting = {}
                for altitude_band in cls.ALTITUDE_BANDS:
                    feature_data[lighting][altitude_band] = cls._coerce_scale(
                        src_lighting.get(altitude_band, fallback_scale)
                    )
            normalized[feature] = feature_data
        return normalized

    @classmethod
    def _normalize_active_profile(cls, active_profile):
        profile = {
            "feature": cls.FEATURE_MODES[0],
            "lighting": cls.LIGHTING_MODES[0],
            "altitude": cls.ALTITUDE_BANDS[0],
        }
        if not isinstance(active_profile, dict):
            return profile
        feature = str(active_profile.get("feature", profile["feature"])).strip().lower()
        lighting = str(active_profile.get("lighting", profile["lighting"])).strip().lower()
        altitude = str(active_profile.get("altitude", profile["altitude"])).strip().upper()
        if feature in cls.FEATURE_MODES:
            profile["feature"] = feature
        if lighting in cls.LIGHTING_MODES:
            profile["lighting"] = lighting
        if altitude in cls.ALTITUDE_BANDS:
            profile["altitude"] = altitude
        return profile

    def snapshot(self):
        with self._lock:
            return {
                "profiles": {
                    feature: {
                        lighting: dict(altitudes)
                        for lighting, altitudes in feature_data.items()
                    }
                    for feature, feature_data in self._profiles.items()
                },
                "active": dict(self._active_profile),
                "current_scale": self._profiles.get(self._active_profile["feature"], {}).get(
                    self._active_profile["lighting"], {}
                ).get(
                    self._active_profile["altitude"], self._fallback_scale
                ),
                "options": {
                    "features": list(OpticalFlowScaleProfileState.FEATURE_MODES),
                    "lightings": list(OpticalFlowScaleProfileState.LIGHTING_MODES),
                    "altitudes": list(OpticalFlowScaleProfileState.ALTITUDE_BANDS),
                },
            }

    def set_profiles(self, profiles, active_profile=None):
        with self._lock:
            self._profiles = self._normalize_profiles(
                profiles if isinstance(profiles, dict) else None,
                self._fallback_scale,
            )
            if active_profile is not None:
                self._active_profile = self._normalize_active_profile(active_profile)

    def set_active(self, active_profile):
        with self._lock:
            self._active_profile = self._normalize_active_profile(active_profile)

    def set_fallback_scale(self, fallback_scale):
        with self._lock:
            self._fallback_scale = self._coerce_scale(fallback_scale)
            self._profiles = self._normalize_profiles(self._profiles, self._fallback_scale)

    def get_current_scale(self, fallback_scale=None):
        with self._lock:
            fallback = (
                self._coerce_scale(fallback_scale) if fallback_scale is not None else self._fallback_scale
            )
            return self._profiles.get(self._active_profile["feature"], {}).get(
                self._active_profile["lighting"], {}
            ).get(self._active_profile["altitude"], fallback)

    def set_profile_scale(self, feature, lighting, altitude, scale):
        feature_value = str(feature or "").strip().lower()
        lighting_value = str(lighting or "").strip().lower()
        altitude_value = str(altitude or "").strip().upper()
        normalized_scale = self._coerce_scale(scale)
        with self._lock:
            if feature_value not in self.FEATURE_MODES:
                return False
            if lighting_value not in self.LIGHTING_MODES:
                return False
            if altitude_value not in self.ALTITUDE_BANDS:
                return False
            self._profiles[feature_value][lighting_value][altitude_value] = normalized_scale
            return True

    def update_profiles(self, profiles):
        if not isinstance(profiles, dict):
            return False
        changed = False
        for feature in self.FEATURE_MODES:
            feature_block = profiles.get(feature, {}) if isinstance(profiles, dict) else {}
            if not isinstance(feature_block, dict):
                continue
            for lighting in self.LIGHTING_MODES:
                lighting_block = feature_block.get(lighting, {})
                if not isinstance(lighting_block, dict):
                    continue
                for altitude in self.ALTITUDE_BANDS:
                    if altitude not in lighting_block:
                        continue
                    try:
                        normalized_scale = self._coerce_scale(lighting_block.get(altitude))
                    except Exception:
                        continue
                    if self._profiles.get(feature, {}).get(lighting, {}).get(altitude) != normalized_scale:
                        self._profiles[feature][lighting][altitude] = normalized_scale
                        changed = True
        return changed


    @classmethod
    def from_config(cls, optical_flow_cfg, fallback_scale):
        if not isinstance(optical_flow_cfg, dict):
            optical_flow_cfg = {}
        return cls(
            profiles=optical_flow_cfg.get("scale_profiles"),
            active_profile=optical_flow_cfg.get("active_scale_profile"),
            fallback_scale=fallback_scale,
        )


class FrameState:
    """Thread-safe storage for latest JPEG frame."""
    def __init__(self):
        self._lock = threading.Lock()
        self._jpg = None
        self._timestamp = None

    def update(self, jpg_bytes):
        with self._lock:
            self._jpg = jpg_bytes
            self._timestamp = time.time()

    def snapshot(self):
        with self._lock:
            return self._jpg, self._timestamp


class BlackBoxRecorder:
    """Thread-safe black-box recorder for dashboard data and JPEG frames."""

    def __init__(self, root_dir):
        self._lock = threading.Lock()
        self._root_dir = Path(root_dir)
        self._root_dir.mkdir(parents=True, exist_ok=True)
        self._recording = False
        self._session_id = None
        self._session_dir = None
        self._data_path = None
        self._video_path = None
        self._meta_path = None
        self._data_file = None
        self._video_file = None
        self._started_at = None
        self._stopped_at = None
        self._data_count = 0
        self._frame_count = 0
        self._last_session_dir = None

    def _session_status_locked(self):
        return {
            "recording": self._recording,
            "session_id": self._session_id,
            "started_at": self._started_at,
            "stopped_at": self._stopped_at,
            "data_points": self._data_count,
            "frames": self._frame_count,
            "data_file": str(self._data_path) if self._data_path else None,
            "video_file": str(self._video_path) if self._video_path else None,
        }

    def _write_meta_locked(self):
        if not self._meta_path:
            return
        payload = self._session_status_locked()
        try:
            self._meta_path.write_text(
                json.dumps(payload, indent=2, sort_keys=True),
                encoding="utf-8",
            )
        except Exception:
            return

    def start(self):
        with self._lock:
            if self._recording:
                return self._session_status_locked()
            stamp = time.strftime("%Y%m%d_%H%M%S")
            base_id = f"flight_{stamp}"
            session_id = base_id
            idx = 1
            session_dir = self._root_dir / session_id
            while session_dir.exists():
                idx += 1
                session_id = f"{base_id}_{idx}"
                session_dir = self._root_dir / session_id
            session_dir.mkdir(parents=True, exist_ok=False)
            self._session_id = session_id
            self._session_dir = session_dir
            self._data_path = session_dir / "flight_data.txt"
            self._video_path = session_dir / "flight_video.mjpg"
            self._meta_path = session_dir / "session_meta.json"
            self._data_file = self._data_path.open("w", encoding="utf-8")
            self._video_file = self._video_path.open("wb")
            self._recording = True
            self._started_at = time.time()
            self._stopped_at = None
            self._data_count = 0
            self._frame_count = 0
            self._last_session_dir = session_dir
            self._write_meta_locked()
            return self._session_status_locked()

    def stop(self):
        with self._lock:
            if not self._recording:
                return self._session_status_locked()
            self._recording = False
            self._stopped_at = time.time()
            try:
                if self._data_file:
                    self._data_file.flush()
                    self._data_file.close()
            finally:
                self._data_file = None
            try:
                if self._video_file:
                    self._video_file.flush()
                    self._video_file.close()
            finally:
                self._video_file = None
            self._write_meta_locked()
            return self._session_status_locked()

    def status(self):
        with self._lock:
            return self._session_status_locked()

    def log_data(self, payload):
        with self._lock:
            if not self._recording or self._data_file is None:
                return
            row = {
                "server_ts": time.time(),
                "payload": payload,
            }
            try:
                self._data_file.write(json.dumps(row, separators=(",", ":")) + "\n")
                self._data_file.flush()
                self._data_count += 1
            except Exception:
                return
            self._write_meta_locked()

    def log_frame(self, jpg_bytes):
        if not jpg_bytes:
            return
        with self._lock:
            if not self._recording or self._video_file is None:
                return
            try:
                # MJPEG byte stream: concatenated JPEG frames.
                self._video_file.write(jpg_bytes)
                self._video_file.flush()
                self._frame_count += 1
            except Exception:
                return
            self._write_meta_locked()

    def build_download_zip(self):
        with self._lock:
            session_dir = self._session_dir if self._session_dir is not None else self._last_session_dir
            if session_dir is None or not session_dir.exists():
                return None, None
            data_path = session_dir / "flight_data.txt"
            video_path = session_dir / "flight_video.mjpg"
            meta_path = session_dir / "session_meta.json"
            if not data_path.exists() and not video_path.exists():
                return None, None
            session_id = self._session_id or session_dir.name
            tmp = tempfile.NamedTemporaryFile(
                prefix=f"{session_id}_",
                suffix=".zip",
                dir=str(self._root_dir),
                delete=False,
            )
            tmp_path = Path(tmp.name)
            tmp.close()
            with zipfile.ZipFile(tmp_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
                if data_path.exists():
                    zf.write(data_path, arcname=f"{session_dir.name}/flight_data.txt")
                if video_path.exists():
                    zf.write(video_path, arcname=f"{session_dir.name}/flight_video.mjpg")
                if meta_path.exists():
                    zf.write(meta_path, arcname=f"{session_dir.name}/session_meta.json")
            download_name = f"{session_dir.name}_blackbox.zip"
            return tmp_path, download_name


class SensorCsvRecorder:
    """Thread-safe CSV recorder for complete dashboard payload snapshots."""

    def __init__(self, root_dir):
        self._lock = threading.Lock()
        self._root_dir = Path(root_dir)
        self._root_dir.mkdir(parents=True, exist_ok=True)
        self._recording = False
        self._session_id = None
        self._session_dir = None
        self._csv_path = None
        self._meta_path = None
        self._csv_file = None
        self._csv_writer = None
        self._fieldnames = None
        self._started_at = None
        self._stopped_at = None
        self._row_count = 0
        self._last_session_dir = None

    def _session_status_locked(self):
        return {
            "recording": self._recording,
            "session_id": self._session_id,
            "started_at": self._started_at,
            "stopped_at": self._stopped_at,
            "data_points": self._row_count,
            "csv_file": str(self._csv_path) if self._csv_path else None,
        }

    def start(self):
        with self._lock:
            if self._recording:
                return self._session_status_locked()
            stamp = time.strftime("%Y%m%d_%H%M%S")
            base_id = f"sensor_csv_{stamp}"
            session_id = base_id
            idx = 1
            session_dir = self._root_dir / session_id
            while session_dir.exists():
                idx += 1
                session_id = f"{base_id}_{idx}"
                session_dir = self._root_dir / session_id
            session_dir.mkdir(parents=True, exist_ok=False)
            self._session_id = session_id
            self._session_dir = session_dir
            self._csv_path = session_dir / "sensor_data.csv"
            self._meta_path = session_dir / "session_meta.json"
            self._csv_file = self._csv_path.open("w", encoding="utf-8", newline="")
            self._csv_writer = None
            self._fieldnames = None
            self._recording = True
            self._started_at = time.time()
            self._stopped_at = None
            self._row_count = 0
            self._last_session_dir = session_dir
            return self._session_status_locked()

    def stop(self):
        with self._lock:
            if not self._recording:
                return self._session_status_locked()
            self._recording = False
            self._stopped_at = time.time()
            try:
                if self._csv_writer:
                    self._csv_file.flush()
            except Exception:
                pass
            try:
                if self._csv_file:
                    self._csv_file.flush()
                    self._csv_file.close()
            finally:
                self._csv_file = None
                self._csv_writer = None
            self._write_meta_locked()
            return self._session_status_locked()

    def status(self):
        with self._lock:
            return self._session_status_locked()

    def _write_meta_locked(self):
        if not self._meta_path:
            return
        payload = self._session_status_locked()
        try:
            self._meta_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        except Exception:
            return

    def log_data(self, payload):
        with self._lock:
            if not self._recording or self._csv_file is None:
                return
            payload = payload if isinstance(payload, dict) else {}
            flat = _build_sensor_csv_flat_payload(payload)
            flat = {k: _normalise_csv_value(v) for k, v in flat.items()}
            base = {
                "server_ts": time.time(),
                "server_timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "payload_timestamp": payload.get("timestamp"),
            }
            row = {**base, **flat}
            row["payload_json"] = json.dumps(payload, separators=(",", ":"))
            if self._fieldnames is None:
                self._fieldnames = sorted(row.keys())
                if "payload_json" not in self._fieldnames:
                    self._fieldnames.append("payload_json")
                if self._fieldnames and self._fieldnames[-1] != "payload_json":
                    self._fieldnames = [name for name in self._fieldnames if name != "payload_json"]
                    self._fieldnames.append("payload_json")
                self._fieldnames = list(self._fieldnames)
                try:
                    self._csv_writer = csv.DictWriter(
                        self._csv_file,
                        fieldnames=self._fieldnames,
                        extrasaction="ignore",
                    )
                    self._csv_writer.writeheader()
                except Exception:
                    return
            try:
                self._csv_writer.writerow(row)
                self._csv_file.flush()
                self._row_count += 1
            except Exception:
                return
            self._write_meta_locked()

    def download_csv(self):
        with self._lock:
            session_dir = self._session_dir if self._session_dir is not None else self._last_session_dir
            if session_dir is None or not session_dir.exists():
                return None, None
            csv_path = session_dir / "sensor_data.csv"
            if not csv_path.exists():
                return None, None
            return csv_path, f"{session_dir.name}_sensor_data.csv"


def _normalize_gps_format(value):
    fmt = str(value or "").strip().lower().replace("+", "_")
    if fmt == "nmea_ubx":
        return "ubx_nmea"
    if fmt in {"ubx", "nmea", "ubx_nmea"}:
        return fmt
    return "nmea"


def _make_dashboard_handler(
    state,
    mode_state,
    gps_format_state,
    altitude_offset_state,
    get_gps_origin,
    set_gps_origin,
    frame_state,
    blackbox_recorder,
    sensor_csv_recorder,
    root_dir,
    allowed_modes,
    allowed_gps_formats,
    pixhawk_config_path,
    calibration_enabled,
    calibration_gps_graph_enabled,
    calibration_tuning_state,
    calibration_tuning_defaults,
    optical_flow_scale_state,
):
    class DashboardHandler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(root_dir), **kwargs)

        def do_GET(self):
            parsed = urlparse(self.path)
            req_path = parsed.path
            query = parse_qs(parsed.query or "")
            if req_path in {"/plotly.min.js", "/plotly-2.35.2.min.js"}:
                js_data = _get_plotly_js_bytes()
                if js_data is None:
                    self.send_error(404, "Local Plotly bundle unavailable")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "application/javascript")
                self.send_header("Cache-Control", "public, max-age=3600")
                self.send_header("Content-Length", str(len(js_data)))
                self.end_headers()
                self.wfile.write(js_data)
                return
            if self.path in ("/", "/index.html"):
                self.path = "/gui.html"
            if req_path.startswith("/calibration-data"):
                if not calibration_enabled:
                    data = json.dumps(
                        {
                            "enabled": False,
                            "error": "Calibration UI is disabled in config/pixhawk.yaml "
                            "(set calibration.enabled: true).",
                        }
                    ).encode("utf-8")
                    self.send_response(403)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                snap = state.snapshot()
                sensors = snap.get("sensors", {}) if isinstance(snap, dict) else {}
                gps_input = sensors.get("gps_input", {}) if isinstance(sensors, dict) else {}
                optical = sensors.get("optical_flow", {}) if isinstance(sensors, dict) else {}
                gps_fused = snap.get("gps_ll_from_fused", {}) if isinstance(snap, dict) else {}
                gps_root = snap.get("gps", {}) if isinstance(snap, dict) else {}
                gps_origin = gps_root.get("origin", {}) if isinstance(gps_root, dict) else {}
                outputs = snap.get("outputs", {}) if isinstance(snap, dict) else {}
                if not isinstance(outputs, dict):
                    outputs = {}
                gps_port_out = outputs.get("gps_port") if isinstance(outputs.get("gps_port"), dict) else {}
                of_gps_port_out = (
                    outputs.get("optical_flow_gps_port")
                    if isinstance(outputs.get("optical_flow_gps_port"), dict)
                    else {}
                )
                of_gps_port_imu_out = (
                    outputs.get("optical_gps_port_imu")
                    if isinstance(outputs.get("optical_gps_port_imu"), dict)
                    else {}
                )
                payload = {
                    "enabled": True,
                    "gps_graph_enabled": bool(calibration_gps_graph_enabled),
                    "gps_tuning": {
                        "lat_scale": float(calibration_tuning_state["lat_scale"].get()),
                        "lon_scale": float(calibration_tuning_state["lon_scale"].get()),
                        "alt_offset_m": float(calibration_tuning_state["alt_offset_m"].get()),
                        "vo_scale": float(calibration_tuning_state["vo_scale"].get()),
                        "vo_lat_scale": float(calibration_tuning_state["vo_lat_scale"].get()),
                        "vo_lon_scale": float(calibration_tuning_state["vo_lon_scale"].get()),
                    },
                    "timestamp": snap.get("timestamp") if isinstance(snap, dict) else None,
                    "url": snap.get("url") if isinstance(snap, dict) else None,
                    "urls": snap.get("urls") if isinstance(snap, dict) else [],
                    "gps_input": {
                        "lat": _safe_float(gps_input.get("lat")),
                        "lon": _safe_float(gps_input.get("lon")),
                        "alt_m": _safe_float(gps_input.get("alt_m")),
                        "fix_type": _safe_float(gps_input.get("fix_type")),
                    },
                    "gps_fused": {
                        "lat": _safe_float(gps_fused.get("lat")),
                        "lon": _safe_float(gps_fused.get("lon")),
                        "alt_m": _safe_float(gps_fused.get("alt_m")),
                    },
                    "gps_origin": {
                        "lat": _safe_float(gps_origin.get("lat")),
                        "lon": _safe_float(gps_origin.get("lon")),
                        "alt_m": _safe_float(gps_origin.get("alt_m")),
                    },
                    "mode": snap.get("mode") if isinstance(snap, dict) else None,
                    "outputs": {
                        "gps_port": {
                            "lat": _safe_float(gps_port_out.get("lat")),
                            "lon": _safe_float(gps_port_out.get("lon")),
                            "alt_m": _safe_float(gps_port_out.get("alt_m")),
                            "heading_deg": _safe_float(gps_port_out.get("heading_deg")),
                        },
                        "optical_flow_gps_port": {
                            "lat": _safe_float(of_gps_port_out.get("lat")),
                            "lon": _safe_float(of_gps_port_out.get("lon")),
                            "alt_m": _safe_float(of_gps_port_out.get("alt_m")),
                            "heading_deg": _safe_float(of_gps_port_out.get("heading_deg")),
                        },
                        "optical_gps_port_imu": {
                            "lat": _safe_float(of_gps_port_imu_out.get("lat")),
                            "lon": _safe_float(of_gps_port_imu_out.get("lon")),
                            "alt_m": _safe_float(of_gps_port_imu_out.get("alt_m")),
                            "heading_deg": _safe_float(of_gps_port_imu_out.get("heading_deg")),
                        },
                    },
                    "optical_flow": {
                        "speed_x_mps": _safe_float(optical.get("speed_x")),
                        "speed_y_mps": _safe_float(optical.get("speed_y")),
                        "dist_mm": _safe_float(optical.get("dist_mm")),
                        "flow_q": _safe_float(optical.get("flow_q")),
                        "flow_ok": _safe_float(optical.get("flow_ok")),
                        "dist_ok": _safe_float(optical.get("dist_ok")),
                        "ts": optical.get("ts"),
                    },
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/calibration-tuning"):
                payload = {
                    "ok": True,
                    "tuning": {
                        "lat_scale": float(calibration_tuning_state["lat_scale"].get()),
                        "lon_scale": float(calibration_tuning_state["lon_scale"].get()),
                        "alt_offset_m": float(calibration_tuning_state["alt_offset_m"].get()),
                        "vo_scale": float(calibration_tuning_state["vo_scale"].get()),
                        "vo_lat_scale": float(calibration_tuning_state["vo_lat_scale"].get()),
                        "vo_lon_scale": float(calibration_tuning_state["vo_lon_scale"].get()),
                    },
                    "defaults": {
                        "lat_scale": float(calibration_tuning_defaults["lat_scale"]),
                        "lon_scale": float(calibration_tuning_defaults["lon_scale"]),
                        "alt_offset_m": float(calibration_tuning_defaults["alt_offset_m"]),
                        "vo_scale": float(calibration_tuning_defaults["vo_scale"]),
                        "vo_lat_scale": float(calibration_tuning_defaults["vo_lat_scale"]),
                        "vo_lon_scale": float(calibration_tuning_defaults["vo_lon_scale"]),
                    },
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/vo-scale"):
                payload = {
                    "ok": True,
                    "vo_scale": float(calibration_tuning_state["vo_scale"].get()),
                    "vo_lat_scale": float(calibration_tuning_state["vo_lat_scale"].get()),
                    "vo_lon_scale": float(calibration_tuning_state["vo_lon_scale"].get()),
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/optical-flow-scale"):
                payload = {
                    "ok": True,
                    **optical_flow_scale_state.snapshot(),
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/data"):
                payload = state.snapshot()
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/mode"):
                payload = {
                    "mode": mode_state.get(),
                    "allowed": sorted(allowed_modes),
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/gps-format"):
                payload = {
                    "format": gps_format_state.get(),
                    "allowed": list(allowed_gps_formats),
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/altitude-offset"):
                payload = {
                    "offset_m": float(altitude_offset_state.get()),
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/gps-origin"):
                current_origin = get_gps_origin()
                snap = state.snapshot()
                gps_input = (
                    snap.get("sensors", {}).get("gps_input", {})
                    if isinstance(snap, dict)
                    else {}
                )
                payload = {
                    "origin": {
                        "lat": _safe_float(current_origin[0]) if current_origin else None,
                        "lon": _safe_float(current_origin[1]) if current_origin else None,
                        "alt_m": _safe_float(current_origin[2]) if current_origin else None,
                    },
                    "gps_input": {
                        "lat": _safe_float(gps_input.get("lat")),
                        "lon": _safe_float(gps_input.get("lon")),
                        "alt_m": _safe_float(gps_input.get("alt_m")),
                        "fix_type": _safe_float(gps_input.get("fix_type")),
                    },
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/persist"):
                persisted_cfg = _load_yaml(Path(pixhawk_config_path))
                gps_output_cfg = (
                    persisted_cfg.get("gps_output")
                    if isinstance(persisted_cfg, dict)
                    else {}
                )
                if not isinstance(gps_output_cfg, dict):
                    gps_output_cfg = {}
                payload = {
                    "runtime": {
                        "mode": mode_state.get(),
                        "gps_format": gps_format_state.get(),
                        "offset_m": float(altitude_offset_state.get()),
                    },
                    "persisted": {
                        "output_mode": persisted_cfg.get("output_mode")
                        if isinstance(persisted_cfg, dict)
                        else None,
                        "gps_format": gps_output_cfg.get("format"),
                        "altitude_offset_m": persisted_cfg.get("altitude_offset_m")
                        if isinstance(persisted_cfg, dict)
                        else None,
                    },
                    "config_path": str(pixhawk_config_path),
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/service"):
                active_ok, active_msg = _run_service_command(
                    "is-active", NAVISAR_SERVICE_NAME
                )
                payload = {
                    "service": NAVISAR_SERVICE_NAME,
                    "ok": active_ok,
                    "state": active_msg if active_msg else "unknown",
                }
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200 if active_ok else 500)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/blackbox/status"):
                payload = blackbox_recorder.status()
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/blackbox/download"):
                zip_path, download_name = blackbox_recorder.build_download_zip()
                if zip_path is None:
                    data = json.dumps(
                        {"ok": False, "error": "no blackbox session available"}
                    ).encode("utf-8")
                    self.send_response(404)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                try:
                    blob = zip_path.read_bytes()
                finally:
                    try:
                        zip_path.unlink(missing_ok=True)
                    except Exception:
                        pass
                self.send_response(200)
                self.send_header("Content-Type", "application/zip")
                self.send_header(
                    "Content-Disposition",
                    f'attachment; filename="{download_name}"',
                )
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(blob)))
                self.end_headers()
                self.wfile.write(blob)
                return
            if req_path.startswith("/sensor-csv/status"):
                payload = sensor_csv_recorder.status()
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if req_path.startswith("/sensor-csv/download"):
                csv_path, download_name = sensor_csv_recorder.download_csv()
                if csv_path is None:
                    data = json.dumps(
                        {"ok": False, "error": "no sensor csv session available"}
                    ).encode("utf-8")
                    self.send_response(404)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                blob = csv_path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/csv")
                self.send_header(
                    "Content-Disposition",
                    f'attachment; filename="{download_name}"',
                )
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(blob)))
                self.end_headers()
                self.wfile.write(blob)
                return
            if req_path.startswith("/video"):
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        jpg, _ts = frame_state.snapshot()
                        if jpg:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("utf-8"))
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    return
            if req_path.startswith("/frame.jpg"):
                jpg, _ts = frame_state.snapshot()
                if not jpg:
                    self.send_response(503)
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Length", str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
                return
            return super().do_GET()

        def do_POST(self):
            if (
                not self.path.startswith("/mode")
                and not self.path.startswith("/gps-format")
                and not self.path.startswith("/altitude-zero")
                and not self.path.startswith("/altitude-offset")
                and not self.path.startswith("/gps-origin")
                and not self.path.startswith("/persist")
                and not self.path.startswith("/calibration-tuning")
                and not self.path.startswith("/optical-flow-scale")
                and not self.path.startswith("/service")
                and not self.path.startswith("/blackbox")
                and not self.path.startswith("/sensor-csv")
            ):
                return super().do_POST()
            parsed = urlparse(self.path)
            req_path = parsed.path
            length = int(self.headers.get("Content-Length", "0") or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                payload = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                payload = {}
            if req_path.startswith("/mode"):
                requested = str(payload.get("mode", "")).strip().lower()
                if requested in allowed_modes:
                    mode_state.set(requested)
                    persisted = None
                    persist_error = None
                    try:
                        persisted = _persist_pixhawk_runtime_settings(
                            pixhawk_config_path=pixhawk_config_path,
                            mode_state=mode_state,
                            gps_format_state=gps_format_state,
                            altitude_offset_state=altitude_offset_state,
                        )
                    except Exception as exc:
                        persist_error = str(exc)
                    data = json.dumps(
                        {
                            "ok": True,
                            "mode": requested,
                            "persisted": persisted,
                            "persist_error": persist_error,
                        }
                    ).encode("utf-8")
                    self.send_response(200)
                else:
                    data = json.dumps(
                        {"ok": False, "mode": mode_state.get(), "allowed": sorted(allowed_modes)}
                    ).encode("utf-8")
                    self.send_response(400)
            elif req_path.startswith("/gps-format"):
                requested = _normalize_gps_format(payload.get("format"))
                if requested in {"ubx", "nmea", "ubx_nmea"}:
                    gps_format_state.set(requested)
                    data = json.dumps(
                        {
                            "ok": True,
                            "format": requested,
                            "allowed": list(allowed_gps_formats),
                        }
                    ).encode("utf-8")
                    self.send_response(200)
                else:
                    data = json.dumps(
                        {
                            "ok": False,
                            "format": gps_format_state.get(),
                            "allowed": list(allowed_gps_formats),
                        }
                    ).encode("utf-8")
                    self.send_response(400)
            elif req_path.startswith("/altitude-offset"):
                requested = payload.get("offset_m")
                try:
                    offset_m = float(requested)
                except (TypeError, ValueError):
                    data = json.dumps(
                        {"ok": False, "error": "offset_m must be numeric"}
                    ).encode("utf-8")
                    self.send_response(400)
                else:
                    altitude_offset_state.set(offset_m)
                    data = json.dumps({"ok": True, "offset_m": offset_m}).encode("utf-8")
                    self.send_response(200)
            elif req_path.startswith("/gps-origin"):
                mode = str(payload.get("mode", "variable")).strip().lower()
                source = "variable"
                lat = lon = alt_m = None
                if mode in {"initial_home", "initial-home", "home"}:
                    # Allow caller-provided fix to avoid race with live dashboard snapshots.
                    lat = _safe_float(payload.get("lat"))
                    lon = _safe_float(payload.get("lon"))
                    alt_m = _safe_float(payload.get("alt_m"))
                    if lat is None or lon is None:
                        snap = state.snapshot()
                        gps_input = (
                            snap.get("sensors", {}).get("gps_input", {})
                            if isinstance(snap, dict)
                            else {}
                        )
                        lat = _safe_float(gps_input.get("lat"))
                        lon = _safe_float(gps_input.get("lon"))
                        alt_m = _safe_float(gps_input.get("alt_m"))
                    source = "initial_home"
                    if lat is None or lon is None:
                        data = json.dumps(
                            {
                                "ok": False,
                                "error": "no live GPS fix available for initial_home",
                            }
                        ).encode("utf-8")
                        self.send_response(409)
                        self.send_header("Content-Type", "application/json")
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return
                else:
                    try:
                        lat = float(payload.get("lat"))
                        lon = float(payload.get("lon"))
                    except (TypeError, ValueError):
                        data = json.dumps(
                            {
                                "ok": False,
                                "error": "lat/lon must be numeric for variable mode",
                            }
                        ).encode("utf-8")
                        self.send_response(400)
                        self.send_header("Content-Type", "application/json")
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return
                    try:
                        alt_raw = payload.get("alt_m")
                        alt_m = None if alt_raw in (None, "") else float(alt_raw)
                    except (TypeError, ValueError):
                        data = json.dumps(
                            {
                                "ok": False,
                                "error": "alt_m must be numeric when provided",
                            }
                        ).encode("utf-8")
                        self.send_response(400)
                        self.send_header("Content-Type", "application/json")
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return

                set_gps_origin(lat, lon, alt_m)
                written = _persist_gps_origin(pixhawk_config_path, lat, lon, alt_m)
                data = json.dumps(
                    {
                        "ok": True,
                        "source": source,
                        "origin": written,
                    }
                ).encode("utf-8")
                self.send_response(200)
            elif req_path.startswith("/persist"):
                requested_mode = str(payload.get("mode", "")).strip().lower()
                if requested_mode:
                    if requested_mode not in allowed_modes:
                        data = json.dumps(
                            {
                                "ok": False,
                                "error": "invalid mode",
                                "mode": mode_state.get(),
                                "allowed_modes": sorted(allowed_modes),
                            }
                        ).encode("utf-8")
                        self.send_response(400)
                        self.send_header("Content-Type", "application/json")
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return
                    mode_state.set(requested_mode)

                requested_format_raw = payload.get("format")
                if requested_format_raw is not None:
                    requested_format = _normalize_gps_format(requested_format_raw)
                    if requested_format not in {"ubx", "nmea", "ubx_nmea"}:
                        data = json.dumps(
                            {
                                "ok": False,
                                "error": "invalid gps format",
                                "format": gps_format_state.get(),
                                "allowed_formats": list(allowed_gps_formats),
                            }
                        ).encode("utf-8")
                        self.send_response(400)
                        self.send_header("Content-Type", "application/json")
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return
                    gps_format_state.set(requested_format)

                if "offset_m" in payload:
                    try:
                        altitude_offset_state.set(float(payload.get("offset_m")))
                    except (TypeError, ValueError):
                        data = json.dumps(
                            {
                                "ok": False,
                                "error": "offset_m must be numeric",
                            }
                        ).encode("utf-8")
                        self.send_response(400)
                        self.send_header("Content-Type", "application/json")
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return

                written = _persist_pixhawk_runtime_settings(
                    pixhawk_config_path=pixhawk_config_path,
                    mode_state=mode_state,
                    gps_format_state=gps_format_state,
                    altitude_offset_state=altitude_offset_state,
                )
                data = json.dumps(
                    {
                        "ok": True,
                        "written": written,
                        "runtime": {
                            "mode": mode_state.get(),
                            "gps_format": gps_format_state.get(),
                            "offset_m": float(altitude_offset_state.get()),
                        },
                    }
                ).encode("utf-8")
                self.send_response(200)
            elif req_path.startswith("/calibration-tuning"):
                lat_scale_raw = payload.get(
                    "lat_scale", float(calibration_tuning_state["lat_scale"].get())
                )
                lon_scale_raw = payload.get(
                    "lon_scale", float(calibration_tuning_state["lon_scale"].get())
                )
                alt_offset_raw = payload.get(
                    "alt_offset_m", float(calibration_tuning_state["alt_offset_m"].get())
                )
                vo_scale_raw = payload.get(
                    "vo_scale", float(calibration_tuning_state["vo_scale"].get())
                )
                vo_lat_scale_raw = payload.get(
                    "vo_lat_scale", float(calibration_tuning_state["vo_lat_scale"].get())
                )
                vo_lon_scale_raw = payload.get(
                    "vo_lon_scale", float(calibration_tuning_state["vo_lon_scale"].get())
                )
                try:
                    lat_scale = float(lat_scale_raw)
                    lon_scale = float(lon_scale_raw)
                    alt_offset_m = float(alt_offset_raw)
                    vo_scale = float(vo_scale_raw)
                    vo_lat_scale = float(vo_lat_scale_raw)
                    vo_lon_scale = float(vo_lon_scale_raw)
                except (TypeError, ValueError):
                    data = json.dumps(
                        {
                            "ok": False,
                            "error": (
                                "lat_scale, lon_scale, alt_offset_m, vo_scale, "
                                "vo_lat_scale and vo_lon_scale must be numeric"
                            ),
                        }
                    ).encode("utf-8")
                    self.send_response(400)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                lat_scale = max(0.1, min(5.0, lat_scale))
                lon_scale = max(0.1, min(5.0, lon_scale))
                alt_offset_m = max(-20.0, min(20.0, alt_offset_m))
                vo_scale = max(0.1, min(5.0, vo_scale))
                vo_lat_scale = max(0.1, min(5.0, vo_lat_scale))
                vo_lon_scale = max(0.1, min(5.0, vo_lon_scale))
                calibration_tuning_state["lat_scale"].set(lat_scale)
                calibration_tuning_state["lon_scale"].set(lon_scale)
                calibration_tuning_state["alt_offset_m"].set(alt_offset_m)
                calibration_tuning_state["vo_scale"].set(vo_scale)
                calibration_tuning_state["vo_lat_scale"].set(vo_lat_scale)
                calibration_tuning_state["vo_lon_scale"].set(vo_lon_scale)
                persisted_tuning = _persist_calibration_tuning(
                    pixhawk_config_path=pixhawk_config_path,
                    lat_scale=lat_scale,
                    lon_scale=lon_scale,
                    alt_offset_m=alt_offset_m,
                    vo_scale=vo_scale,
                    vo_lat_scale=vo_lat_scale,
                    vo_lon_scale=vo_lon_scale,
                )
                data = json.dumps(
                    {
                        "ok": True,
                        "tuning": {
                            "lat_scale": lat_scale,
                            "lon_scale": lon_scale,
                            "alt_offset_m": alt_offset_m,
                            "vo_scale": vo_scale,
                            "vo_lat_scale": vo_lat_scale,
                            "vo_lon_scale": vo_lon_scale,
                        },
                        "persisted": persisted_tuning,
                        "defaults": {
                            "lat_scale": float(calibration_tuning_defaults["lat_scale"]),
                            "lon_scale": float(calibration_tuning_defaults["lon_scale"]),
                            "alt_offset_m": float(calibration_tuning_defaults["alt_offset_m"]),
                            "vo_scale": float(calibration_tuning_defaults["vo_scale"]),
                            "vo_lat_scale": float(calibration_tuning_defaults["vo_lat_scale"]),
                            "vo_lon_scale": float(calibration_tuning_defaults["vo_lon_scale"]),
                        },
                    }
                ).encode("utf-8")
                self.send_response(200)
            elif req_path.startswith("/vo-scale"):
                scale_raw = payload.get("scale", payload.get("vo_scale"))
                if scale_raw is None:
                    data = json.dumps(
                        {"ok": False, "error": "scale is required"}
                    ).encode("utf-8")
                    self.send_response(400)
                else:
                    try:
                        new_scale = float(scale_raw)
                    except (TypeError, ValueError):
                        data = json.dumps(
                            {"ok": False, "error": "scale must be numeric"}
                        ).encode("utf-8")
                        self.send_response(400)
                    else:
                        new_scale = max(0.1, min(5.0, new_scale))
                        calibration_tuning_state["vo_scale"].set(new_scale)
                        calibration_tuning_state["vo_lat_scale"].set(new_scale)
                        calibration_tuning_state["vo_lon_scale"].set(new_scale)
                        persisted_tuning = _persist_calibration_tuning(
                            pixhawk_config_path=pixhawk_config_path,
                            lat_scale=float(calibration_tuning_state["lat_scale"].get()),
                            lon_scale=float(calibration_tuning_state["lon_scale"].get()),
                            alt_offset_m=float(calibration_tuning_state["alt_offset_m"].get()),
                            vo_scale=float(calibration_tuning_state["vo_scale"].get()),
                            vo_lat_scale=float(calibration_tuning_state["vo_lat_scale"].get()),
                            vo_lon_scale=float(calibration_tuning_state["vo_lon_scale"].get()),
                        )
                        data = json.dumps(
                            {
                                "ok": True,
                                "vo_scale": float(calibration_tuning_state["vo_scale"].get()),
                                "vo_lat_scale": float(
                                    calibration_tuning_state["vo_lat_scale"].get()
                                ),
                                "vo_lon_scale": float(
                                    calibration_tuning_state["vo_lon_scale"].get()
                                ),
                                "persisted": persisted_tuning,
                            }
                        ).encode("utf-8")
                        self.send_response(200)
            elif req_path.startswith("/optical-flow-scale"):
                feature = payload.get("feature")
                lighting = payload.get("lighting")
                altitude = payload.get("altitude")
                scale_raw = payload.get("scale")
                set_active = payload.get("set_active", True)
                active_payload = payload.get("active")
                profiles_payload = payload.get("profiles")

                profiles_updated = False
                active_updated = False

                if isinstance(profiles_payload, dict):
                    profiles_updated = optical_flow_scale_state.update_profiles(profiles_payload)

                if feature is not None or lighting is not None or altitude is not None or "scale" in payload:
                    if feature is None or lighting is None or altitude is None or "scale" not in payload:
                        data = json.dumps(
                            {
                                "ok": False,
                                "error": "feature, lighting, altitude and scale are required for single profile update",
                            }
                        ).encode("utf-8")
                        self.send_response(400)
                    else:
                        try:
                            new_scale = float(scale_raw)
                        except (TypeError, ValueError):
                            data = json.dumps(
                                {
                                    "ok": False,
                                    "error": "scale must be numeric",
                                }
                            ).encode("utf-8")
                            self.send_response(400)
                        else:
                            updated = optical_flow_scale_state.set_profile_scale(
                                feature,
                                lighting,
                                altitude,
                                new_scale,
                            )
                            if updated:
                                profiles_updated = True
                                if bool(set_active):
                                    before = optical_flow_scale_state.snapshot()["active"]
                                    optical_flow_scale_state.set_active(
                                        {
                                            "feature": feature,
                                            "lighting": lighting,
                                            "altitude": altitude,
                                        }
                                    )
                                    after = optical_flow_scale_state.snapshot()["active"]
                                    active_updated = before != after

                if active_payload is not None:
                    if not isinstance(active_payload, dict):
                        data = json.dumps(
                            {
                                "ok": False,
                                "error": "active must be an object",
                            }
                        ).encode("utf-8")
                        self.send_response(400)
                    else:
                        before = optical_flow_scale_state.snapshot()["active"]
                        optical_flow_scale_state.set_active(
                            {
                                "feature": active_payload.get("feature", before["feature"]),
                                "lighting": active_payload.get("lighting", before["lighting"]),
                                "altitude": active_payload.get("altitude", before["altitude"]),
                            }
                        )
                        after = optical_flow_scale_state.snapshot()["active"]
                        active_updated = before != after or active_updated

                if "data" not in locals():
                    if profiles_updated or active_updated:
                        try:
                            persisted = _persist_optical_flow_scale_profiles(
                                pixhawk_config_path=pixhawk_config_path,
                                scale_state=optical_flow_scale_state,
                            )
                        except Exception as exc:
                            data = json.dumps(
                                {
                                    "ok": False,
                                    "error": str(exc),
                                }
                            ).encode("utf-8")
                            self.send_response(500)
                        else:
                            payload = optical_flow_scale_state.snapshot()
                            payload["ok"] = True
                            payload["persisted"] = persisted
                            data = json.dumps(payload).encode("utf-8")
                            self.send_response(200)
                    else:
                        data = json.dumps(
                            {
                                "ok": True,
                                "message": "no change",
                                "scale_state": optical_flow_scale_state.snapshot(),
                            }
                        ).encode("utf-8")
                        self.send_response(200)
            elif req_path.startswith("/service"):
                action = str(payload.get("action", "")).strip().lower()
                if action not in {"start", "stop"}:
                    data = json.dumps(
                        {
                            "ok": False,
                            "error": "action must be 'start' or 'stop'",
                        }
                    ).encode("utf-8")
                    self.send_response(400)
                elif action == "stop":
                    def _stop_later():
                        time.sleep(0.2)
                        _run_service_command("stop", NAVISAR_SERVICE_NAME)

                    threading.Thread(target=_stop_later, daemon=True).start()
                    data = json.dumps(
                        {
                            "ok": True,
                            "action": "stop",
                            "service": NAVISAR_SERVICE_NAME,
                        }
                    ).encode("utf-8")
                    self.send_response(200)
                else:
                    ok, msg = _run_service_command("start", NAVISAR_SERVICE_NAME)
                    data = json.dumps(
                        {
                            "ok": ok,
                            "action": "start",
                            "service": NAVISAR_SERVICE_NAME,
                            "message": msg,
                        }
                    ).encode("utf-8")
                    self.send_response(200 if ok else 500)
            elif req_path.startswith("/blackbox/start"):
                status = blackbox_recorder.start()
                data = json.dumps({"ok": True, "status": status}).encode("utf-8")
                self.send_response(200)
            elif req_path.startswith("/blackbox/stop"):
                status = blackbox_recorder.stop()
                data = json.dumps({"ok": True, "status": status}).encode("utf-8")
                self.send_response(200)
            elif req_path.startswith("/sensor-csv/start"):
                status = sensor_csv_recorder.start()
                data = json.dumps({"ok": True, "status": status}).encode("utf-8")
                self.send_response(200)
            elif req_path.startswith("/sensor-csv/stop"):
                status = sensor_csv_recorder.stop()
                data = json.dumps({"ok": True, "status": status}).encode("utf-8")
                self.send_response(200)
            else:
                snap = state.snapshot()
                current_alt = None
                fused = snap.get("fused") if isinstance(snap, dict) else None
                if isinstance(fused, dict):
                    current_alt = _safe_float(fused.get("z"))
                if current_alt is None:
                    sensors = snap.get("sensors") if isinstance(snap, dict) else None
                    if isinstance(sensors, dict):
                        baro = sensors.get("barometer")
                        if isinstance(baro, dict):
                            current_alt = _safe_float(baro.get("height_m"))
                        oflow = sensors.get("optical_flow")
                        if current_alt is None and isinstance(oflow, dict):
                            dist_mm = _safe_float(oflow.get("distance_mm"))
                            if dist_mm is not None:
                                current_alt = dist_mm / 1000.0
                if current_alt is None:
                    data = json.dumps(
                        {"ok": False, "error": "altitude not available"}
                    ).encode("utf-8")
                    self.send_response(409)
                else:
                    previous_offset = float(altitude_offset_state.get())
                    new_offset = previous_offset - current_alt
                    altitude_offset_state.set(new_offset)
                    data = json.dumps(
                        {
                            "ok": True,
                            "measured_altitude_m": current_alt,
                            "previous_offset_m": previous_offset,
                            "offset_m": new_offset,
                        }
                    ).encode("utf-8")
                    self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, _format, *_args):
            return

    return DashboardHandler


def start_dashboard_server(
    state,
    mode_state,
    gps_format_state,
    altitude_offset_state,
    get_gps_origin,
    set_gps_origin,
    frame_state,
    blackbox_recorder,
    sensor_csv_recorder,
    root_dir,
    host,
    port,
    allowed_modes,
    allowed_gps_formats,
    pixhawk_config_path,
    calibration_enabled,
    calibration_gps_graph_enabled,
    calibration_tuning_state,
    calibration_tuning_defaults,
    optical_flow_scale_state,
    open_browser=True,
):
    handler = _make_dashboard_handler(
        state,
        mode_state,
        gps_format_state,
        altitude_offset_state,
        get_gps_origin,
        set_gps_origin,
        frame_state,
        blackbox_recorder,
        sensor_csv_recorder,
        root_dir,
        allowed_modes,
        allowed_gps_formats,
        pixhawk_config_path,
        calibration_enabled,
        calibration_gps_graph_enabled,
        calibration_tuning_state,
        calibration_tuning_defaults,
        optical_flow_scale_state,
    )
    server = http.server.ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    bind_all = host in {"0.0.0.0", "::", ""}
    local_url = f"http://127.0.0.1:{port}/"
    if bind_all:
        access_urls = [f"http://{ip}:{port}/" for ip in _discover_local_ipv4_addresses()]
        if not access_urls:
            access_urls = [local_url]
    else:
        access_urls = [f"http://{host}:{port}/"]
    print("Dashboard URLs:")
    for url in access_urls:
        print(f"  {url}")
    server.navisar_urls = access_urls
    if open_browser:
        webbrowser.open(local_url if bind_all else access_urls[0])
    return server


def _build_intrinsics(camera_cfg):
    """Build intrinsics matrix and distortion coeffs from config."""
    img_width = camera_cfg.get("width", IMG_WIDTH)
    img_height = camera_cfg.get("height", IMG_HEIGHT)
    intrinsics = camera_cfg.get("intrinsics", {})
    fx = intrinsics.get("fx", FX)
    fy = intrinsics.get("fy", FY)
    cx = intrinsics.get("cx", img_width / 2.0)
    cy = intrinsics.get("cy", img_height / 2.0)
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist_coeffs = intrinsics.get("dist_coeffs", DIST_COEFFS)
    return img_width, img_height, k, dist_coeffs


def _iter_candidate_mavlink_devices(pixhawk_cfg):
    configured = str(pixhawk_cfg.get("device", MAVLINK_DEVICE)).strip()
    if configured.startswith("/dev/tty/ACM"):
        configured = configured.replace("/dev/tty/", "/dev/tty")
    candidates = []
    seen = set()

    if configured:
        candidates.append(configured)
        seen.add(configured)

    for i in range(10):
        path = f"/dev/ttyACM{i}"
        if path not in seen:
            seen.add(path)
            candidates.append(path)

    for path in candidates:
        yield path


def _init_mavlink_interface(pixhawk_cfg, use_barometer, use_mavlink=None):
    """Create and configure the MAVLink interface if enabled."""
    if use_mavlink is None:
        use_mavlink = pixhawk_cfg.get("use_mavlink", USE_MAVLINK)
    attitude_rate_hz = float(pixhawk_cfg.get("attitude_rate_hz", ATTITUDE_RATE_HZ))
    mavlink_device = str(pixhawk_cfg.get("device", MAVLINK_DEVICE)).strip() or MAVLINK_DEVICE
    mavlink_baud = int(pixhawk_cfg.get("baud", MAVLINK_BAUD))
    mavlink_source = pixhawk_cfg.get("mavlink_source", {})
    mavlink_source_sysid = int(mavlink_source.get("system_id", 200))
    mavlink_source_compid = int(
        mavlink_source.get("component_id", mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)
    )
    mavlink_rangefinder_compid = int(
        mavlink_source.get(
            "rangefinder_component_id",
            getattr(mavutil.mavlink, "MAV_COMP_ID_RANGEFINDER", 173),
        )
    )
    barometer_cfg = pixhawk_cfg.get("barometer", {})
    if not isinstance(barometer_cfg, dict):
        barometer_cfg = {}
    barometer_rate_hz = float(barometer_cfg.get("rate_hz", BARO_RATE_HZ))
    default_baro_msgs = [
        "SCALED_PRESSURE",
        "SCALED_PRESSURE2",
        "SCALED_PRESSURE3",
        "HIGHRES_IMU",
    ]
    raw_baro_msgs = barometer_cfg.get("messages", default_baro_msgs)
    if not isinstance(raw_baro_msgs, list):
        raw_baro_msgs = default_baro_msgs
    barometer_messages = []
    for msg_name in raw_baro_msgs:
        msg_text = str(msg_name).strip().upper()
        if not msg_text:
            continue
        if msg_text.startswith("MAVLINK_MSG_ID_"):
            msg_text = msg_text[len("MAVLINK_MSG_ID_"):]
        barometer_messages.append(msg_text)
    if not barometer_messages:
        barometer_messages = default_baro_msgs
    mavlink_interface = None
    if use_mavlink or use_barometer:
        tried = []
        for candidate_device in _iter_candidate_mavlink_devices(pixhawk_cfg):
            tried.append(candidate_device)
            try:
                mavlink_interface = MavlinkInterface(
                    candidate_device,
                    baud=mavlink_baud,
                    source_system=mavlink_source_sysid,
                    source_component=mavlink_source_compid,
                    rangefinder_component=mavlink_rangefinder_compid,
                )
                print(f"Pixhawk connected on {candidate_device}")
                mavlink_interface.request_message_interval(
                    msg_id=mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                    rate_hz=attitude_rate_hz,
                )
                mavlink_interface.request_message_interval(
                    msg_id=mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
                    rate_hz=IMU_RATE_HZ,
                )
                if use_barometer:
                    mavlink_interface.set_barometer_message_types(barometer_messages)
                    for msg_name in barometer_messages:
                        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{msg_name}", None)
                        if msg_id is not None:
                            mavlink_interface.request_message_interval(
                                msg_id=msg_id,
                                rate_hz=barometer_rate_hz,
                            )
                break
            except Exception as exc:
                print(f"MAVLink probe failed on {candidate_device}: {exc}")
                continue
        else:
            print(
                "Warning: Configured MAVLink device not available; trying fallback ports."
                f" (configured: {mavlink_device})"
            )
            if not mavlink_device or not mavlink_device.startswith("/dev/ttyACM"):
                print(
                    "Warning: MAVLink not available on configured device or /dev/ttyACM0-ACM9; "
                    f"using fallback height. Tried: {', '.join(tried) or 'none'}"
                )
            else:
                print(
                    "Warning: MAVLink not available on configured device or /dev/ttyACM0-ACM9; "
                    f"using fallback height. Tried: {', '.join(tried) or 'none'}"
                )
            
    return mavlink_interface


def build_vo_pipeline():
    """Create and configure the visual-odometry pipeline."""
    configs = _load_configs()
    camera_cfg = configs["camera"]
    vio_cfg = configs["vio"]
    pixhawk_cfg = configs["pixhawk"]

    img_width, img_height, k, dist_coeffs = _build_intrinsics(camera_cfg)
    fx = k[0, 0]
    fy = k[1, 1]

    use_barometer = pixhawk_cfg.get("use_barometer", USE_BAROMETER)
    altitude_m = pixhawk_cfg.get("fallback_altitude_m", ALTITUDE_M)

    use_mavlink = pixhawk_cfg.get("use_mavlink", USE_MAVLINK)
    use_imu_fusion = pixhawk_cfg.get("use_imu_fusion", True)
    use_sensor_fusion = pixhawk_cfg.get("use_sensor_fusion", True)

    camera = create_camera_driver(camera_cfg)
    yaw_offset_deg = float(camera_cfg.get("yaw_offset_deg", 0.0))
    mavlink_interface = _init_mavlink_interface(
        pixhawk_cfg, use_barometer=use_barometer, use_mavlink=use_mavlink
    )

    barometer = BarometerHeightEstimator(
        mavlink_interface,
        fallback_m=altitude_m,
    )
    height_estimator = HeightEstimator(
        use_barometer=use_barometer,
        fallback_m=altitude_m,
        barometer_driver=barometer,
    )
    algorithm_name = str(vio_cfg.get("algorithm", "ransac_affine")).lower()
    if algorithm_name in ("median_flow_exact", "median_flow_fb", "median_flow_script"):
        exact_cfg = vio_cfg.get("median_flow_exact", {})
        frame_size = (
            int(camera_cfg.get("width", img_width)),
            int(camera_cfg.get("height", img_height)),
        )
        use_undistort = bool(exact_cfg.get("use_undistort", True))
        focal_length_px = float(exact_cfg.get("focal_length_px", fx))
        height_m = float(exact_cfg.get("height_m", altitude_m))
        K = k.astype(np.float32)
        D = None
        if dist_coeffs is not None:
            dist = np.array(dist_coeffs, dtype=np.float32).ravel()
            if dist.size == 4:
                D = dist
        vo = MedianFlowVO(
            camera_driver=camera,
            height_estimator=height_estimator,
            frame_size=frame_size,
            focal_length_px=focal_length_px,
            height_m=height_m,
            grid_rows=vio_cfg.get("grid_rows", GRID_ROWS),
            grid_cols=vio_cfg.get("grid_cols", GRID_COLS),
            max_corners=exact_cfg.get("max_corners", MAX_FEATURES),
            quality_level=exact_cfg.get("quality_level", 0.01),
            min_distance=exact_cfg.get("min_distance", 7),
            fb_err_thresh=exact_cfg.get("fb_err_thresh", 1.0),
            min_features=exact_cfg.get("min_features", MIN_FEATURES),
            use_undistort=use_undistort,
            K=K,
            D=D,
            frame_delay_s=vio_cfg.get("frame_delay_s", 0.02),
            show_window=exact_cfg.get("show_window", False),
            window_name=exact_cfg.get("window_name", "Median Flow VO"),
        )
    else:
        feature_tracker = FeatureTracker(
            min_features=vio_cfg.get("min_features", MIN_FEATURES),
            max_features=vio_cfg.get("max_features", MAX_FEATURES),
            redetect_interval=vio_cfg.get("redetect_interval", REDETECT_INTERVAL),
            ransac_reproj_thresh=vio_cfg.get("ransac_reproj_thresh", RANSAC_REPROJ_THRESH),
            grid_rows=vio_cfg.get("grid_rows", GRID_ROWS),
            grid_cols=vio_cfg.get("grid_cols", GRID_COLS),
            per_cell_max_features=vio_cfg.get("per_cell_max_features", CELL_MAX_FEATURES),
            texture_threshold=vio_cfg.get("texture_threshold", CELL_TEXTURE_THRESHOLD),
            quality_level=vio_cfg.get("corner_quality_level", CORNER_QUALITY_LEVEL),
        )
        if algorithm_name in ("median_flow", "lk_median", "median"):
            algorithm = MedianFlowEstimator()
        else:
            algorithm = RansacAffineEstimator()
        pose_estimator = PoseEstimator(fx, fy, k, ransac_thresh=1.0, algorithm=algorithm)
        yaw_provider = None
        if mavlink_interface is not None:
            yaw_provider = mavlink_interface.recv_attitude
        vo = VisualOdometry(
            camera_driver=camera,
            feature_tracker=feature_tracker,
            pose_estimator=pose_estimator,
            height_estimator=height_estimator,
            dist_coeffs=dist_coeffs,
            metric_threshold=vio_cfg.get("metric_threshold_m", METRIC_THRESHOLD),
            img_width=img_width,
            img_height=img_height,
            yaw_provider=yaw_provider,
            min_flow_px=vio_cfg.get("min_flow_px", MIN_FLOW_PX),
            min_height_m=vio_cfg.get("min_height_m", MIN_HEIGHT_M),
            exposure_min_mean=vio_cfg.get("exposure_min_mean", EXPOSURE_MIN_MEAN),
            exposure_max_mean=vio_cfg.get("exposure_max_mean", EXPOSURE_MAX_MEAN),
            motion_gate_enabled=vio_cfg.get("motion_gate_enabled", True),
            min_inlier_ratio=vio_cfg.get("min_inlier_ratio", MIN_INLIER_RATIO),
            max_flow_mad_px=vio_cfg.get("max_flow_mad_px", MAX_FLOW_MAD_PX),
        )
        vo.min_inliers = vio_cfg.get("min_inliers", MIN_INLIERS)
        vo.motion_confirm_frames = vio_cfg.get(
            "motion_confirm_frames", MOTION_CONFIRM_FRAMES
        )
        vo.motion_window = vio_cfg.get("motion_smooth_window", MOTION_SMOOTH_WINDOW)
        vo.zero_motion_window = vio_cfg.get("zero_motion_window", ZERO_MOTION_WINDOW)
        vo.zero_motion_mean_m = vio_cfg.get("zero_motion_mean_m", ZERO_MOTION_MEAN_M)
        vo.zero_motion_std_m = vio_cfg.get("zero_motion_std_m", ZERO_MOTION_STD_M)
        vo.motion_deadband_m = vio_cfg.get("motion_deadband_m", 0.003)
    return vo, mavlink_interface, yaw_offset_deg


def _camera_signature(camera_cfg):
    """Return a tuple describing the camera selection for comparison."""
    model = str(camera_cfg.get("model", "opencv")).strip().lower()
    index = None
    format_name = None
    if model in {"opencv", "usb", "generic"}:
        index = camera_cfg.get("index", 0)
    if model in {"ov9281", "ov9821"}:
        format_name = camera_cfg.get("format", "YUV420")
    return model, index, format_name


def build_slam_pipeline(configs, vo_camera=None, vo_camera_cfg=None):
    """Create and configure the visual SLAM pipeline."""
    vio_cfg = configs["vio"]
    slam_cfg = vio_cfg.get("slam", {})
    slam_enabled = bool(slam_cfg.get("enabled", SLAM_ENABLED))
    if not slam_enabled:
        return None
    backend = str(slam_cfg.get("backend", SLAM_BACKEND)).strip().lower()
    if backend != "opencv":
        return None

    share_camera = bool(slam_cfg.get("share_camera", False))
    vo_camera_cfg = vo_camera_cfg or configs.get("camera", {})
    if share_camera and vo_camera is not None:
        _img_width, _img_height, k, dist_coeffs = _build_intrinsics(vo_camera_cfg)
        config = SlamConfig(
            max_features=int(slam_cfg.get("max_features", SLAM_MAX_FEATURES)),
            draw_matches=int(slam_cfg.get("draw_matches", SLAM_DRAW_MATCHES)),
            motion_scale=float(slam_cfg.get("motion_scale", SLAM_MOTION_SCALE)),
            trajectory_size=int(slam_cfg.get("trajectory_size", SLAM_TRAJECTORY_SIZE)),
            trajectory_scale=float(slam_cfg.get("trajectory_scale", SLAM_TRAJECTORY_SCALE)),
            frame_delay_s=float(slam_cfg.get("frame_delay_s", SLAM_FRAME_DELAY_S)),
        )
        return VisualSlam(vo_camera, k, dist_coeffs=dist_coeffs, config=config)

    camera_cfg = dict(configs["camera"])
    camera_cfg["index"] = slam_cfg.get(
        "camera_index", slam_cfg.get("index", camera_cfg.get("index", SLAM_CAMERA_INDEX))
    )
    if "width" in slam_cfg:
        camera_cfg["width"] = slam_cfg["width"]
    if "height" in slam_cfg:
        camera_cfg["height"] = slam_cfg["height"]
    if "model" in slam_cfg:
        camera_cfg["model"] = slam_cfg["model"]
    if "intrinsics" in slam_cfg:
        camera_cfg["intrinsics"] = slam_cfg["intrinsics"]

    if vo_camera_cfg and _camera_signature(camera_cfg) == _camera_signature(vo_camera_cfg):
        print(
            "Warning: SLAM and VO are configured to use the same camera. "
            "Enable slam.share_camera to avoid device contention."
        )

    _img_width, _img_height, k, dist_coeffs = _build_intrinsics(camera_cfg)
    camera = create_camera_driver(camera_cfg)
    config = SlamConfig(
        max_features=int(slam_cfg.get("max_features", SLAM_MAX_FEATURES)),
        draw_matches=int(slam_cfg.get("draw_matches", SLAM_DRAW_MATCHES)),
        motion_scale=float(slam_cfg.get("motion_scale", SLAM_MOTION_SCALE)),
        trajectory_size=int(slam_cfg.get("trajectory_size", SLAM_TRAJECTORY_SIZE)),
        trajectory_scale=float(slam_cfg.get("trajectory_scale", SLAM_TRAJECTORY_SCALE)),
        frame_delay_s=float(slam_cfg.get("frame_delay_s", SLAM_FRAME_DELAY_S)),
    )
    return VisualSlam(camera, k, dist_coeffs=dist_coeffs, config=config)


def _probe_slam_camera(configs):
    """Return True if the SLAM camera can be opened and read at least one frame."""
    vio_cfg = configs["vio"]
    slam_cfg = vio_cfg.get("slam", {})
    backend = str(slam_cfg.get("backend", SLAM_BACKEND)).strip().lower()
    if backend != "opencv":
        return True
    if bool(slam_cfg.get("share_camera", False)):
        return True

    camera_cfg = dict(configs["camera"])
    camera_cfg["index"] = slam_cfg.get(
        "camera_index", slam_cfg.get("index", camera_cfg.get("index", SLAM_CAMERA_INDEX))
    )
    if "width" in slam_cfg:
        camera_cfg["width"] = slam_cfg["width"]
    if "height" in slam_cfg:
        camera_cfg["height"] = slam_cfg["height"]
    if "model" in slam_cfg:
        camera_cfg["model"] = slam_cfg["model"]
    if "intrinsics" in slam_cfg:
        camera_cfg["intrinsics"] = slam_cfg["intrinsics"]

    camera = None
    try:
        camera = create_camera_driver(camera_cfg)
        ok, _frame = camera.read()
        if not ok:
            print("SLAM disabled: camera not available.")
            return False
        return True
    except Exception as exc:
        print(f"SLAM disabled: camera not available ({exc})")
        return False
    finally:
        if camera is not None:
            camera.release()


def _run_slam_process(window_name, trajectory_window):
    """Run the SLAM loop in a separate process for OpenCV GUI stability."""
    configs = _load_configs()
    slam_cfg = configs["vio"].get("slam", {})
    if bool(slam_cfg.get("share_camera", False)):
        print("SLAM: share_camera is not supported in a separate process.")
    visual_slam = build_slam_pipeline(configs)
    if visual_slam is None:
        return
    visual_slam.run(window_name=window_name, trajectory_window=trajectory_window)



def main():
    """Entry point for VO processing and GPS/MAVLink output."""
    configs = _load_configs()
    dashboard_state = None
    dashboard_server = None
    pixhawk_cfg = configs["pixhawk"]
    calibration_cfg = pixhawk_cfg.get("calibration", {})
    calibration_enabled = bool(calibration_cfg.get("enabled", False))
    calibration_gps_graph_enabled = bool(
        calibration_cfg.get("gps_live_graph", calibration_enabled)
    )
    tuning_cfg = calibration_cfg.get("optical_gps_tuning", {})
    if not isinstance(tuning_cfg, dict):
        tuning_cfg = {}
    vo_height_cfg = calibration_cfg.get("vo_height_scaling", {})
    if not isinstance(vo_height_cfg, dict):
        vo_height_cfg = {}
    tuning_defaults = {
        "lat_scale": float(tuning_cfg.get("lat_scale", 1.0)),
        "lon_scale": float(tuning_cfg.get("lon_scale", 1.0)),
        "alt_offset_m": float(tuning_cfg.get("alt_offset_m", 0.0)),
        "vo_scale": float(tuning_cfg.get("vo_scale", 1.0)),
        "vo_lat_scale": float(tuning_cfg.get("vo_lat_scale", tuning_cfg.get("lat_scale", 1.0))),
        "vo_lon_scale": float(tuning_cfg.get("vo_lon_scale", tuning_cfg.get("lon_scale", 1.0))),
    }
    lat_scale_state = ModeState(float(tuning_defaults["lat_scale"]))
    lon_scale_state = ModeState(float(tuning_defaults["lon_scale"]))
    alt_offset_state = ModeState(float(tuning_defaults["alt_offset_m"]))
    vo_scale_state = ModeState(float(tuning_defaults["vo_scale"]))
    vo_lat_scale_state = ModeState(float(tuning_defaults["vo_lat_scale"]))
    vo_lon_scale_state = ModeState(float(tuning_defaults["vo_lon_scale"]))
    vo_height_scaling_enabled = bool(vo_height_cfg.get("enabled", False))
    vo_height_reference_m = max(0.05, float(vo_height_cfg.get("reference_height_m", 1.0)))
    vo_height_power = max(0.0, float(vo_height_cfg.get("power", 1.0)))
    vo_height_min_factor = max(0.1, float(vo_height_cfg.get("min_factor", 0.3)))
    vo_height_max_factor = max(
        vo_height_min_factor, float(vo_height_cfg.get("max_factor", 5.0))
    )
    altitude_offset_m = float(pixhawk_cfg.get("altitude_offset_m", 0.0))
    altitude_offset_state = ModeState(altitude_offset_m)
    use_imu_fusion = pixhawk_cfg.get("use_imu_fusion", True)
    use_sensor_fusion = pixhawk_cfg.get("use_sensor_fusion", True)
    # Output mode controls how odometry and GPS data are consumed/emitted.
    output_mode = str(pixhawk_cfg.get("output_mode", OUTPUT_MODE)).strip().lower()
    if output_mode not in {
        "odometry",
        "gps_mavlink",
        "gps_port",
        "gps_passthrough",
        "optical_flow_mavlink",
        "optical_flow_gps_port",
        "optical_gps_port_imu",
        "optical_flow_then_vo",
    }:
        print(f"Unknown output_mode '{output_mode}', defaulting to '{OUTPUT_MODE}'.")
        output_mode = OUTPUT_MODE
    optical_modes = {
        "optical_flow_mavlink",
        "optical_flow_gps_port",
        "optical_gps_port_imu",
    }
    allowed_modes = set(optical_modes) | {
        "odometry",
        "gps_mavlink",
        "gps_port",
        "gps_passthrough",
        "optical_flow_then_vo",
    }
    hybrid_optical_vo_mode = output_mode == "optical_flow_then_vo"
    # Keep VO pipeline available so runtime dashboard mode switching can move
    # between optical-flow and VO-backed modes without restart.
    use_vo_pipeline = True
    vio_mode = str(pixhawk_cfg.get("vio_mode", VIO_MODE)).strip().lower()
    if vio_mode not in {"vo", "vio_imu"}:
        print(f"Unknown vio_mode '{vio_mode}', defaulting to '{VIO_MODE}'.")
        vio_mode = VIO_MODE
    if not use_vo_pipeline and vio_mode == "vio_imu":
        print("VIO mode ignored for optical-flow outputs; using VO mode settings.")
        vio_mode = "vo"
    mode_state = ModeState(output_mode)
    frame_state = FrameState()
    blackbox_recorder = BlackBoxRecorder(_repo_root() / "blackbox_logs")
    sensor_csv_recorder = SensorCsvRecorder(_repo_root() / "sensor_csv_logs")
    gps_input_cfg = pixhawk_cfg.get("gps_input", {})
    gps_input_enabled = bool(gps_input_cfg.get("enabled", True))
    gps_input_port = gps_input_cfg.get("port")
    gps_input_baud_raw = gps_input_cfg.get("baud")
    if isinstance(gps_input_baud_raw, str) and gps_input_baud_raw.lower() == "auto":
        gps_input_baud = "auto"
    else:
        gps_input_baud = int(gps_input_baud_raw) if gps_input_baud_raw is not None else None
    gps_input_fmt = gps_input_cfg.get("format", GPS_SERIAL_FORMAT)
    gps_input_wait_s = float(gps_input_cfg.get("init_wait_s", 60.0))
    gps_input_startup_wait_s = float(gps_input_cfg.get("startup_wait_s", 5.0))
    gps_input_min_fix = int(gps_input_cfg.get("min_fix_type", GPS_MIN_FIX_TYPE))
    gps_input_reader = None
    passthrough_requested = output_mode == "gps_passthrough"
    gps_serial_cfg = pixhawk_cfg.get("gps_serial", {})
    gps_serial_enabled = bool(gps_serial_cfg.get("enabled", False))
    gps_serial_port = gps_serial_cfg.get("port")
    gps_serial_baud_raw = gps_serial_cfg.get("baud")
    if isinstance(gps_serial_baud_raw, str) and gps_serial_baud_raw.lower() == "auto":
        gps_serial_baud = "auto"
    else:
        gps_serial_baud = (
            int(gps_serial_baud_raw) if gps_serial_baud_raw is not None else None
        )
    gps_serial_fmt = gps_serial_cfg.get("format", GPS_SERIAL_FORMAT)
    gps_serial_min_fix = int(gps_serial_cfg.get("min_fix_type", GPS_MIN_FIX_TYPE))
    gps_serial_reader = None
    optical_cfg = pixhawk_cfg.get("optical_flow", {})
    optical_enabled = bool(optical_cfg.get("enabled", OPTICAL_FLOW_ENABLED))
    optical_port = optical_cfg.get("port", OPTICAL_FLOW_PORT)
    optical_baud = int(optical_cfg.get("baud", OPTICAL_FLOW_BAUD))
    optical_rate_hz = float(optical_cfg.get("rate_hz", OPTICAL_FLOW_RATE_HZ))
    optical_altitude_offset_m = float(optical_cfg.get("altitude_offset_m", 0.0))
    optical_data_mode = str(optical_cfg.get("data_mode", "stable")).strip().lower()
    if optical_data_mode not in {"stable", "raw_tool"}:
        print(
            "Warning: optical_flow.data_mode must be 'stable' or 'raw_tool'; using 'stable'."
        )
        optical_data_mode = "stable"
    optical_raw_mode = optical_data_mode == "raw_tool"
    optical_heartbeat_s = float(
        optical_cfg.get("heartbeat_interval_s", OPTICAL_FLOW_HEARTBEAT_S)
    )
    optical_print = bool(optical_cfg.get("print", OPTICAL_FLOW_PRINT))
    optical_max_flow_raw = optical_cfg.get("max_flow_raw")
    if optical_raw_mode:
        optical_max_flow_raw = None
    if optical_max_flow_raw is not None:
        try:
            optical_max_flow_raw = float(optical_max_flow_raw)
        except (TypeError, ValueError):
            print("Warning: optical_flow.max_flow_raw must be numeric; ignoring.")
            optical_max_flow_raw = None

    optical_flow_scale_state = OpticalFlowScaleProfileState.from_config(
        optical_cfg,
        fallback_scale=1.0,
    )
    optical_reader = None
    last_optical_flow = {"value": None}
    mavlink_interface = None
    barometer_driver = None
    yaw_offset_deg = 0.0
    vo = None
    if use_vo_pipeline:
        vo, mavlink_interface, yaw_offset_deg = build_vo_pipeline()
    else:
        use_barometer = pixhawk_cfg.get("use_barometer", USE_BAROMETER)
        altitude_m = pixhawk_cfg.get("fallback_altitude_m", ALTITUDE_M)
        use_mavlink = pixhawk_cfg.get("use_mavlink", USE_MAVLINK)
        mavlink_interface = _init_mavlink_interface(
            pixhawk_cfg, use_barometer=use_barometer, use_mavlink=use_mavlink
        )
        barometer_driver = BarometerHeightEstimator(
            mavlink_interface,
            fallback_m=altitude_m,
        )
    slam_cfg = configs["vio"].get("slam", {})
    slam_enabled = False
    slam_backend = str(slam_cfg.get("backend", SLAM_BACKEND)).strip().lower()
    run_in_process = bool(slam_cfg.get("run_in_process", True))
    share_camera = False
    shared_camera = None
    visual_slam = None
    slam_thread = None
    slam_process = None
    orbslam_runner = None
    if use_vo_pipeline:
        slam_enabled = bool(slam_cfg.get("enabled", SLAM_ENABLED))
        share_camera = (
            slam_enabled
            and slam_backend == "opencv"
            and bool(slam_cfg.get("share_camera", False))
        )
        if slam_enabled and not share_camera:
            if not _probe_slam_camera(configs):
                slam_enabled = False
        if share_camera:
            if run_in_process:
                print("SLAM: share_camera is ignored when run_in_process is enabled.")
            else:
                shared_camera = SharedCamera(vo.camera_driver)
                vo.camera_driver = shared_camera
        if slam_enabled and slam_backend == "orbslam3":
            orb_cfg = slam_cfg.get("orbslam3", {})
            orbslam_runner = OrbSlam3Runner(
                OrbSlam3Config(
                    command=orb_cfg.get("command"),
                    bin_path=orb_cfg.get("bin_path"),
                    vocab_path=orb_cfg.get("vocab_path"),
                    settings_path=orb_cfg.get("settings_path"),
                    dataset_path=orb_cfg.get("dataset_path"),
                    timestamps_path=orb_cfg.get("timestamps_path"),
                    extra_args=orb_cfg.get("extra_args"),
                    cwd=orb_cfg.get("cwd"),
                )
            )
            try:
                orbslam_runner.start()
            except Exception as exc:
                print(f"ORB-SLAM3 launch failed: {exc}")
                orbslam_runner = None
        if slam_enabled and slam_backend == "opencv":
            if not run_in_process:
                visual_slam = build_slam_pipeline(
                    configs, vo_camera=shared_camera, vo_camera_cfg=configs.get("camera", {})
                )
            slam_window = str(
                configs["vio"]
                .get("slam", {})
                .get("window_name", SLAM_WINDOW_NAME)
            )
            traj_window = str(
                configs["vio"]
                .get("slam", {})
                .get("trajectory_window", SLAM_TRAJECTORY_WINDOW)
            )
            if run_in_process:
                slam_process = multiprocessing.Process(
                    target=_run_slam_process,
                    args=(slam_window, traj_window),
                    daemon=True,
                )
                slam_process.start()
            else:
                if visual_slam is None:
                    visual_slam = build_slam_pipeline(
                        configs,
                        vo_camera=shared_camera,
                        vo_camera_cfg=configs.get("camera", {}),
                    )
                slam_thread = threading.Thread(
                    target=visual_slam.run,
                    kwargs={"window_name": slam_window, "trajectory_window": traj_window},
                    daemon=True,
                )
                slam_thread.start()
    fusion = SensorFusion() if use_sensor_fusion else None
    if not use_sensor_fusion:
        print("Sensor fusion disabled: using raw VO position for outputs.")
    vio_imu_cfg = pixhawk_cfg.get("vio_imu", {})
    vio_imu_print = bool(vio_imu_cfg.get("print", True))
    vio_imu_print_interval_s = float(vio_imu_cfg.get("print_interval_s", 0.5))
    imu_estimator = None
    imu_last_print = {"time": 0.0}
    if vio_mode == "vio_imu":
        if mavlink_interface is None:
            raise RuntimeError("vio_imu mode requires a MAVLink connection.")
        imu_rate_hz = float(pixhawk_cfg.get("imu_rate_hz", vio_imu.IMU_RATE_HZ))
        print("VIO mode: VO + IMU velocity integration.")
        mavlink_interface.request_message_interval(
            msg_id=mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
            rate_hz=imu_rate_hz,
        )
        mavlink_interface.request_message_interval(
            msg_id=mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
            rate_hz=imu_rate_hz,
        )
        imu_estimator = vio_imu.ImuVelocityEstimator()
    if mavlink_interface is not None:
        imu_rate_hz = float(pixhawk_cfg.get("imu_rate_hz", vio_imu.IMU_RATE_HZ))
        mavlink_interface.request_message_interval(
            msg_id=mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
            rate_hz=imu_rate_hz,
        )
    selector = PositionSourceSelector(
        drift_threshold_m=pixhawk_cfg.get("gps_drift_threshold_m", GPS_DRIFT_THRESHOLD_M),
        gps_timeout_s=pixhawk_cfg.get("gps_timeout_s", GPS_TIMEOUT_S),
        min_fix_type=pixhawk_cfg.get("gps_min_fix_type", GPS_MIN_FIX_TYPE),
    )
    gnss_cfg = pixhawk_cfg.get("gnss_monitor", {})
    spoof_cfg = gnss_cfg.get("spoof_detector", {})
    spoof_detector = None
    if bool(spoof_cfg.get("enabled", True)):
        spoof_detector = SpoofDetector(
            drift_threshold_m=float(
                spoof_cfg.get("drift_threshold_m", GPS_DRIFT_THRESHOLD_M)
            ),
            max_speed_mps=float(spoof_cfg.get("max_speed_mps", SPOOF_MAX_SPEED_MPS)),
            consecutive_required=int(
                spoof_cfg.get("consecutive", SPOOF_CONSECUTIVE)
            ),
            cooldown_s=float(spoof_cfg.get("cooldown_s", SPOOF_COOLDOWN_S)),
            min_fix_type=int(spoof_cfg.get("min_fix_type", GPS_MIN_FIX_TYPE)),
        )
    reporter_cfg = gnss_cfg.get("spoof_reporter", {})
    spoof_reporter = None
    if bool(reporter_cfg.get("enabled", True)):
        spoof_reporter = SpoofReporter(
            SpoofReportConfig(
                log_path=reporter_cfg.get("log_path", "data/spoof_events.jsonl"),
                gcs_severity=reporter_cfg.get("gcs_severity", "warning"),
                min_interval_s=float(reporter_cfg.get("min_interval_s", 2.0)),
            ),
            mavlink_interface=mavlink_interface,
        )
    gps_output_cfg = pixhawk_cfg.get("gps_output", {})
    gps_output_enabled = bool(gps_output_cfg.get("enabled", False))
    gps_output_format = _normalize_gps_format(gps_output_cfg.get("format", "nmea"))
    gps_output_port = gps_output_cfg.get("port", GPS_OUTPUT_PORT)
    gps_output_baud = int(gps_output_cfg.get("baud", GPS_OUTPUT_BAUD))
    gps_output_rate_hz = float(gps_output_cfg.get("rate_hz", GPS_OUTPUT_RATE_HZ))
    gps_output_heading_mode = (
        str(gps_output_cfg.get("heading_mode", "send")).strip().lower() or "send"
    )
    if gps_output_heading_mode not in {"send", "none"}:
        print(
            "Warning: unsupported gps_output.heading_mode "
            f"'{gps_output_heading_mode}', falling back to send."
        )
        gps_output_heading_mode = "send"
    gps_output_send_heading = gps_output_heading_mode == "send"
    gps_output_fix_quality = int(
        gps_output_cfg.get("fix_quality", GPS_OUTPUT_FIX_QUALITY)
    )
    gps_output_min_sats = int(gps_output_cfg.get("min_sats", GPS_OUTPUT_MIN_SATS))
    gps_output_max_sats = int(gps_output_cfg.get("max_sats", GPS_OUTPUT_MAX_SATS))
    gps_output_update_s = float(
        gps_output_cfg.get("update_s", GPS_OUTPUT_UPDATE_S)
    )
    gps_output_ubx_h_acc_mm = 500
    gps_output_ubx_v_acc_mm = 100
    gps_output_print = bool(gps_output_cfg.get("print", True))
    gps_output_raw_print = bool(gps_output_cfg.get("raw_print", False))
    nmea_emitter = None
    ubx_emitter = None
    vps_gps_cfg = pixhawk_cfg.get("vps_gps", {})
    vps_gps_send_interval_s = float(
        vps_gps_cfg.get("send_interval_s", ODOM_GPS_SEND_INTERVAL_S)
    )
    mavlink_gps_send_interval_s = float(
        vps_gps_cfg.get("mavlink_send_interval_s", vps_gps_send_interval_s)
    )
    gps_mav_emitter = FakeGpsEmitter(
        send_interval_s=mavlink_gps_send_interval_s,
        smooth_alpha=float(vps_gps_cfg.get("smooth_alpha", FAKE_GPS_SMOOTH_ALPHA)),
        max_step_m=float(vps_gps_cfg.get("max_step_m", FAKE_GPS_MAX_STEP_M)),
    )
    gps_port_emitter = FakeGpsEmitter(
        send_interval_s=vps_gps_send_interval_s,
        smooth_alpha=float(vps_gps_cfg.get("smooth_alpha", FAKE_GPS_SMOOTH_ALPHA)),
        max_step_m=float(vps_gps_cfg.get("max_step_m", FAKE_GPS_MAX_STEP_M)),
    )
    vps_gps_fix_type = int(vps_gps_cfg.get("fix_type", ODOM_GPS_FIX_TYPE))
    vps_gps_sats = int(vps_gps_cfg.get("satellites", ODOM_GPS_SATS))
    vps_gps_print = bool(vps_gps_cfg.get("print", True))
    vps_gps_use_compass_yaw = bool(vps_gps_cfg.get("use_compass_yaw", False))
    vps_gps_ignore_flags = int(vps_gps_cfg.get("ignore_flags", 28))
    print(f"GPS_INPUT ignore_flags: {vps_gps_ignore_flags}")
    if (
        gps_output_enabled
        and not passthrough_requested
        and gps_output_format in {"nmea", "ubx_nmea", "ubx+nmea"}
    ):
        nmea_emitter = NmeaSerialEmitter(
            port=gps_output_port,
            baud=gps_output_baud,
            rate_hz=gps_output_rate_hz,
            fix_quality=gps_output_fix_quality,
            min_sats=gps_output_min_sats,
            max_sats=gps_output_max_sats,
            update_s=gps_output_update_s,
            raw_print=gps_output_raw_print,
        )
    if (
        gps_output_enabled
        and not passthrough_requested
        and gps_output_format in {"ubx", "ubx_nmea", "ubx+nmea"}
    ):
        ubx_emitter = UbxSerialEmitter(
            port=gps_output_port,
            baud=gps_output_baud,
            rate_hz=gps_output_rate_hz,
            fix_type=vps_gps_fix_type,
            min_sats=gps_output_min_sats,
            max_sats=gps_output_max_sats,
            update_s=gps_output_update_s,
            h_acc_mm=gps_output_ubx_h_acc_mm,
            v_acc_mm=gps_output_ubx_v_acc_mm,
            raw_print=gps_output_raw_print,
        )
    gps_format_order = ("ubx", "nmea", "ubx_nmea")
    allowed_gps_formats = list(gps_format_order) if gps_output_enabled else []
    initial_gps_format = gps_output_format
    if initial_gps_format not in allowed_gps_formats:
        if "ubx_nmea" in allowed_gps_formats:
            initial_gps_format = "ubx_nmea"
        elif "ubx" in allowed_gps_formats:
            initial_gps_format = "ubx"
        elif "nmea" in allowed_gps_formats:
            initial_gps_format = "nmea"
        else:
            initial_gps_format = "nmea"
    gps_format_state = ModeState(initial_gps_format)
    if DASHBOARD_ENABLED:
        dashboard_state = DashboardState()
        dashboard_root = _repo_root() / "simulation"
        try:
            dashboard_server = start_dashboard_server(
                dashboard_state,
                mode_state,
                gps_format_state,
                altitude_offset_state,
                selector.gps_origin,
                selector.set_gps_origin,
                frame_state,
                blackbox_recorder,
                sensor_csv_recorder,
                dashboard_root,
                DASHBOARD_HOST,
                DASHBOARD_PORT,
                allowed_modes,
                allowed_gps_formats,
                _repo_root() / "config" / "pixhawk.yaml",
                calibration_enabled,
                calibration_gps_graph_enabled,
                {
                    "lat_scale": lat_scale_state,
                    "lon_scale": lon_scale_state,
                    "alt_offset_m": alt_offset_state,
                    "vo_scale": vo_scale_state,
                    "vo_lat_scale": vo_lat_scale_state,
                    "vo_lon_scale": vo_lon_scale_state,
                },
                tuning_defaults,
                optical_flow_scale_state,
                open_browser=_can_auto_open_browser(),
            )
            server_urls = getattr(dashboard_server, "navisar_urls", None) or [
                f"http://{DASHBOARD_HOST}:{DASHBOARD_PORT}/"
            ]
            dashboard_state.update(
                {
                    "status": "running",
                    "url": server_urls[0],
                    "urls": server_urls,
                    "calibration_enabled": calibration_enabled,
                    "calibration_gps_graph_enabled": calibration_gps_graph_enabled,
                    "calibration_tuning": {
                        "lat_scale": float(lat_scale_state.get()),
                        "lon_scale": float(lon_scale_state.get()),
                        "alt_offset_m": float(alt_offset_state.get()),
                        "vo_scale": float(vo_scale_state.get()),
                        "vo_lat_scale": float(vo_lat_scale_state.get()),
                        "vo_lon_scale": float(vo_lon_scale_state.get()),
                    },
                }
            )
        except Exception as exc:
            _ = exc
            dashboard_state = None
    gps_origin = pixhawk_cfg.get("gps_origin", {})
    cfg_lat = gps_origin.get("lat")
    cfg_lon = gps_origin.get("lon")
    cfg_alt = gps_origin.get("alt")
    gps_input_probe_seconds = float(gps_input_cfg.get("probe_seconds", 3.0))
    gps_input_probe_port = (
        gps_input_cfg.get("probe_port")
        or gps_input_port
        or "/dev/ttyAMA0"
    )
    if passthrough_requested:
        gps_input_enabled = False
    if gps_input_enabled:
        if gps_input_startup_wait_s > 0:
            print(
                f"Waiting {gps_input_startup_wait_s:.1f}s for GPS sensor warm-up..."
            )
            time.sleep(gps_input_startup_wait_s)
        probe_locked = False
        if isinstance(gps_input_baud, int):
            probe_locked = probe_nmea_on_port(
                gps_input_probe_port,
                baud=gps_input_baud,
                seconds=gps_input_probe_seconds,
                verbose=True,
            )
            if probe_locked:
                gps_input_port = gps_input_probe_port
            else:
                fallback = find_gps_port_and_baud(
                    port=gps_input_probe_port,
                    bauds=[4800, 9600, 19200, 38400, 57600, 115200, 230400],
                    probe_seconds=gps_input_probe_seconds,
                    verbose=True,
                )
                if fallback:
                    gps_input_port, gps_input_baud = fallback
                    probe_locked = True
                    print(
                        "GPS probe fallback matched "
                        f"{gps_input_port} @ {gps_input_baud}"
                    )
        else:
            fallback = find_gps_port_and_baud(
                port=gps_input_probe_port,
                probe_seconds=gps_input_probe_seconds,
                verbose=True,
            )
            if fallback:
                gps_input_port, gps_input_baud = fallback
                probe_locked = True
                print(
                    f"GPS auto probe locked {gps_input_port} @ {gps_input_baud}"
                )

        if not probe_locked:
            print(
                f"GPS probe did not lock on {gps_input_probe_port} after "
                f"{gps_input_probe_seconds:.0f}s; continuing with configured GPS input."
            )
            if gps_input_port is None:
                gps_input_enabled = False
    if GPS_ORIGIN_LAT is not None and GPS_ORIGIN_LON is not None:
        try:
            origin_lat = float(GPS_ORIGIN_LAT)
            origin_lon = float(GPS_ORIGIN_LON)
            origin_alt = float(GPS_ORIGIN_ALT) if GPS_ORIGIN_ALT is not None else None
            selector.set_gps_origin(origin_lat, origin_lon, origin_alt)
            print(
                "Using manual GPS origin: "
                f"lat={origin_lat:.7f} lon={origin_lon:.7f} alt_m={origin_alt or 0.0:.2f}"
            )
        except ValueError:
            print("Invalid GPS_ORIGIN_* values; expected numeric strings.")
    elif gps_input_enabled:
        if gps_input_port is None:
            raise ValueError("gps_input.port must be set in config/pixhawk.yaml")
        if gps_input_baud is None:
            raise ValueError("gps_input.baud must be set in config/pixhawk.yaml")
        try:
            gps_input_reader = GpsSerialReader(
                gps_input_port, baud=gps_input_baud, fmt=gps_input_fmt
            )
            print(
                f"Waiting for GPS fix on {gps_input_port} @ {gps_input_baud} "
                f"(timeout {gps_input_wait_s:.0f}s)"
            )
            deadline = time.time() + gps_input_wait_s
            while time.time() < deadline:
                fix, fix_time = gps_input_reader.read_messages()
                if fix is not None and fix.get("fix_type", 0) >= gps_input_min_fix:
                    selector.set_gps_origin(
                        fix["lat"], fix["lon"], fix.get("alt_m")
                    )
                    print(
                        "Using GPS origin from serial: "
                        f"lat={fix['lat']:.7f} lon={fix['lon']:.7f} "
                        f"alt_m={(fix.get('alt_m') or 0.0):.2f}"
                    )
                    break
                time.sleep(0.1)
            if selector.gps_origin() is None:
                if cfg_lat is not None and cfg_lon is not None:
                    selector.set_gps_origin(
                        float(cfg_lat),
                        float(cfg_lon),
                        float(cfg_alt) if cfg_alt else None,
                    )
                    print(
                        "GPS origin fallback: "
                        f"lat={float(cfg_lat):.7f} lon={float(cfg_lon):.7f} "
                        f"alt_m={float(cfg_alt) if cfg_alt else 0.0:.2f}"
                    )
                else:
                    print("GPS origin not set; set gps_origin in config or env.")
        except Exception as exc:
            print(f"Warning: GPS input not available ({exc})")
    elif cfg_lat is not None and cfg_lon is not None:
        selector.set_gps_origin(float(cfg_lat), float(cfg_lon), float(cfg_alt) if cfg_alt else None)

    if gps_serial_enabled:
        try:
            gps_serial_reader = GpsSerialReader(
                gps_serial_port, baud=gps_serial_baud, fmt=gps_serial_fmt
            )
            print(
                "GPS serial enabled: "
                f"{gps_serial_reader.port} @ {gps_serial_reader.baud}"
            )
        except Exception as exc:
            print(f"Warning: GPS serial not available ({exc})")
            gps_serial_reader = None
    compass_cfg = pixhawk_cfg.get("compass", {})
    compass_enabled = bool(compass_cfg.get("enabled", False))
    compass_rate_hz = float(compass_cfg.get("rate_hz", 10.0))
    compass_print = bool(compass_cfg.get("print", False))
    compass_bus = int(compass_cfg.get("i2c_bus", 1))
    compass_mode = str(compass_cfg.get("mode", "i2c")).strip().lower() or "i2c"
    compass_send_mavlink = bool(compass_cfg.get("send_mavlink", True))
    compass_calibration = dict(compass_cfg.get("calibration", {}))
    if compass_mode not in {"i2c", "mavlink_compass"}:
        print(
            f"Warning: unsupported compass mode '{compass_mode}', falling back to i2c."
        )
        compass_mode = "i2c"
    compass_calibration_file = compass_cfg.get("calibration_file")
    if compass_calibration_file:
        calibration_path = Path(str(compass_calibration_file)).expanduser()
        if not calibration_path.is_absolute():
            calibration_path = (_repo_root() / calibration_path).resolve()
        try:
            file_calibration = load_calibration_file(calibration_path)
            # Inline config stays as highest-priority override.
            merged_calibration = dict(file_calibration)
            merged_calibration.update(compass_calibration)
            compass_calibration = merged_calibration
            print(f"Loaded compass calibration file: {calibration_path}")
        except Exception as exc:
            print(f"Warning: failed to load compass calibration file {calibration_path} ({exc})")
    compass_heading_offset = compass_cfg.get("heading_offset_deg")
    if compass_heading_offset is not None and "heading_offset_deg" not in compass_calibration:
        compass_calibration["heading_offset_deg"] = compass_heading_offset
    compass_heading_alpha = compass_cfg.get("heading_smoothing_alpha")
    compass_heading_max_delta = compass_cfg.get("heading_max_delta_deg")
    compass_heading_jump_reject = compass_cfg.get("heading_jump_reject_deg")
    compass_reader = None
    if compass_enabled:
        if mavlink_interface is None:
            print("Warning: compass enabled but MAVLink is not available.")
        if compass_mode == "mavlink_compass":
            compass_meta = {
                "bus": None,
                "addr": None,
                "source": "mavlink_compass",
                "message_type": None,
            }
            print("Compass enabled in MAVLink mode.")
        else:
            try:
                compass_reader = CompassReader(
                    preferred_bus=compass_bus,
                    calibration=compass_calibration,
                )
                compass_meta = {
                    "bus": compass_reader.bus_index,
                    "addr": compass_reader.addr,
                    "source": "i2c",
                    "message_type": None,
                }
                print(
                    f"Compass enabled on I2C bus {compass_reader.bus_index} "
                    f"(addr=0x{compass_reader.addr:02X})"
                )
            except Exception as exc:
                print(f"Warning: compass not available ({exc})")
                compass_reader = None
    compass_interval_s = 1.0 / compass_rate_hz if compass_rate_hz > 0 else 0.0
    last_compass_time = {"time": 0.0}
    last_compass_error = {"time": 0.0}
    last_compass_in = {"heading_deg": None, "x_mg": None, "y_mg": None, "z_mg": None}
    last_compass_out = {"time_boot_ms": None, "x_mg": None, "y_mg": None, "z_mg": None}
    if "compass_meta" not in locals():
        compass_meta = {
            "bus": None,
            "addr": None,
            "source": compass_mode,
            "message_type": None,
        }

    def _update_compass(now):
        if (
            not compass_enabled
            or compass_interval_s <= 0.0
            or now - last_compass_time["time"] < compass_interval_s
        ):
            return
        last_compass_time["time"] = now
        try:
            if compass_mode == "mavlink_compass":
                if mavlink_interface is None:
                    return
                compass_sample = mavlink_interface.recv_compass()
                if compass_sample is None:
                    return
                heading, (x_mg, y_mg, z_mg) = heading_from_milligauss(
                    compass_sample["x_mg"],
                    compass_sample["y_mg"],
                    compass_sample["z_mg"],
                    calibration=compass_calibration,
                )
                compass_meta["message_type"] = compass_sample.get("message_type")
            else:
                if compass_reader is None:
                    return
                heading, (x_mg, y_mg, z_mg) = compass_reader.read_milligauss()
            if compass_heading_jump_reject is not None:
                prev_heading = last_compass_in.get("heading_deg")
                if prev_heading is not None:
                    delta = (heading - prev_heading + 540.0) % 360.0 - 180.0
                    if abs(delta) > float(compass_heading_jump_reject):
                        heading = prev_heading
            if compass_heading_alpha is not None:
                heading = _smooth_heading_deg(
                    last_compass_in.get("heading_deg"),
                    heading,
                    compass_heading_alpha,
                    compass_heading_max_delta,
                )
            last_compass_in.update(
                {
                    "heading_deg": heading,
                    "x_mg": x_mg,
                    "y_mg": y_mg,
                    "z_mg": z_mg,
                }
            )
            time_boot_ms = int(now * 1000) % (2**32)
            if (
                compass_mode == "i2c"
                and compass_send_mavlink
                and mavlink_interface is not None
            ):
                mavlink_interface.send_compass(
                    x_mg, y_mg, z_mg, time_boot_ms=time_boot_ms
                )
                last_compass_out.update(
                    {
                        "time_boot_ms": time_boot_ms,
                        "x_mg": int(x_mg),
                        "y_mg": int(y_mg),
                        "z_mg": int(z_mg),
                    }
                )
            if compass_print:
                print(
                    f"Compass[{compass_mode}]: heading={heading:6.2f} deg "
                    f"mag=({x_mg:.1f},{y_mg:.1f},{z_mg:.1f})"
                )
        except Exception as exc:
            if now - last_compass_error["time"] > 2.0:
                last_compass_error["time"] = now
                print(f"Warning: compass read failed ({exc})")
    if optical_enabled:
        try:
            optical_reader = MTF01OpticalFlowReader(
                port=optical_port,
                baudrate=optical_baud,
                data_frequency=optical_rate_hz,
                heartbeat_interval_s=optical_heartbeat_s,
                print_enabled=optical_print,
                max_flow_raw=optical_max_flow_raw,
            )
            optical_reader.start()
            print(
                "Optical flow enabled: "
                f"{optical_port} @ {optical_baud} ({optical_rate_hz:.1f} Hz)"
            )
        except Exception as exc:
            print(f"Warning: optical flow not available ({exc})")
            optical_reader = None
    last_source = {"value": None}
    last_report = {"time": 0.0}
    last_report_times = {"gps": 0.0, "baro": 0.0}
    last_vio_imu = {"time": None, "x": None, "y": None, "z": None}
    last_cam_fusion = {"time": None, "x": None, "y": None, "z": None}
    last_imu = {"ax": None, "ay": None, "az": None, "gx": None, "gy": None, "gz": None}
    last_attitude = {"value": None}
    last_gps_input = {"value": None}
    last_baro = {"time": None, "height_m": None, "vz_mps": None}
    last_runtime_mode = {"requested": None, "active": None, "drive": None}
    last_runtime_gps_format = {"requested": None, "active": None}
    heading_state = {"deg": None, "source": "none"}
    heading_velocity_min_mps = float(
        pixhawk_cfg.get("heading_velocity_min_mps", 0.03)
    )
    heading_runtime_alpha = pixhawk_cfg.get("heading_runtime_alpha", 0.35)
    heading_runtime_max_delta = pixhawk_cfg.get("heading_runtime_max_delta_deg", 45.0)

    odom_send_interval_s = pixhawk_cfg.get(
        "odometry_send_interval_s", ODOMETRY_SEND_INTERVAL_S
    )
    print_interval_s = pixhawk_cfg.get("print_interval_s", PRINT_INTERVAL_S)
    gps_mavlink_mode = GpsMavlinkMode(
        emitter=gps_mav_emitter,
        fix_type=vps_gps_fix_type,
        satellites=vps_gps_sats,
        print_enabled=vps_gps_print,
        ignore_flags=vps_gps_ignore_flags,
    )
    configured_final_altitude_offset_m = float(
        pixhawk_cfg.get("final_altitude_offset_m", 0.0)
    )
    startup_baro_driver = barometer_driver
    if startup_baro_driver is None and vo is not None:
        startup_baro_driver = getattr(
            getattr(vo, "height_estimator", None), "barometer_driver", None
        )
    calibrated_final_altitude_offset_m = configured_final_altitude_offset_m
    # Read startup barometer once (no wait window) and use it as optical-flow altitude offset.
    startup_optical_altitude_offset_m = None
    if startup_baro_driver is not None:
        try:
            startup_baro_driver.update()
        except Exception:
            pass
        startup_optical_altitude_offset_m = _safe_float(
            getattr(startup_baro_driver, "raw_alt_m", None)
        )
        if startup_optical_altitude_offset_m is None:
            startup_optical_altitude_offset_m = _safe_float(
                getattr(startup_baro_driver, "current_m", None)
            )
        if startup_optical_altitude_offset_m is None:
            try:
                startup_optical_altitude_offset_m = _safe_float(
                    startup_baro_driver.get_height_m()
                )
            except Exception:
                startup_optical_altitude_offset_m = None
    if startup_optical_altitude_offset_m is not None:
        optical_altitude_offset_m = float(startup_optical_altitude_offset_m)
        optical_cfg["altitude_offset_m"] = optical_altitude_offset_m
        print(
            "Optical altitude offset initialized from startup barometer sample: "
            f"{optical_altitude_offset_m:.3f} m"
        )
    else:
        print(
            "Startup barometer sample unavailable for optical offset; "
            f"using config optical_flow.altitude_offset_m={optical_altitude_offset_m:.3f} m"
        )
    gps_port_mode = GpsPortMode(
        emitter=gps_port_emitter,
        nmea_emitter=nmea_emitter,
        ubx_emitter=ubx_emitter,
        print_enabled=gps_output_print,
        final_altitude_offset_m=calibrated_final_altitude_offset_m,
    )
    gps_passthrough_cfg = pixhawk_cfg.get("gps_passthrough", {})
    gps_passthrough_input_port = gps_passthrough_cfg.get("input_port", gps_input_port)
    gps_passthrough_input_baud = gps_passthrough_cfg.get("input_baud", gps_input_baud)
    if isinstance(gps_passthrough_input_baud, str):
        gps_passthrough_input_baud = (
            9600 if gps_passthrough_input_baud.strip().lower() == "auto"
            else int(gps_passthrough_input_baud)
        )
    if gps_passthrough_input_baud is None:
        gps_passthrough_input_baud = 9600
    gps_passthrough_output_port = gps_passthrough_cfg.get("output_port", gps_output_port)
    gps_passthrough_output_baud = int(
        gps_passthrough_cfg.get("output_baud", gps_output_baud)
    )
    gps_passthrough_log_dir = gps_passthrough_cfg.get(
        "log_dir", str(_repo_root() / "data" / "gps_passthrough_logs")
    )
    gps_passthrough_mode = GpsPassthroughMode(
        input_port=gps_passthrough_input_port,
        input_baud=gps_passthrough_input_baud,
        output_port=gps_passthrough_output_port,
        output_baud=gps_passthrough_output_baud,
        log_dir=gps_passthrough_log_dir,
        print_enabled=bool(gps_passthrough_cfg.get("print", True)),
    )

    def _ensure_nmea_emitter():
        nonlocal nmea_emitter
        if nmea_emitter is not None or not gps_output_enabled:
            return
        try:
            nmea_emitter = NmeaSerialEmitter(
                port=gps_output_port,
                baud=gps_output_baud,
                rate_hz=gps_output_rate_hz,
                fix_quality=gps_output_fix_quality,
                min_sats=gps_output_min_sats,
                max_sats=gps_output_max_sats,
                update_s=gps_output_update_s,
                raw_print=gps_output_raw_print,
            )
        except Exception as exc:
            print(f"Warning: unable to initialize NMEA emitter ({exc})")

    def _close_nmea_emitter():
        nonlocal nmea_emitter
        if nmea_emitter is None:
            return
        try:
            nmea_emitter._ser.close()
        except Exception:
            pass
        nmea_emitter = None

    def _ensure_ubx_emitter():
        nonlocal ubx_emitter
        if ubx_emitter is not None or not gps_output_enabled:
            return
        try:
            ubx_emitter = UbxSerialEmitter(
                port=gps_output_port,
                baud=gps_output_baud,
                rate_hz=gps_output_rate_hz,
                fix_type=vps_gps_fix_type,
                min_sats=gps_output_min_sats,
                max_sats=gps_output_max_sats,
                update_s=gps_output_update_s,
                h_acc_mm=gps_output_ubx_h_acc_mm,
                v_acc_mm=gps_output_ubx_v_acc_mm,
                raw_print=gps_output_raw_print,
            )
        except Exception as exc:
            print(f"Warning: unable to initialize UBX emitter ({exc})")

    def _close_ubx_emitter():
        nonlocal ubx_emitter
        if ubx_emitter is None:
            return
        try:
            ubx_emitter._ser.close()
        except Exception:
            pass
        ubx_emitter = None

    def apply_gps_format_selection():
        requested = _normalize_gps_format(gps_format_state.get())
        if mode_state.get() == "gps_passthrough":
            _close_nmea_emitter()
            _close_ubx_emitter()
            gps_port_mode.nmea_emitter = None
            gps_port_mode.ubx_emitter = None
            return requested, "disabled"
        if requested == "nmea":
            _ensure_nmea_emitter()
        elif requested == "ubx":
            _ensure_ubx_emitter()
        elif requested == "ubx_nmea":
            _ensure_nmea_emitter()
            _ensure_ubx_emitter()
        if requested not in {"ubx", "nmea", "ubx_nmea"}:
            requested = initial_gps_format
            gps_format_state.set(requested)
        active_nmea = None
        active_ubx = None
        if requested == "ubx":
            active_ubx = ubx_emitter
        elif requested == "nmea":
            active_nmea = nmea_emitter
        elif requested == "ubx_nmea":
            active_nmea = nmea_emitter
            active_ubx = ubx_emitter
        active = "disabled"
        if active_nmea is not None and active_ubx is not None:
            active = "ubx_nmea"
        elif active_ubx is not None:
            active = "ubx"
        elif active_nmea is not None:
            active = "nmea"
        gps_port_mode.nmea_emitter = active_nmea
        gps_port_mode.ubx_emitter = active_ubx
        if (
            requested != last_runtime_gps_format["requested"]
            or active != last_runtime_gps_format["active"]
        ):
            print(f"GPS output format -> requested={requested} active={active}")
            last_runtime_gps_format["requested"] = requested
            last_runtime_gps_format["active"] = active
        return requested, active

    apply_gps_format_selection()

    def _print_sensor_debug(now, barometer_driver, optical_sample, interval_s):
        if now - last_report_times["baro"] < interval_s:
            return
        last_report_times["baro"] = now
        baro_parts = []
        if barometer_driver is None:
            baro_parts.append("BARO_ERROR(driver_unavailable)")
        else:
            height_m = barometer_driver.current_m
            raw_alt_m = barometer_driver.raw_alt_m
            last_msg_time = getattr(barometer_driver, "last_msg_time", None)
            if height_m is not None and math.isfinite(height_m):
                baro_parts.append(f"baro_h={height_m:.3f}m")
                if raw_alt_m is not None:
                    baro_parts.append(f"baro_alt_m={raw_alt_m:.3f}")
            else:
                reasons = []
                if last_msg_time is None:
                    reasons.append("no_messages")
                else:
                    reasons.append(f"last_msg_age_s={now - last_msg_time:.2f}")
                if raw_alt_m is None:
                    reasons.append("raw_alt_m=None")
                baro_parts.append(f"BARO_ERROR({', '.join(reasons)})")

        opt_parts = []
        if optical_sample is None:
            opt_parts.append("OPT_ERROR(no_messages)")
        else:
            dist_mm = getattr(optical_sample, "distance_mm", None)
            dist_ok = bool(getattr(optical_sample, "dist_ok", 0))
            flow_ok = bool(getattr(optical_sample, "flow_ok", 0))
            dist_m = None
            if dist_mm is not None:
                try:
                    dist_m = float(dist_mm) / 1000.0
                except Exception:
                    dist_m = None
            if dist_m is not None:
                opt_parts.append(f"opt_h={dist_m:.3f}m")
            else:
                opt_parts.append("OPT_ERROR(dist_missing)")
            opt_parts.append(f"opt_dist_ok={int(dist_ok)}")
            opt_parts.append(f"opt_flow_ok={int(flow_ok)}")
            flow_vx = getattr(optical_sample, "flow_vx", None)
            flow_vy = getattr(optical_sample, "flow_vy", None)
            if flow_vx is not None and flow_vy is not None:
                opt_parts.append(f"opt_flow=({flow_vx},{flow_vy})")
            flow_q = getattr(optical_sample, "flow_quality", None)
            if flow_q is not None:
                opt_parts.append(f"opt_q={flow_q}")

        print(f"SENSORS: {' '.join(baro_parts)} | {' '.join(opt_parts)}")

    def select_runtime_heading(active_mode, vx_enu=None, vy_enu=None):
        """Select heading source: optical flow > VO > compass."""
        compass_heading = _safe_float(last_compass_in.get("heading_deg"))
        vo_heading = _heading_from_velocity(
            vx_enu,
            vy_enu,
            min_speed_mps=heading_velocity_min_mps,
        )
        optical_heading = None
        sample = last_optical_flow.get("value")
        if (
            sample is not None
            and getattr(sample, "flow_ok", False)
            and getattr(sample, "dist_ok", False)
        ):
            optical_heading = _heading_from_velocity(
                getattr(sample, "speed_x", None),
                getattr(sample, "speed_y", None),
                min_speed_mps=heading_velocity_min_mps,
            )

        selected = None
        source = "none"
        if active_mode in optical_modes and optical_heading is not None:
            selected = optical_heading
            source = "optical_flow"
        elif vo_heading is not None:
            selected = vo_heading
            source = "vo"
        elif compass_heading is not None:
            selected = compass_heading
            source = "compass"

        if selected is not None and heading_runtime_alpha is not None:
            selected = _smooth_heading_deg(
                heading_state.get("deg"),
                selected,
                heading_runtime_alpha,
                heading_runtime_max_delta,
            )
        if selected is not None:
            heading_state["deg"] = selected
            heading_state["source"] = source
        return heading_state.get("deg"), heading_state.get("source", "none")
    optical_flow_mav_cfg = pixhawk_cfg.get("optical_flow_mavlink", {})
    flow_min_quality = int(optical_flow_mav_cfg.get("min_quality", 50))
    flow_max_speed_mps = float(optical_flow_mav_cfg.get("max_speed_mps", 2.0))
    flow_deadband_mps = float(optical_flow_mav_cfg.get("deadband_mps", 0.01))
    flow_smoothing_alpha = float(optical_flow_mav_cfg.get("smoothing_alpha", 0.2))
    flow_stationary_speed_mps = float(
        optical_flow_mav_cfg.get("stationary_speed_mps", 0.02)
    )
    flow_stationary_samples = int(optical_flow_mav_cfg.get("stationary_samples", 15))
    flow_stationary_quality_min = int(
        optical_flow_mav_cfg.get("stationary_quality_min", 40)
    )
    flow_speed_scale = float(optical_flow_mav_cfg.get("speed_scale", 1.0))
    optical_flow_scale_state.set_fallback_scale(flow_speed_scale)
    alt_smoothing_alpha = float(optical_cfg.get("altitude_smoothing_alpha", 0.18))
    alt_jump_limit_m = float(optical_cfg.get("altitude_jump_limit_m", 0.06))
    alt_deadband_m = float(optical_cfg.get("altitude_deadband_m", 0.004))
    alt_min_m = float(optical_cfg.get("altitude_min_m", 0.05))
    alt_max_m = float(optical_cfg.get("altitude_max_m", 8.0))

    if optical_raw_mode:
        flow_min_quality = 0
        flow_max_speed_mps = 1000.0
        flow_deadband_mps = 0.0
        flow_smoothing_alpha = 1.0
        flow_stationary_speed_mps = 0.0
        flow_stationary_samples = 10**9
        flow_stationary_quality_min = 255
        flow_speed_scale = 1.0
        alt_smoothing_alpha = 1.0
        alt_jump_limit_m = 1000.0
        alt_deadband_m = 0.0
        alt_min_m = -1000.0
        alt_max_m = 1000.0
        optical_flow_scale_state.set_fallback_scale(flow_speed_scale)
        print(
            "Optical flow data mode: raw_tool (raw-like output and no smoothing/clamping)"
        )

    optical_flow_gps_port_mode = OpticalFlowGpsPortMode(
        gps_port_mode=gps_port_mode,
        min_quality=flow_min_quality,
        max_speed_mps=flow_max_speed_mps,
        deadband_mps=flow_deadband_mps,
        smoothing_alpha=flow_smoothing_alpha,
        stationary_speed_mps=flow_stationary_speed_mps,
        stationary_samples=flow_stationary_samples,
        stationary_quality_min=flow_stationary_quality_min,
        speed_scale=optical_flow_scale_state.get_current_scale(flow_speed_scale),
        altitude_smoothing_alpha=alt_smoothing_alpha,
        altitude_jump_limit_m=alt_jump_limit_m,
        altitude_deadband_m=alt_deadband_m,
        altitude_min_m=alt_min_m,
        altitude_max_m=alt_max_m,
        lat_scale=float(lat_scale_state.get()),
        lon_scale=float(lon_scale_state.get()),
        alt_offset_m=float(alt_offset_state.get()),
    )
    imu_cfg = pixhawk_cfg.get("imu", {})
    optical_gps_port_imu_mode = OpticalGpsPortImuMode(
        gps_port_mode=gps_port_mode,
        min_quality=flow_min_quality,
        max_speed_mps=flow_max_speed_mps,
        deadband_mps=flow_deadband_mps,
        smoothing_alpha=flow_smoothing_alpha,
        stationary_speed_mps=flow_stationary_speed_mps,
        stationary_samples=flow_stationary_samples,
        stationary_quality_min=flow_stationary_quality_min,
        speed_scale=optical_flow_scale_state.get_current_scale(flow_speed_scale),
        altitude_smoothing_alpha=alt_smoothing_alpha,
        altitude_jump_limit_m=alt_jump_limit_m,
        altitude_deadband_m=alt_deadband_m,
        altitude_min_m=alt_min_m,
        altitude_max_m=alt_max_m,
        lat_scale=float(lat_scale_state.get()),
        lon_scale=float(lon_scale_state.get()),
        alt_offset_m=float(alt_offset_state.get()),
        imu_enabled=bool(imu_cfg.get("enabled", True)),
        imu_f_pixels=float(imu_cfg.get("f_pixels", 16.0)),
        imu_beta=float(imu_cfg.get("imu_beta", 0.05)),
        gyro_comp_enabled=bool(imu_cfg.get("gyro_comp_enabled", True)),
        accel_fuse_enabled=bool(imu_cfg.get("accel_fuse_enabled", True)),
        tilt_corr_enabled=bool(imu_cfg.get("tilt_corr_enabled", True)),
        imu_provider=lambda: dict(last_imu),
        attitude_provider=lambda: (
            dict(last_attitude["value"])
            if isinstance(last_attitude.get("value"), dict)
            else None
        ),
    )
    odometry_mode = OdometryMode(
        send_interval_s=odom_send_interval_s,
        print_interval_s=print_interval_s,
    )
    optical_flow_mav_send_interval_s = float(
        optical_flow_mav_cfg.get("send_interval_s", OPTICAL_FLOW_MAV_SEND_INTERVAL_S)
    )
    optical_flow_mav_print = bool(
        optical_flow_mav_cfg.get("print", OPTICAL_FLOW_MAV_PRINT)
    )
    optical_flow_range_min = float(
        optical_flow_mav_cfg.get("range_min_m", 0.01)
    )
    optical_flow_range_max = float(
        optical_flow_mav_cfg.get("range_max_m", 8.0)
    )
    optical_flow_vo_cfg = pixhawk_cfg.get("optical_flow_vo", {})
    optical_flow_switch_m = float(
        optical_flow_vo_cfg.get("switch_m", optical_flow_range_max)
    )
    optical_flow_output_mode = str(
        optical_flow_vo_cfg.get("optical_flow_output_mode", "optical_flow_gps_port")
    ).strip().lower()
    vo_output_mode = str(
        optical_flow_vo_cfg.get("vo_output_mode", "gps_mavlink")
    ).strip().lower()
    if hybrid_optical_vo_mode:
        if optical_flow_output_mode not in optical_modes:
            print(
                "optical_flow_vo.optical_flow_output_mode must be "
                f"{sorted(optical_modes)}; defaulting to optical_flow_gps_port."
            )
            optical_flow_output_mode = "optical_flow_gps_port"
        if vo_output_mode not in {"gps_mavlink", "gps_port", "gps_passthrough", "odometry"}:
            print(
                "optical_flow_vo.vo_output_mode must be "
                "gps_mavlink, gps_port, gps_passthrough, or odometry; "
                "defaulting to gps_mavlink."
            )
            vo_output_mode = "gps_mavlink"
    optical_flow_mode = OpticalFlowMavlinkMode(
        send_interval_s=optical_flow_mav_send_interval_s,
        print_enabled=optical_flow_mav_print,
        range_min_m=optical_flow_range_min,
        range_max_m=optical_flow_range_max,
    )

    def on_update(
        x_m,
        y_m,
        z_m,
        dx_m,
        dy_m,
        dz_m,
        dx_pixels=None,
        dy_pixels=None,
        inlier_count=None,
        inlier_ratio=None,
        flow_mad_px=None,
        *_rest,
    ):
        now = time.time()
        if optical_reader is not None:
            sample = optical_reader.get_latest()
            if sample is not None:
                last_optical_flow["value"] = sample
        if mavlink_interface is not None:
            att = mavlink_interface.recv_attitude()
            if att is not None:
                last_attitude["value"] = att
        _update_compass(now)
        if mavlink_interface is not None:
            while True:
                imu = mavlink_interface.recv_imu()
                if imu is None:
                    break
                if use_imu_fusion and fusion is not None:
                    fusion.update_imu(imu)
                last_imu.update(imu)
        if vio_mode == "vio_imu" and imu_estimator is not None and use_imu_fusion:
            att = mavlink_interface.get_last_attitude()
            if att is not None:
                imu_estimator.update_attitude(att["roll"], att["pitch"], att["yaw"])
                last_attitude["value"] = att
            while True:
                imu_msg = mavlink_interface.recv_match_safe(
                    type=["HIGHRES_IMU", "RAW_IMU"],
                    blocking=False,
                )
                if imu_msg is None:
                    break
                result = imu_estimator.process_message(imu_msg)
                if result is None:
                    continue
                vx, vy, vz, frame = result
                now = time.time()
                if (
                    vio_imu_print
                    and now - imu_last_print["time"] >= vio_imu_print_interval_s
                ):
                    print(
                        f"IMU({frame}) Vx: {vx:.3f} | Vy: {vy:.3f} | Vz: {vz:.3f} m/s"
                    )
                    imu_last_print["time"] = now
        if vio_mode == "vio_imu":
            print_interval_s = pixhawk_cfg.get("print_interval_s", PRINT_INTERVAL_S)
            last_time = last_vio_imu["time"]
            if last_time is None or now - last_time >= print_interval_s:
                if (
                    last_time is not None
                    and now > last_time
                    and last_vio_imu["x"] is not None
                    and last_vio_imu["y"] is not None
                    and last_vio_imu["z"] is not None
                ):
                    dt = now - last_time
                    vx_enu = (x_m - last_vio_imu["x"]) / dt
                    vy_enu = (y_m - last_vio_imu["y"]) / dt
                    vz_enu = (z_m - last_vio_imu["z"]) / dt
                else:
                    vx_enu = 0.0
                    vy_enu = 0.0
                    vz_enu = 0.0
                print(
                    "VO XYZ: "
                    f"X={x_m:.2f} Y={y_m:.2f} Z={z_m:.2f} | "
                    f"Vx={vx_enu:.2f} Vy={vy_enu:.2f} Vz={vz_enu:.2f} m/s"
                )
                last_vio_imu["time"] = now
                last_vio_imu["x"] = x_m
                last_vio_imu["y"] = y_m
                last_vio_imu["z"] = z_m

        gps_serial_fix = None
        if gps_serial_reader is not None:
            fix, fix_time = gps_serial_reader.read_messages()
            if fix is not None and fix.get("fix_type", 0) >= gps_serial_min_fix:
                gps_serial_fix = dict(fix)
                gps_serial_fix["time"] = fix_time or now
                if selector.gps_origin() is None:
                    selector.set_gps_origin(
                        fix["lat"], fix["lon"], fix.get("alt_m")
                    )
                selector.update_gps(
                    fix["lat"],
                    fix["lon"],
                    fix.get("alt_m"),
                    fix.get("fix_type"),
                    timestamp=fix_time or now,
                )
                last_gps_input["value"] = {
                    "lat": fix.get("lat"),
                    "lon": fix.get("lon"),
                    "alt_m": fix.get("alt_m"),
                    "fix_type": fix.get("fix_type"),
                    "time": fix_time or now,
                }
        if gps_serial_fix is None and gps_input_reader is not None:
            fix, fix_time = gps_input_reader.read_messages()
            if fix is not None and fix.get("fix_type", 0) >= gps_input_min_fix:
                selector.update_gps(
                    fix["lat"],
                    fix["lon"],
                    fix.get("alt_m"),
                    fix.get("fix_type"),
                    timestamp=fix_time or now,
                )
                last_gps_input["value"] = {
                    "lat": fix.get("lat"),
                    "lon": fix.get("lon"),
                    "alt_m": fix.get("alt_m"),
                    "fix_type": fix.get("fix_type"),
                    "time": fix_time or now,
                }

        barometer_driver = vo.height_estimator.barometer_driver
        print_sensors = pixhawk_cfg.get("print_sensor_values", PRINT_SENSOR_VALUES)
        print_interval_s = pixhawk_cfg.get("print_interval_s", PRINT_INTERVAL_S)
        if print_sensors:
            _print_sensor_debug(
                now,
                barometer_driver,
                last_optical_flow.get("value"),
                print_interval_s,
            )

        if yaw_offset_deg:
            theta = math.radians(yaw_offset_deg)
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            x_m, y_m = (
                x_m * cos_t - y_m * sin_t,
                x_m * sin_t + y_m * cos_t,
            )

        last_time = last_cam_fusion["time"]
        if (
            last_time is not None
            and now > last_time
            and last_cam_fusion["x"] is not None
            and last_cam_fusion["y"] is not None
            and last_cam_fusion["z"] is not None
        ):
            dt = now - last_time
            vx_cam = (x_m - last_cam_fusion["x"]) / dt
            vy_cam = (y_m - last_cam_fusion["y"]) / dt
            vz_cam = (z_m - last_cam_fusion["z"]) / dt
        else:
            vx_cam = 0.0
            vy_cam = 0.0
            vz_cam = 0.0
        if fusion is not None:
            fusion.update_camera(
                {
                    "x": x_m,
                    "y": y_m,
                    "z": z_m,
                    "vx": vx_cam,
                    "vy": vy_cam,
                    "vz": vz_cam,
                }
            )
        last_cam_fusion["time"] = now
        last_cam_fusion["x"] = x_m
        last_cam_fusion["y"] = y_m
        last_cam_fusion["z"] = z_m

        if fusion is not None:
            fused = fusion.fused_state()
            x_f = fused["x"]
            y_f = fused["y"]
            z_f = fused["z"]
        else:
            fused = {"x": x_m, "y": y_m, "z": z_m, "vx": None, "vy": None, "vz": None}
            x_f = x_m
            y_f = y_m
            z_f = z_m
        barometer_driver = vo.height_estimator.barometer_driver
        current_altitude_offset_m = float(altitude_offset_state.get())
        if barometer_driver is not None and barometer_driver.current_m is not None:
            z_f = barometer_driver.current_m + current_altitude_offset_m
        z_for_output = z_f

        selector.update_odometry(x_f, y_f, z_f, timestamp=now)
        if spoof_detector is not None:
            spoofed, reason = spoof_detector.update(
                gps_local=selector.gps_local(),
                gps_time=selector.gps_time(),
                gps_fix_type=selector.gps_fix_type(),
                drift_m=selector.drift_m(),
                timestamp=now,
            )
            if spoofed:
                print(f"GPS spoofing detected: {reason}")
                if spoof_reporter is not None:
                    spoof_reporter.report(
                        reason=reason,
                        gps_local=selector.gps_local(),
                        drift_m=selector.drift_m(),
                        gps_fix_type=selector.gps_fix_type(),
                        timestamp=now,
                    )
        source = selector.current_source(now)
        if source != last_source["value"]:
            drift = selector.drift_m()
            drift_text = "n/a" if drift is None else f"{drift:.2f}m"
            reason = ""
            if source == "odometry":
                if not selector.gps_available(now):
                    reason = "gps missing/invalid"
                elif drift is not None and drift > GPS_DRIFT_THRESHOLD_M:
                    reason = "gps drift high (possible spoofing)"
            if reason:
                print(f"Position source -> {source} (drift: {drift_text}, reason: {reason})")
            else:
                print(f"Position source -> {source} (drift: {drift_text})")
            last_source["value"] = source

        alt_override_m = None
        barometer_altitude_m_direct = None
        vz_override_mps = None
        if barometer_driver is not None:
            # Absolute barometer altitude (meters). This is the value shown as raw_alt_m.
            baro_abs_alt_m = _safe_float(getattr(barometer_driver, "raw_alt_m", None))
            if baro_abs_alt_m is not None:
                barometer_altitude_m_direct = float(baro_abs_alt_m)
            else:
                # Fallback only when raw absolute altitude is unavailable.
                height_source_raw = barometer_driver.current_m
                if height_source_raw is None:
                    height_source_raw = barometer_driver.get_height_m()
                if height_source_raw is not None:
                    barometer_altitude_m_direct = float(height_source_raw)

            # Keep existing behavior for generic altitude override paths.
            height_source = barometer_driver.current_m
            if height_source is None:
                height_source = barometer_driver.get_height_m()
            if height_source is not None:
                height_source = float(height_source) + current_altitude_offset_m
            alt_override_m = height_source
            last_time = last_baro["time"]
            last_height = last_baro["height_m"]
            if height_source is not None:
                if last_time is not None and last_height is not None and now > last_time:
                    vz_override_mps = (height_source - last_height) / (now - last_time)
                last_baro["time"] = now
                last_baro["height_m"] = height_source
                last_baro["vz_mps"] = vz_override_mps

        speed_accuracy_mps = _vo_speed_accuracy(inlier_ratio, flow_mad_px)
        _gps_format_requested, gps_format_active = apply_gps_format_selection()
        current_mode = mode_state.get()
        active_output_mode = current_mode
        if current_mode not in allowed_modes:
            active_output_mode = output_mode
        if current_mode == "optical_flow_then_vo":
            sample = last_optical_flow["value"]
            use_optical_flow = False
            if sample is not None and sample.dist_ok:
                distance_m = float(sample.distance_mm) / 1000.0
                if distance_m <= optical_flow_switch_m:
                    use_optical_flow = True
            active_output_mode = (
                optical_flow_output_mode if use_optical_flow else vo_output_mode
            )
        drive_source = "optical" if active_output_mode in optical_modes else "vo"
        runtime_heading_deg, runtime_heading_source = select_runtime_heading(
            active_output_mode,
            vx_enu=vx_cam,
            vy_enu=vy_cam,
        )
        if (
            current_mode != last_runtime_mode["requested"]
            or active_output_mode != last_runtime_mode["active"]
            or drive_source != last_runtime_mode["drive"]
        ):
            print(
                "Runtime mode -> "
                f"requested={current_mode} active={active_output_mode} drive={drive_source}"
            )
            last_runtime_mode["requested"] = current_mode
            last_runtime_mode["active"] = active_output_mode
            last_runtime_mode["drive"] = drive_source

        # Compass yaw used by VPS->GPS MAVLink path (config-gated behavior).
        compass_heading_deg = None
        if vps_gps_use_compass_yaw:
            compass_heading_deg = _safe_float(last_compass_in.get("heading_deg"))
        # VO GPS port path should use compass heading when available.
        vo_compass_heading_for_port = _safe_float(last_compass_in.get("heading_deg"))
        # Optical-flow GPS integration should always use compass heading when available.
        optical_compass_heading_deg = _safe_float(last_compass_in.get("heading_deg"))
        vo_scale_value = max(0.1, min(5.0, float(vo_scale_state.get())))
        vo_height_factor = 1.0
        if vo_height_scaling_enabled:
            height_for_scaling = _safe_float(
                alt_override_m if alt_override_m is not None else z_f
            )
            if height_for_scaling is not None and height_for_scaling > 0.0:
                raw_height_factor = (
                    float(height_for_scaling) / vo_height_reference_m
                ) ** vo_height_power
                vo_height_factor = max(
                    vo_height_min_factor,
                    min(vo_height_max_factor, raw_height_factor),
                )
        x_vo_output = x_f * vo_scale_value * vo_height_factor
        y_vo_output = y_f * vo_scale_value * vo_height_factor

        if active_output_mode == "gps_mavlink":
            gps_mavlink_mode.handle(
                now,
                x_vo_output,
                y_vo_output,
                z_f,
                selector.gps_origin(),
                mavlink_interface,
                alt_override_m=alt_override_m,
                vz_override_mps=vz_override_mps,
                speed_accuracy_mps=speed_accuracy_mps,
                gps_fix=gps_serial_fix,
                yaw_deg=compass_heading_deg,
            )
        elif active_output_mode == "gps_port":
            gps_port_mode.handle(
                now,
                x_vo_output,
                y_vo_output,
                z_f,
                selector.gps_origin(),
                alt_override_m=barometer_altitude_m_direct,
                heading_deg=(
                    vo_compass_heading_for_port
                    if vo_compass_heading_for_port is not None
                    else runtime_heading_deg
                ),
                send_heading=gps_output_send_heading,
                heading_only=False,
                apply_final_altitude_offset=False,
                use_origin_altitude=False,
            )
        elif active_output_mode == "gps_passthrough":
            gps_passthrough_mode.handle(now)
        elif active_output_mode == "odometry":
            odometry_mode.handle(
                now,
                x_vo_output,
                y_vo_output,
                z_for_output,
                mavlink_interface,
                mavlink_interface.get_last_attitude() if mavlink_interface else None,
            )
        elif active_output_mode == "optical_flow_mavlink":
            optical_flow_mode.handle(
                now,
                last_optical_flow["value"],
                mavlink_interface,
            )
        elif active_output_mode == "optical_flow_gps_port":
            optical_flow_gps_port_mode.set_speed_scale(
                1.0 if optical_raw_mode else optical_flow_scale_state.get_current_scale()
            )
            optical_flow_gps_port_mode.set_gps_calibration(
                lat_scale=float(lat_scale_state.get()),
                lon_scale=float(lon_scale_state.get()),
                alt_offset_m=float(alt_offset_state.get()),
            )
            optical_alt_override_m = None
            optical_sample = last_optical_flow["value"]
            if optical_sample is not None and optical_sample.dist_ok:
                optical_alt_override_m = (
                    float(optical_sample.distance_mm) / 1000.0
                    + optical_altitude_offset_m
                )
            optical_flow_gps_port_mode.handle(
                now,
                optical_sample,
                selector.gps_origin(),
                alt_override_m=optical_alt_override_m,
                heading_deg=optical_compass_heading_deg,
                send_heading=gps_output_send_heading,
                heading_only=False,
            )
        elif active_output_mode == "optical_gps_port_imu":
            optical_gps_port_imu_mode.set_speed_scale(
                1.0 if optical_raw_mode else optical_flow_scale_state.get_current_scale()
            )
            optical_gps_port_imu_mode.set_gps_calibration(
                lat_scale=float(lat_scale_state.get()),
                lon_scale=float(lon_scale_state.get()),
                alt_offset_m=float(alt_offset_state.get()),
            )
            optical_gps_port_imu_mode.handle(
                now,
                last_optical_flow["value"],
                selector.gps_origin(),
                alt_override_m=None,
                heading_deg=optical_compass_heading_deg,
                send_heading=gps_output_send_heading,
                heading_only=False,
            )

        if dashboard_state is not None:
            active_optical_gps_port_mode = (
                optical_gps_port_imu_mode
                if active_output_mode == "optical_gps_port_imu"
                else optical_flow_gps_port_mode
            )
            gps_origin = selector.gps_origin()
            gps_local = selector.gps_local()
            drift_m = selector.drift_m()
            vo_gps_port_preview = None
            if gps_origin is not None:
                vo_lat, vo_lon = selector.local_to_ll(x_vo_output, y_vo_output, gps_origin)
                vo_vel_e_mps = _safe_float(vx_cam)
                vo_vel_n_mps = _safe_float(vy_cam)
                vo_vel_d_mps = 0.0
                vo_course_deg = _heading_from_velocity(vo_vel_e_mps, vo_vel_n_mps)
                vo_heading_deg = (
                    vo_compass_heading_for_port
                    if vo_compass_heading_for_port is not None
                    else (runtime_heading_deg if runtime_heading_deg is not None else vo_course_deg)
                )
                vo_alt_m = (
                    _safe_float(barometer_altitude_m_direct)
                    if barometer_altitude_m_direct is not None
                    else _safe_float(alt_override_m)
                )
                hdop_guess = 1.3
                if gps_output_min_sats >= 18:
                    hdop_guess = 0.7
                elif gps_output_min_sats >= 15:
                    hdop_guess = 1.0
                vo_gps_port_preview = {
                    "time_s": _safe_float(now),
                    "lat": _safe_float(vo_lat),
                    "lon": _safe_float(vo_lon),
                    "alt_m": vo_alt_m,
                    "vel_n_mps": vo_vel_n_mps,
                    "vel_e_mps": vo_vel_e_mps,
                    "vel_d_mps": vo_vel_d_mps,
                    "heading_deg": _safe_float(vo_heading_deg),
                    "course_deg": _safe_float(vo_course_deg),
                    "fix_type": int(vps_gps_fix_type),
                    "satellites": int(gps_output_min_sats),
                    "hdop": _safe_float(hdop_guess),
                    "vdop": _safe_float(hdop_guess),
                    "pdop": _safe_float(hdop_guess),
                    "horizontal_accuracy_m": _safe_float(float(gps_output_ubx_h_acc_mm) / 1000.0),
                    "vertical_accuracy_m": _safe_float(float(gps_output_ubx_v_acc_mm) / 1000.0),
                    "speed_accuracy_mps": _safe_float(speed_accuracy_mps),
                }
            optical_flow_gps_port_six = _build_six_parameters_payload(
                active_optical_gps_port_mode.last_payload,
                default_timestamp=_safe_float(now),
            )
            vo_gps_port_six = _build_six_parameters_payload(
                vo_gps_port_preview,
                default_timestamp=_safe_float(now),
            )
            odom_ned = {
                "x": _safe_float(y_f),
                "y": _safe_float(x_f),
                "z": _safe_float(-z_for_output),
            }
            gps_ll = None
            if gps_origin is not None:
                lat, lon = selector.local_to_ll(x_f, y_f, gps_origin)
                alt_base = 0.0 if gps_origin[2] is None else gps_origin[2]
                gps_ll = {
                    "lat": _safe_float(lat),
                    "lon": _safe_float(lon),
                    "alt_m": _safe_float(alt_base + (0.0 if z_f is None else z_f)),
                }
            dashboard_payload = {
                    "timestamp": _safe_float(now),
                    "mode": active_output_mode,
                    "gps_output_format": gps_format_active,
                    "calibration_tuning": {
                        "lat_scale": float(lat_scale_state.get()),
                        "lon_scale": float(lon_scale_state.get()),
                        "alt_offset_m": float(alt_offset_state.get()),
                        "vo_scale": float(vo_scale_state.get()),
                        "vo_lat_scale": float(vo_lat_scale_state.get()),
                        "vo_lon_scale": float(vo_lon_scale_state.get()),
                    },
                    "vo_height_scaling": {
                        "enabled": bool(vo_height_scaling_enabled),
                        "factor": float(vo_height_factor),
                        "reference_height_m": float(vo_height_reference_m),
                        "power": float(vo_height_power),
                        "min_factor": float(vo_height_min_factor),
                        "max_factor": float(vo_height_max_factor),
                    },
                    "altitude_offset_m": current_altitude_offset_m,
                    "optical_altitude_offset_m": _safe_float(optical_altitude_offset_m),
                    "final_altitude_offset_m": _safe_float(
                        gps_port_mode.final_altitude_offset_m
                    ),
                    "heading": {
                        "deg": _safe_float(runtime_heading_deg),
                        "source": runtime_heading_source,
                    },
                    "vio_mode": vio_mode,
                    "source": source,
                    "drift_m": _safe_float(drift_m),
                    "camera": {
                        "x": _safe_float(x_m),
                        "y": _safe_float(y_m),
                        "z": _safe_float(z_m),
                        "dx": _safe_float(dx_m),
                        "dy": _safe_float(dy_m),
                        "dz": _safe_float(dz_m),
                        "vx": _safe_float(vx_cam),
                        "vy": _safe_float(vy_cam),
                        "vz": _safe_float(vz_cam),
                    },
                    "raw": {
                        "dx_px": _safe_float(dx_pixels),
                        "dy_px": _safe_float(dy_pixels),
                        "inliers": _safe_float(inlier_count),
                        "inlier_ratio": _safe_float(inlier_ratio),
                        "flow_mad_px": _safe_float(flow_mad_px),
                    },
                    "sensors": {
                        "imu": {
                            "ax": _safe_float(last_imu.get("ax")),
                            "ay": _safe_float(last_imu.get("ay")),
                            "az": _safe_float(last_imu.get("az")),
                            "gx": _safe_float(last_imu.get("gx")),
                            "gy": _safe_float(last_imu.get("gy")),
                            "gz": _safe_float(last_imu.get("gz")),
                        },
                        "attitude": {
                            "roll": _safe_float(last_attitude["value"]["roll"])
                            if last_attitude["value"]
                            else None,
                            "pitch": _safe_float(last_attitude["value"]["pitch"])
                            if last_attitude["value"]
                            else None,
                            "yaw": _safe_float(last_attitude["value"]["yaw"])
                            if last_attitude["value"]
                            else None,
                            "roll_rate": _safe_float(last_attitude["value"]["roll_rate"])
                            if last_attitude["value"]
                            else None,
                            "pitch_rate": _safe_float(last_attitude["value"]["pitch_rate"])
                            if last_attitude["value"]
                            else None,
                            "yaw_rate": _safe_float(last_attitude["value"]["yaw_rate"])
                            if last_attitude["value"]
                            else None,
                        },
                        "barometer": {
                            "height_m": _safe_float(barometer_driver.current_m)
                            if barometer_driver
                            else None,
                            "raw_press_hpa": _safe_float(barometer_driver.raw_press_hpa)
                            if barometer_driver
                            else None,
                            "raw_temp_c": _safe_float(barometer_driver.raw_temp_c)
                            if barometer_driver
                            else None,
                            "raw_alt_m": _safe_float(barometer_driver.raw_alt_m)
                            if barometer_driver
                            else None,
                        },
                        "gps_input": {
                            "lat": _safe_float(last_gps_input["value"]["lat"])
                            if last_gps_input["value"]
                            else None,
                            "lon": _safe_float(last_gps_input["value"]["lon"])
                            if last_gps_input["value"]
                            else None,
                            "alt_m": _safe_float(last_gps_input["value"]["alt_m"])
                            if last_gps_input["value"]
                            else None,
                            "fix_type": _safe_float(last_gps_input["value"]["fix_type"])
                            if last_gps_input["value"]
                            else None,
                        },
                        "optical_flow": last_optical_flow["value"].to_dict()
                        if last_optical_flow["value"]
                        else None,
                    },
                    "compass": {
                        "source": compass_meta.get("source"),
                        "message_type": compass_meta.get("message_type"),
                        "bus": _safe_float(compass_meta["bus"]),
                        "addr": _safe_float(compass_meta["addr"]),
                        "incoming": {
                            "heading_deg": _safe_float(last_compass_in["heading_deg"]),
                            "x_mg": _safe_float(last_compass_in["x_mg"]),
                            "y_mg": _safe_float(last_compass_in["y_mg"]),
                            "z_mg": _safe_float(last_compass_in["z_mg"]),
                        },
                        "outgoing": {
                            "time_boot_ms": _safe_float(last_compass_out["time_boot_ms"]),
                            "x_mg": _safe_float(last_compass_out["x_mg"]),
                            "y_mg": _safe_float(last_compass_out["y_mg"]),
                            "z_mg": _safe_float(last_compass_out["z_mg"]),
                        },
                    },
                    "fused": {
                        "x": _safe_float(x_f),
                        "y": _safe_float(y_f),
                        "z": _safe_float(z_f),
                        "vx": _safe_float(fused.get("vx")),
                        "vy": _safe_float(fused.get("vy")),
                        "vz": _safe_float(fused.get("vz")),
                    },
                    "gps": {
                        "local": {
                            "x": _safe_float(gps_local[0]) if gps_local else None,
                            "y": _safe_float(gps_local[1]) if gps_local else None,
                            "z": _safe_float(gps_local[2]) if gps_local else None,
                        },
                        "origin": {
                            "lat": _safe_float(gps_origin[0]) if gps_origin else None,
                            "lon": _safe_float(gps_origin[1]) if gps_origin else None,
                            "alt_m": _safe_float(gps_origin[2]) if gps_origin else None,
                        },
                        "fix_type": _safe_float(selector.gps_fix_type()),
                    },
                    "odom_ned": odom_ned,
                    "gps_ll_from_fused": gps_ll,
                    "outputs": {
                        "odometry": odometry_mode.last_payload,
                        "gps_mavlink": gps_mavlink_mode.last_payload,
                        "gps_port": gps_port_mode.last_payload,
                        "gps_passthrough": gps_passthrough_mode.last_payload,
                        "optical_flow_mavlink": optical_flow_mode.last_payload,
                        "optical_flow_gps_port": optical_flow_gps_port_mode.last_payload,
                        "optical_gps_port_imu": optical_gps_port_imu_mode.last_payload,
                        "optical_flow_gps_port_six_parameters": optical_flow_gps_port_six,
                        "vo_gps_port_six_parameters": vo_gps_port_six,
                    },
                }
            dashboard_state.update(dashboard_payload)
            blackbox_recorder.log_data(dashboard_payload)
            sensor_csv_recorder.log_data(dashboard_payload)

        if now - last_report["time"] >= 1.0:
            position = selector.get_position(now)
            if position is not None:
                px, py, pz = position
            last_report["time"] = now

    if use_vo_pipeline:
        print("VO + Barometer started")
        try:
            last_frame_time = {"time": 0.0}

            def frame_callback(frame):
                now = time.time()
                if now - last_frame_time["time"] < 0.1:
                    return
                last_frame_time["time"] = now
                try:
                    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                    if ok:
                        jpg_bytes = buf.tobytes()
                        frame_state.update(jpg_bytes)
                        blackbox_recorder.log_frame(jpg_bytes)
                except Exception:
                    return

            vo.run(on_update=on_update, frame_callback=frame_callback, show_window=False)
        finally:
            if optical_reader is not None:
                optical_reader.stop()
            gps_passthrough_mode.close()
            if dashboard_server is not None:
                dashboard_server.shutdown()
            if visual_slam is not None:
                visual_slam.stop()
            if slam_thread is not None:
                slam_thread.join(timeout=2.0)
            if slam_process is not None:
                slam_process.terminate()
                slam_process.join(timeout=2.0)
            if orbslam_runner is not None:
                orbslam_runner.stop()
    else:
        print("Optical flow mode started")
        try:
            while True:
                now = time.time()
                current_altitude_offset_m = float(altitude_offset_state.get())
                print_sensors = pixhawk_cfg.get("print_sensor_values", PRINT_SENSOR_VALUES)
                print_interval_s = pixhawk_cfg.get("print_interval_s", PRINT_INTERVAL_S)
                if barometer_driver is not None:
                    barometer_driver.update()
                if optical_reader is not None:
                    sample = optical_reader.get_latest()
                    if sample is not None:
                        last_optical_flow["value"] = sample
                if print_sensors:
                    _print_sensor_debug(
                        now,
                        barometer_driver,
                        last_optical_flow.get("value"),
                        print_interval_s,
                    )
                _update_compass(now)
                _gps_format_requested, gps_format_active = apply_gps_format_selection()

                current_mode = mode_state.get()
                if current_mode not in optical_modes:
                    current_mode = output_mode
                runtime_heading_deg, runtime_heading_source = select_runtime_heading(
                    current_mode,
                    vx_enu=None,
                    vy_enu=None,
                )
                drive_source = "optical"
                if (
                    current_mode != last_runtime_mode["requested"]
                    or current_mode != last_runtime_mode["active"]
                    or drive_source != last_runtime_mode["drive"]
                ):
                    print(
                        "Runtime mode -> "
                        f"requested={current_mode} active={current_mode} drive={drive_source}"
                    )
                    last_runtime_mode["requested"] = current_mode
                    last_runtime_mode["active"] = current_mode
                    last_runtime_mode["drive"] = drive_source

                gps_serial_fix = None
                if gps_serial_reader is not None:
                    fix, fix_time = gps_serial_reader.read_messages()
                    if fix is not None and fix.get("fix_type", 0) >= gps_serial_min_fix:
                        gps_serial_fix = dict(fix)
                        gps_serial_fix["time"] = fix_time or now
                        if selector.gps_origin() is None:
                            selector.set_gps_origin(
                                fix["lat"], fix["lon"], fix.get("alt_m")
                            )
                        selector.update_gps(
                            fix["lat"],
                            fix["lon"],
                            fix.get("alt_m"),
                            fix.get("fix_type"),
                            timestamp=fix_time or now,
                        )
                        last_gps_input["value"] = {
                            "lat": fix.get("lat"),
                            "lon": fix.get("lon"),
                            "alt_m": fix.get("alt_m"),
                            "fix_type": fix.get("fix_type"),
                            "time": fix_time or now,
                        }
                if gps_serial_fix is None and gps_input_reader is not None:
                    fix, fix_time = gps_input_reader.read_messages()
                    if fix is not None and fix.get("fix_type", 0) >= gps_input_min_fix:
                        selector.update_gps(
                            fix["lat"],
                            fix["lon"],
                            fix.get("alt_m"),
                            fix.get("fix_type"),
                            timestamp=fix_time or now,
                        )
                        last_gps_input["value"] = {
                            "lat": fix.get("lat"),
                            "lon": fix.get("lon"),
                            "alt_m": fix.get("alt_m"),
                            "fix_type": fix.get("fix_type"),
                            "time": fix_time or now,
                        }

                alt_override_m = None
                sample = last_optical_flow["value"]
                if sample is not None and sample.dist_ok:
                    alt_override_m = float(sample.distance_mm) / 1000.0 + optical_altitude_offset_m

                if current_mode == "optical_flow_mavlink":
                    optical_flow_mode.handle(
                        now,
                        sample,
                        mavlink_interface,
                    )
                else:
                    optical_compass_heading_deg = _safe_float(
                        last_compass_in.get("heading_deg")
                    )
                    active_optical_port_mode = (
                        optical_gps_port_imu_mode
                        if current_mode == "optical_gps_port_imu"
                        else optical_flow_gps_port_mode
                    )
                    active_optical_port_mode.set_speed_scale(
                        1.0 if optical_raw_mode else optical_flow_scale_state.get_current_scale()
                    )
                    active_optical_port_mode.set_gps_calibration(
                        lat_scale=float(lat_scale_state.get()),
                        lon_scale=float(lon_scale_state.get()),
                        alt_offset_m=float(alt_offset_state.get()),
                    )
                    active_optical_port_mode.handle(
                        now,
                        sample,
                        selector.gps_origin(),
                        alt_override_m=None if current_mode == "optical_gps_port_imu" else alt_override_m,
                        heading_deg=optical_compass_heading_deg,
                        send_heading=gps_output_send_heading,
                        heading_only=False,
                    )

                if dashboard_state is not None:
                    gps_origin = selector.gps_origin()
                    gps_local = selector.gps_local()
                    drift_m = selector.drift_m()
                    dashboard_state.update(
                        {
                            "timestamp": _safe_float(now),
                            "mode": current_mode if current_mode in optical_modes else output_mode,
                            "gps_output_format": gps_format_active,
                            "calibration_tuning": {
                                "lat_scale": float(lat_scale_state.get()),
                                "lon_scale": float(lon_scale_state.get()),
                                "alt_offset_m": float(alt_offset_state.get()),
                                "vo_scale": float(vo_scale_state.get()),
                                "vo_lat_scale": float(vo_lat_scale_state.get()),
                                "vo_lon_scale": float(vo_lon_scale_state.get()),
                            },
                            "altitude_offset_m": current_altitude_offset_m,
                            "optical_altitude_offset_m": _safe_float(optical_altitude_offset_m),
                            "final_altitude_offset_m": _safe_float(
                                gps_port_mode.final_altitude_offset_m
                            ),
                            "heading": {
                                "deg": _safe_float(runtime_heading_deg),
                                "source": runtime_heading_source,
                            },
                            "vio_mode": "vo",
                            "source": "optical_flow",
                            "drift_m": _safe_float(drift_m),
                            "camera": {
                                "x": None,
                                "y": None,
                                "z": None,
                                "dx": None,
                                "dy": None,
                                "dz": None,
                                "vx": None,
                                "vy": None,
                                "vz": None,
                            },
                            "raw": {
                                "dx_px": None,
                                "dy_px": None,
                                "inliers": None,
                                "inlier_ratio": None,
                                "flow_mad_px": None,
                            },
                            "sensors": {
                                "imu": {
                                    "ax": _safe_float(last_imu.get("ax")),
                                    "ay": _safe_float(last_imu.get("ay")),
                                    "az": _safe_float(last_imu.get("az")),
                                    "gx": _safe_float(last_imu.get("gx")),
                                    "gy": _safe_float(last_imu.get("gy")),
                                    "gz": _safe_float(last_imu.get("gz")),
                                },
                                "attitude": {
                                    "roll": _safe_float(last_attitude["value"]["roll"])
                                    if last_attitude["value"]
                                    else None,
                                    "pitch": _safe_float(last_attitude["value"]["pitch"])
                                    if last_attitude["value"]
                                    else None,
                                    "yaw": _safe_float(last_attitude["value"]["yaw"])
                                    if last_attitude["value"]
                                    else None,
                                    "roll_rate": _safe_float(last_attitude["value"]["roll_rate"])
                                    if last_attitude["value"]
                                    else None,
                                    "pitch_rate": _safe_float(last_attitude["value"]["pitch_rate"])
                                    if last_attitude["value"]
                                    else None,
                                    "yaw_rate": _safe_float(last_attitude["value"]["yaw_rate"])
                                    if last_attitude["value"]
                                    else None,
                                },
                                "barometer": {
                                    "height_m": _safe_float(barometer_driver.current_m)
                                    if barometer_driver
                                    else None,
                                    "raw_press_hpa": _safe_float(barometer_driver.raw_press_hpa)
                                    if barometer_driver
                                    else None,
                                    "raw_temp_c": _safe_float(barometer_driver.raw_temp_c)
                                    if barometer_driver
                                    else None,
                                    "raw_alt_m": _safe_float(barometer_driver.raw_alt_m)
                                    if barometer_driver
                                    else None,
                                },
                                "gps_input": {
                                    "lat": _safe_float(last_gps_input["value"]["lat"])
                                    if last_gps_input["value"]
                                    else None,
                                    "lon": _safe_float(last_gps_input["value"]["lon"])
                                    if last_gps_input["value"]
                                    else None,
                                    "alt_m": _safe_float(last_gps_input["value"]["alt_m"])
                                    if last_gps_input["value"]
                                    else None,
                                    "fix_type": _safe_float(last_gps_input["value"]["fix_type"])
                                    if last_gps_input["value"]
                                    else None,
                                },
                                "optical_flow": sample.to_dict() if sample else None,
                            },
                            "compass": {
                                "source": compass_meta.get("source"),
                                "message_type": compass_meta.get("message_type"),
                                "bus": _safe_float(compass_meta["bus"]),
                                "addr": _safe_float(compass_meta["addr"]),
                                "incoming": {
                                    "heading_deg": _safe_float(last_compass_in["heading_deg"]),
                                    "x_mg": _safe_float(last_compass_in["x_mg"]),
                                    "y_mg": _safe_float(last_compass_in["y_mg"]),
                                    "z_mg": _safe_float(last_compass_in["z_mg"]),
                                },
                                "outgoing": {
                                    "time_boot_ms": _safe_float(last_compass_out["time_boot_ms"]),
                                    "x_mg": _safe_float(last_compass_out["x_mg"]),
                                    "y_mg": _safe_float(last_compass_out["y_mg"]),
                                    "z_mg": _safe_float(last_compass_out["z_mg"]),
                                },
                            },
                            "fused": {
                                "x": None,
                                "y": None,
                                "z": _safe_float(alt_override_m) if alt_override_m is not None else None,
                                "vx": None,
                                "vy": None,
                                "vz": None,
                            },
                            "gps": {
                                "local": {
                                    "x": _safe_float(gps_local[0]) if gps_local else None,
                                    "y": _safe_float(gps_local[1]) if gps_local else None,
                                    "z": _safe_float(gps_local[2]) if gps_local else None,
                                },
                                "origin": {
                                    "lat": _safe_float(gps_origin[0]) if gps_origin else None,
                                    "lon": _safe_float(gps_origin[1]) if gps_origin else None,
                                    "alt_m": _safe_float(gps_origin[2]) if gps_origin else None,
                                },
                                "fix_type": _safe_float(selector.gps_fix_type()),
                            },
                            "odom_ned": {"x": None, "y": None, "z": None},
                            "gps_ll_from_fused": None,
                            "outputs": {
                                "odometry": odometry_mode.last_payload,
                                "gps_mavlink": gps_mavlink_mode.last_payload,
                                "gps_port": gps_port_mode.last_payload,
                                "gps_passthrough": gps_passthrough_mode.last_payload,
                                "optical_flow_mavlink": optical_flow_mode.last_payload,
                                "optical_flow_gps_port": optical_flow_gps_port_mode.last_payload,
                                "optical_gps_port_imu": optical_gps_port_imu_mode.last_payload,
                            },
                        }
                    )
                time.sleep(0.01)
        finally:
            if optical_reader is not None:
                optical_reader.stop()
            gps_passthrough_mode.close()
            if dashboard_server is not None:
                dashboard_server.shutdown()


if __name__ == "__main__":
    main()


"""
in this i want to integrate html dashboard for every value like it show first x,y,z, then make that block it coming from camera then dx,dy, is comming from drift then we got X Y vlue like there are 3 modes 1 for ODOM in that there are different blocks for different data flow how it come fom cam then convert itno NED then how the data come from for the whole format  and same for each mode and put this file in simulation folder and whenever i run the python script it also open with that

"""

"""
Done. 1500 samples in 31.5s

Calibration results (milligauss):
  offsets_mg: [37.61467889908258, 236.23853211009174, -12.385321100917452]
  scales:     [0.9061784897025169, 1.1461649782923298, 0.9765721331689271]

Paste this into config/pixhawk.yaml:
compass:
  calibration:
    offsets_mg: [37.61, 236.24, -12.39]
    scales:     [0.90618, 1.14616, 0.97657]
    axis_map:   ['+x', '+y', '+z']  # adjust if your compass axes are rotated

Calibration results (milligauss):
  offsets_mg: [7.339449541284409, 239.44954128440367, -7.339449541284409]
  scales:     [0.9617737003058106, 0.9662058371735792, 1.0807560137457046]

Paste this into config/pixhawk.yaml:
compass:
  calibration:
    offsets_mg: [7.34, 239.45, -7.34]
    scales:     [0.96177, 0.96621, 1.08076]
    axis_map:   ['+x', '+y', '+z']  # adjust if your compass axes are rotated
    
Calibration results (milligauss):
  offsets_mg: [-20.642201834862362, 179.81651376146792, -25.688073394495405]
  scales:     [1.0100376411543288, 0.8904867256637169, 1.127450980392157]

Paste this into config/pixhawk.yaml:
compass:
  calibration:
    offsets_mg: [-20.64, 179.82, -25.69]
    scales:     [1.01004, 0.89049, 1.12745]
    axis_map:   ['+x', '+y', '+z']  # adjust if your compass axes are rotated

"""
