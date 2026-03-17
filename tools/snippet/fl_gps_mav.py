#!/usr/bin/env python3
"""
fl_gps_mav.py  —  MAVLink GPS + optical-flow CSV logger
=======================================================

Behavior:
  MTF-01 flow+ToF   --> raw + scaled optical-flow values
  Pixhawk MAVLink   --> GPS, IMU, baro, heading
                    --> logs real GPS data from MAVLink bridge
                    --> logs optical-flow-derived GPS estimate
  Combined CSV      --> one row with timestamps, MAV GPS fields, and flow fields
"""

import argparse
import csv
import math
import os
import struct
import threading
import time
import serial
from datetime import datetime, timezone
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────
FLOW_PORT = "/dev/ttyAMA3"
FLOW_BAUD = 115200

MAV_PORT  = "/dev/ttyACM0"
MAV_BAUD  = 115200

LOG_DIR   = os.path.join(os.path.dirname(__file__), "logs")

UPDATE_HZ       = 10      # Combined CSV logging + flow estimate update rate
GPS_TIMEOUT     = 10.0    # Seconds to wait for real GPS before using default
FLOW_QUALITY_MIN = 50     # 0-255: reject flow measurements below this

DEFAULT_LAT  = 12.887817833333335
DEFAULT_LON  = 77.6430275
DEFAULT_ALT  = 948.2   # meters MSL

NUM_SATS_FAKE = 10

# Match tools/snippet/mavlink_compass_heading.py exactly:
# use ATTITUDE.yaw -> wrapped degrees -> wrap-aware smoothing.
HEADING_SMOOTHING_ALPHA = 0.18
HEADING_MAX_DELTA_DEG = 10.0

# Kalman filter noise parameters
# KF_Q: process noise (IMU uncertainty per second, m^2/s^2 per s)
#   Higher -> trust IMU less, position snaps to flow updates faster
# KF_R: measurement noise (flow velocity uncertainty, m^2/s^2)
#   Higher -> trust flow less, smoother but slower to track real motion
KF_Q = 0.05
KF_R = 0.02

# ─────────────────────────────────────────────────────────────
# SHARED STATE
# ─────────────────────────────────────────────────────────────
class SharedState:
    def __init__(self):
        self._lock = threading.RLock()
        self.origin_set   = False
        self.gps_healthy  = False
        self.lat          = DEFAULT_LAT
        self.lon          = DEFAULT_LON
        self.baro_alt     = DEFAULT_ALT
        self.heading_deg  = 0.0
        self.flow_vx      = 0.0   # body-forward  m/s  (already scaled)
        self.flow_vy      = 0.0   # body-right    m/s
        self.flow_quality = 0
        self.flow_ok      = False
        self.imu_ax       = 0.0   # body-forward  m/s^2  (gravity removed)
        self.imu_ay       = 0.0   # body-right    m/s^2
        self.flow_distance_mm = 0
        self.flow_distance_m = 0.0
        self.flow_strength = 0
        self.flow_precision = 0
        self.flow_distance_status = 0
        self.flow_status = 0
        self.flow_raw_vx = 0
        self.flow_raw_vy = 0
        self.flow_vn = 0.0
        self.flow_ve = 0.0
        self.flow_speed = 0.0
        self.gps_fields = {
            "gps_message_type": "",
            "gps_time_usec": "",
            "gps_time_boot_ms": "",
            "gps_fix_type": "",
            "gps_satellites_visible": "",
            "gps_hdop": "",
            "gps_vdop": "",
            "gps_latitude_deg": "",
            "gps_longitude_deg": "",
            "gps_altitude_m": "",
            "gps_alt_ellipsoid_m": "",
            "gps_relative_alt_m": "",
            "gps_speed_mps": "",
            "gps_speed_knots": "",
            "gps_speed_kmph": "",
            "gps_course_deg": "",
            "gps_heading_deg": "",
            "gps_vn_mps": "",
            "gps_ve_mps": "",
            "gps_vd_mps": "",
            "gps_hacc_m": "",
            "gps_vacc_m": "",
            "gps_vel_acc_mps": "",
            "gps_hdg_acc_deg": "",
            "gps_yaw_deg": "",
        }

    def set_origin(self, lat, lon, alt):
        with self._lock:
            if not self.origin_set:
                self.lat = lat
                self.lon = lon
                self.baro_alt = alt
                self.origin_set = True
                self.gps_healthy = True
                print(f"[Origin] Real GPS fix: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m")

    def use_default_origin(self):
        with self._lock:
            if not self.origin_set:
                self.origin_set = True
                print(f"[Origin] Default: lat={self.lat:.6f} lon={self.lon:.6f} alt={self.baro_alt:.1f}m")

    def update_flow(self, vx, vy, quality, ok, raw_vx, raw_vy, dist_mm,
                    dist_m, strength, precision, dis_status, flow_status):
        with self._lock:
            self.flow_vx = vx
            self.flow_vy = vy
            self.flow_quality = quality
            self.flow_ok = ok
            self.flow_raw_vx = raw_vx
            self.flow_raw_vy = raw_vy
            self.flow_distance_mm = dist_mm
            self.flow_distance_m = dist_m
            self.flow_strength = strength
            self.flow_precision = precision
            self.flow_distance_status = dis_status
            self.flow_status = flow_status

    def update_imu(self, ax, ay):
        with self._lock:
            self.imu_ax = ax
            self.imu_ay = ay

    def update_baro(self, alt_m):
        with self._lock:
            self.baro_alt = alt_m

    def update_heading(self, deg):
        with self._lock:
            self.heading_deg = smooth_heading_deg(
                self.heading_deg,
                deg,
                HEADING_SMOOTHING_ALPHA,
                HEADING_MAX_DELTA_DEG,
            )

    def update_position(self, lat, lon):
        with self._lock:
            self.lat = lat
            self.lon = lon

    def update_estimated_velocity(self, vn, ve):
        with self._lock:
            self.flow_vn = vn
            self.flow_ve = ve
            self.flow_speed = math.hypot(vn, ve)

    def update_gps_fields(self, updates):
        with self._lock:
            self.gps_fields.update(updates)

    def update_gps_health(self, healthy):
        with self._lock:
            self.gps_healthy = bool(healthy)

    def snapshot(self):
        with self._lock:
            snap = {
                "origin_set": self.origin_set,
                "gps_healthy": self.gps_healthy,
                "of_latitude_deg": self.lat,
                "of_longitude_deg": self.lon,
                "of_altitude_m": self.baro_alt,
                "of_heading_deg": self.heading_deg,
                "imu_ax_mps2": self.imu_ax,
                "imu_ay_mps2": self.imu_ay,
                "of_body_vx_mps": self.flow_vx,
                "of_body_vy_mps": self.flow_vy,
                "of_quality": self.flow_quality,
                "of_ok": self.flow_ok,
                "of_distance_mm": self.flow_distance_mm,
                "of_distance_m": self.flow_distance_m,
                "of_strength": self.flow_strength,
                "of_precision": self.flow_precision,
                "of_distance_status": self.flow_distance_status,
                "of_status": self.flow_status,
                "of_raw_vx_cms_at_1m": self.flow_raw_vx,
                "of_raw_vy_cms_at_1m": self.flow_raw_vy,
                "of_vn_mps": self.flow_vn,
                "of_ve_mps": self.flow_ve,
                "of_speed_mps": self.flow_speed,
            }
            snap.update(self.gps_fields)
            return snap


state = SharedState()


# ─────────────────────────────────────────────────────────────
# MAVLINK GPS HELPERS
# ─────────────────────────────────────────────────────────────
def _scaled_or_none(value, scale):
    if value in (None, "", 0xFFFF, 0xFFFFFFFF):
        return None
    return float(value) / scale


def _gps_raw_to_fields(msg):
    speed_mps = _scaled_or_none(getattr(msg, "vel", None), 100.0)
    course_deg = _scaled_or_none(getattr(msg, "cog", None), 100.0)
    fields = {
        "gps_message_type": "GPS_RAW_INT",
        "gps_time_usec": getattr(msg, "time_usec", ""),
        "gps_fix_type": getattr(msg, "fix_type", ""),
        "gps_satellites_visible": getattr(msg, "satellites_visible", ""),
        "gps_hdop": _scaled_or_none(getattr(msg, "eph", None), 100.0),
        "gps_vdop": _scaled_or_none(getattr(msg, "epv", None), 100.0),
        "gps_latitude_deg": _scaled_or_none(getattr(msg, "lat", None), 1e7),
        "gps_longitude_deg": _scaled_or_none(getattr(msg, "lon", None), 1e7),
        "gps_altitude_m": _scaled_or_none(getattr(msg, "alt", None), 1000.0),
        "gps_alt_ellipsoid_m": _scaled_or_none(getattr(msg, "alt_ellipsoid", None), 1000.0),
        "gps_speed_mps": speed_mps,
        "gps_speed_knots": speed_mps * 1.943844 if speed_mps is not None else None,
        "gps_speed_kmph": speed_mps * 3.6 if speed_mps is not None else None,
        "gps_course_deg": course_deg,
        "gps_heading_deg": course_deg,
        "gps_hacc_m": _scaled_or_none(getattr(msg, "h_acc", None), 1000.0),
        "gps_vacc_m": _scaled_or_none(getattr(msg, "v_acc", None), 1000.0),
        "gps_vel_acc_mps": _scaled_or_none(getattr(msg, "vel_acc", None), 1000.0),
        "gps_hdg_acc_deg": _scaled_or_none(getattr(msg, "hdg_acc", None), 1e5),
        "gps_yaw_deg": _scaled_or_none(getattr(msg, "yaw", None), 100.0),
    }
    return {k: v for k, v in fields.items() if v not in (None, "")}


def _global_position_to_fields(msg):
    vx = _scaled_or_none(getattr(msg, "vx", None), 100.0)
    vy = _scaled_or_none(getattr(msg, "vy", None), 100.0)
    vz = _scaled_or_none(getattr(msg, "vz", None), 100.0)
    heading_deg = _scaled_or_none(getattr(msg, "hdg", None), 100.0)
    speed_mps = math.hypot(vx or 0.0, vy or 0.0) if (vx is not None and vy is not None) else None
    fields = {
        "gps_message_type": "GLOBAL_POSITION_INT",
        "gps_time_boot_ms": getattr(msg, "time_boot_ms", ""),
        "gps_latitude_deg": _scaled_or_none(getattr(msg, "lat", None), 1e7),
        "gps_longitude_deg": _scaled_or_none(getattr(msg, "lon", None), 1e7),
        "gps_altitude_m": _scaled_or_none(getattr(msg, "alt", None), 1000.0),
        "gps_relative_alt_m": _scaled_or_none(getattr(msg, "relative_alt", None), 1000.0),
        "gps_vn_mps": vx,
        "gps_ve_mps": vy,
        "gps_vd_mps": vz,
        "gps_speed_mps": speed_mps,
        "gps_speed_knots": speed_mps * 1.943844 if speed_mps is not None else None,
        "gps_speed_kmph": speed_mps * 3.6 if speed_mps is not None else None,
        "gps_heading_deg": heading_deg,
        "gps_course_deg": heading_deg,
    }
    return {k: v for k, v in fields.items() if v not in (None, "")}


# ─────────────────────────────────────────────────────────────
# THREAD 2: MTF-01 OPTICAL FLOW READER (Micolink protocol)
# ─────────────────────────────────────────────────────────────
#
# From datasheet + protocol image:
#   flow_vel_x / flow_vel_y  are  int16  in  cm/s  at  1 m
#   speed(cm/s) = flow_vel x distance(m)          <- datasheet formula
#   speed(m/s)  = flow_vel x distance(m) / 100.0  <- divide by 100
#
# Example check:
#   flow_vel_x = 700, dist = 1.0 m -> 700 * 1.0 / 100 = 7.0 m/s  (matches max spec)
#   flow_vel_x = 100, dist = 2.0 m -> 100 * 2.0 / 100 = 2.0 m/s  (correct)

MICO_HEAD     = 0xEF
MICO_ID_RANGE = 0x51
MICO_MAX_PL   = 64


class _MsgBuf:
    __slots__ = ["status","dev_id","sys_id","msg_id","seq","length","payload","cnt","checksum"]
    def __init__(self):
        self.status  = 0
        self.payload = bytearray(MICO_MAX_PL)
        self.cnt     = 0
        self.dev_id = self.sys_id = self.msg_id = self.seq = self.length = self.checksum = 0

    def reset(self):
        self.status = 0
        self.cnt = 0


class FlowReader(threading.Thread):
    name = "FlowReader"
    daemon = True

    def __init__(self):
        super().__init__()
        self._seq = 0

    def _heartbeat(self, ser):
        ts = int(time.time() * 1000) & 0xFFFFFFFF
        msg = bytearray([0xEF, 0x01, 0x00, 0x01, self._seq & 0xFF, 0x0D])
        msg += struct.pack("<I", ts)
        msg += bytes([0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        msg.append(sum(msg) & 0xFF)
        self._seq += 1
        ser.write(bytes(msg))
        ser.flush()

    @staticmethod
    def _cksum(m):
        s = MICO_HEAD + m.dev_id + m.sys_id + m.msg_id + m.seq + m.length
        for i in range(m.length):
            s += m.payload[i]
        return s & 0xFF

    @staticmethod
    def _decode(payload_bytes, print_first=False):
        if len(payload_bytes) < 20:
            print(f"[Flow] _decode: payload too short ({len(payload_bytes)} bytes, need 20)")
            return
        # Payload layout (from protocol image):
        # uint32 time_ms, uint32 distance_mm, uint8 strength, uint8 precision,
        # uint8 dis_status, uint8 reserved, int16 flow_vel_x, int16 flow_vel_y,
        # uint8 flow_quality, uint8 flow_status, uint16 reserved
        d = struct.unpack("<IIBBBBhhBBH", payload_bytes[:24])
        dist_mm      = d[1]
        strength     = d[2]
        precision    = d[3]
        dis_status   = d[4]
        flow_vel_x   = d[6]   # int16, unit: cm/s at 1 m height
        flow_vel_y   = d[7]   # int16, unit: cm/s at 1 m height
        flow_quality = d[8]   # 0-255, higher = better
        flow_status  = d[9]   # 1 = valid, 0 = invalid

        dist_valid = (dist_mm > 0) and (dis_status == 1)
        dist_m = dist_mm / 1000.0 if dist_valid else 0.0

        if dist_valid:
            # flow_vel_x: positive = drone moves FORWARD  (no sign change needed)
            # flow_vel_y: positive = ground moves RIGHT under sensor
            #             = drone moves LEFT  → negate to get drone body +Y = right
            vx_ms = -(flow_vel_x * dist_m) / 100.0   # body forward  m/s  (negated)
            vy_ms = -(flow_vel_y * dist_m) / 100.0   # body right    m/s  (negated)
        else:
            vx_ms = 0.0
            vy_ms = 0.0

        ok = (flow_status == 1) and (flow_quality >= FLOW_QUALITY_MIN) and dist_valid

        # First successful decode — print raw values so we can see sensor state
        if print_first:
            print(
                f"[Flow] FIRST FRAME raw:"
                f"  dist_mm={dist_mm}  dis_status={dis_status}"
                f"  flow_vx={flow_vel_x}  flow_vy={flow_vel_y}"
                f"  quality={flow_quality}  flow_status={flow_status}"
                f"  dist_valid={dist_valid}  ok={ok}"
            )
            if not dist_valid:
                print("[Flow] WARNING: distance invalid — sensor too close (<8cm), too far (>8m), or no surface")
            if flow_status != 1:
                print(f"[Flow] WARNING: flow_status={flow_status} (sensor reports flow invalid)")
            if flow_quality < FLOW_QUALITY_MIN:
                print(f"[Flow] WARNING: quality={flow_quality} below threshold={FLOW_QUALITY_MIN} — poor texture or low light")
            if ok:
                print(f"[Flow] SENSOR OK -> vx={vx_ms:.3f} vy={vy_ms:.3f} m/s")

        state.update_flow(
            vx_ms, vy_ms, flow_quality, ok,
            flow_vel_x, flow_vel_y, dist_mm, dist_m,
            strength, precision, dis_status, flow_status,
        )

    def run(self):
        print(f"[Flow] Opening {FLOW_PORT} @ {FLOW_BAUD}...")
        try:
            ser = serial.Serial(
                FLOW_PORT, FLOW_BAUD,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=1,
                xonxoff=False, rtscts=False, dsrdtr=False)
        except serial.SerialException as e:
            print(f"[Flow] Cannot open: {e}")
            return

        ser.setDTR(True)
        ser.setRTS(True)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        for _ in range(3):
            self._heartbeat(ser)
            time.sleep(0.1)
        time.sleep(0.3)
        ser.reset_input_buffer()

        last_hb    = time.time()
        last_diag  = time.time()
        m = _MsgBuf()

        # Diagnostic counters — printed every 5 s to track where parsing fails
        _bytes_rx       = 0
        _frames_ok      = 0   # checksum passed + msg_id matched
        _frames_ck_fail = 0   # checksum failed
        _frames_id_miss = 0   # checksum ok but wrong msg_id
        _first_raw      = True

        print("[Flow] Streaming at 100 Hz...")

        while True:
            now = time.time()
            if now - last_hb >= 0.6:
                self._heartbeat(ser)
                last_hb = now

            # Print diagnostics every 5 s
            if now - last_diag >= 5.0:
                last_diag = now
                print(
                    f"[Flow] diag: bytes_rx={_bytes_rx}"
                    f"  frames_ok={_frames_ok}"
                    f"  ck_fail={_frames_ck_fail}"
                    f"  id_miss={_frames_id_miss}"
                )
                if _bytes_rx == 0:
                    print("[Flow] ERROR: no bytes received — check wiring / port / baud")
                elif _frames_ok == 0 and _frames_ck_fail == 0:
                    print("[Flow] WARNING: bytes arriving but no 0xEF header found — wrong baud?")
                elif _frames_ck_fail > _frames_ok:
                    print("[Flow] WARNING: mostly checksum failures — possible framing noise")

            if ser.in_waiting == 0:
                time.sleep(0.002)
                continue

            byte = ser.read(1)[0]
            _bytes_rx += 1
            s = m.status

            if s == 0:
                if byte == MICO_HEAD:
                    m.dev_id = 0
                    m.status = 1
            elif s == 1:
                m.dev_id = byte
                m.status = 2
            elif s == 2:
                m.sys_id = byte
                m.status = 3
            elif s == 3:
                m.msg_id = byte
                m.status = 4
            elif s == 4:
                m.seq = byte
                m.status = 5
            elif s == 5:
                m.length = byte
                if m.length == 0:
                    m.status = 7
                elif m.length > MICO_MAX_PL:
                    m.reset()
                else:
                    m.status = 6
            elif s == 6:
                m.payload[m.cnt] = byte
                m.cnt += 1
                if m.cnt == m.length:
                    m.cnt = 0
                    m.status = 7
            elif s == 7:
                m.checksum = byte
                m.status = 0
                calc = self._cksum(m)
                if calc == m.checksum:
                    if m.msg_id == MICO_ID_RANGE:
                        _frames_ok += 1
                        self._decode(bytes(m.payload[:m.length]), _first_raw)
                        _first_raw = False
                    else:
                        _frames_id_miss += 1
                        if _frames_id_miss <= 3:
                            print(f"[Flow] unexpected msg_id=0x{m.msg_id:02X} (expected 0x{MICO_ID_RANGE:02X})")
                else:
                    _frames_ck_fail += 1
                    if _frames_ck_fail <= 3:
                        print(f"[Flow] checksum fail: got=0x{m.checksum:02X} calc=0x{calc:02X} "
                              f"dev_id=0x{m.dev_id:02X} msg_id=0x{m.msg_id:02X} len={m.length}")
                m.reset()
            else:
                m.reset()


# ─────────────────────────────────────────────────────────────
# THREAD 3: MAVLINK BRIDGE (IMU + baro + compass)
# ─────────────────────────────────────────────────────────────
#
# ArduPilot SCALED_IMU:
#   xacc, yacc, zacc  in  milli-g  (1 mg = 0.00980665 m/s^2)
#   Body frame: X = forward, Y = right, Z = down
#   At hover (level): zacc ~ -1000 mg (1g pointing up in sensor)
#   xacc, yacc ~ 0 at hover; nonzero when tilted
#
# We use xacc (forward) and yacc (right) directly as horizontal acceleration.
# No gravity projection needed because zacc handles the vertical component,
# and ArduPilot reports body-frame accel with gravity component included.
# For horizontal-only integration we subtract the gravity projection:
#   At small tilt angles: ax_horiz ~ xacc (forward), ay_horiz ~ yacc (right)
# This is the same approximation ArduPilot's EKF uses for fast horizontal dynamics.

MG_TO_MS2 = 9.80665 / 1000.0   # milli-g -> m/s^2


def _pressure_to_alt(press_hpa, temp_c):
    if press_hpa <= 0:
        return None
    t_k = temp_c + 273.15
    return (t_k / 0.0065) * (1.0 - (press_hpa / 1013.25) ** (1.0 / 5.255))


def wrap_heading(deg):
    if deg is None:
        return None
    return (float(deg) % 360.0 + 360.0) % 360.0


def angle_delta_deg(new_deg, old_deg):
    return (new_deg - old_deg + 540.0) % 360.0 - 180.0


def smooth_heading_deg(prev_deg, new_deg, alpha, max_delta_deg=None):
    if prev_deg is None:
        return wrap_heading(new_deg)
    if new_deg is None:
        return prev_deg
    delta = angle_delta_deg(new_deg, prev_deg)
    if max_delta_deg is not None:
        limit = abs(float(max_delta_deg))
        delta = max(-limit, min(limit, delta))
    alpha = float(alpha)
    return wrap_heading(prev_deg + alpha * delta)


class MAVLinkBridge(threading.Thread):
    name = "MAVLinkBridge"
    daemon = True

    def run(self):
        print(f"[MAV] Connecting {MAV_PORT} @ {MAV_BAUD}...")
        try:
            conn = mavutil.mavlink_connection(MAV_PORT, baud=MAV_BAUD)
        except Exception as e:
            print(f"[MAV] Cannot connect: {e}")
            return

        print("[MAV] Waiting for heartbeat...")
        try:
            conn.wait_heartbeat(timeout=15)
            print("[MAV] Heartbeat received.")
        except Exception:
            print("[MAV] No heartbeat - continuing anyway")

        def req(msg_id, hz):
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, int(1_000_000 / hz), 0, 0, 0, 0, 0)

        # Request GPS + IMU for logging and flow fusion.
        req(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,     10)
        req(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
        req(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,       50)
        req(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,  20)
        req(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,      20)
        req(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3,      20)
        req(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,         20)

        baro_types = {"SCALED_PRESSURE", "SCALED_PRESSURE2", "HIGHRES_IMU"}
        imu_types  = {"SCALED_IMU"}
        heading_types = {"ATTITUDE"}
        gps_types = {"GPS_RAW_INT", "GLOBAL_POSITION_INT"}
        all_types  = list(baro_types | imu_types | heading_types | gps_types)

        # First-detection flags for startup confirmation prints
        _baro_ok = False
        _imu_ok  = False
        _heading_ok  = False
        _gps_ok = False

        print("[MAV] Streaming GPS + IMU + baro + ATTITUDE heading...")
        while True:
            msg = conn.recv_match(type=all_types, blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()

            if t in gps_types:
                if t == "GPS_RAW_INT":
                    fields = _gps_raw_to_fields(msg)
                else:
                    fields = _global_position_to_fields(msg)
                if fields:
                    state.update_gps_fields(fields)
                    lat = fields.get("gps_latitude_deg")
                    lon = fields.get("gps_longitude_deg")
                    alt = fields.get("gps_altitude_m")
                    fix_type = fields.get("gps_fix_type")
                    if fix_type in ("", None):
                        fix_type = state.gps_fields.get("gps_fix_type", 0)
                    gps_healthy = fix_type not in ("", None) and int(fix_type) >= 2
                    state.update_gps_health(gps_healthy)
                    if (
                        not state.origin_set
                        and lat is not None
                        and lon is not None
                        and alt is not None
                        and gps_healthy
                    ):
                        state.set_origin(lat, lon, alt)
                    if not _gps_ok and lat is not None and lon is not None:
                        _gps_ok = True
                        print(
                            f"[MAV] GPS OK        -> lat={lat:.6f} lon={lon:.6f} "
                            f"alt={alt if alt is not None else float('nan'):.1f}m  (src={t})"
                        )

            if t in baro_types:
                if t == "HIGHRES_IMU":
                    alt = _pressure_to_alt(msg.abs_pressure, msg.temperature)
                else:
                    alt = _pressure_to_alt(msg.press_abs, msg.temperature / 100.0)
                if alt is not None:
                    state.update_baro(alt)
                    if not _baro_ok:
                        _baro_ok = True
                        print(f"[MAV] Barometer OK  -> alt={alt:.1f}m  (src={t})")

            if t in imu_types:
                # xacc = body forward (milli-g), yacc = body right (milli-g)
                ax_ms2 = msg.xacc * MG_TO_MS2
                ay_ms2 = msg.yacc * MG_TO_MS2
                state.update_imu(ax_ms2, ay_ms2)
                if not _imu_ok:
                    _imu_ok = True
                    print(f"[MAV] IMU OK        -> ax={ax_ms2:.3f} ay={ay_ms2:.3f} m/s^2  (src={t})")

            if t in heading_types:
                yaw_rad = getattr(msg, "yaw", None)
                if yaw_rad is not None:
                    h = wrap_heading(float(yaw_rad) * 180.0 / math.pi)
                    state.update_heading(h)
                    if not _heading_ok:
                        _heading_ok = True
                        print(
                            f"[MAV] Heading OK    -> heading={h:.1f}deg  "
                            f"yaw_rad={float(yaw_rad):.4f}  (src={t})"
                        )


# ─────────────────────────────────────────────────────────────
# KALMAN FILTER: 1-D velocity (run independently for N and E)
# ─────────────────────────────────────────────────────────────
#
# Exactly what ArduPilot EKF does for velocity states:
#   Predict with IMU (high rate, accumulates error)
#   Correct with flow (lower noise, absolute velocity reference)
#
class VelocityKF:
    def __init__(self, q=KF_Q, r=KF_R):
        self.v = 0.0    # velocity estimate (m/s)
        self.P = 1.0    # error covariance
        self.q = q      # process noise density (m^2/s^2 per second)
        self.r = r      # measurement noise variance (m^2/s^2)

    def predict(self, accel_ms2, dt):
        """Advance state forward using IMU acceleration."""
        self.v += accel_ms2 * dt
        self.P += self.q * dt           # covariance grows with time

    def update(self, v_measured):
        """Correct with a flow velocity measurement."""
        K     = self.P / (self.P + self.r)
        self.v = self.v + K * (v_measured - self.v)
        self.P = (1.0 - K) * self.P    # covariance shrinks after update

    @property
    def velocity(self):
        return self.v


kf_N = VelocityKF()   # North velocity
kf_E = VelocityKF()   # East  velocity


def _body_to_ned(ax_body, ay_body, heading_deg):
    """
    Rotate body-frame acceleration to NED.
    Body frame: X=forward, Y=right
    NED:        N=north,   E=east
    """
    psi = math.radians(heading_deg)
    aN  =  ax_body * math.cos(psi) - ay_body * math.sin(psi)
    aE  =  ax_body * math.sin(psi) + ay_body * math.cos(psi)
    return aN, aE


def _flow_to_ned(vx_body, vy_body, heading_deg):
    """Rotate body-frame optical flow velocity to NED."""
    psi = math.radians(heading_deg)
    vN  =  vx_body * math.cos(psi) - vy_body * math.sin(psi)
    vE  =  vx_body * math.sin(psi) + vy_body * math.cos(psi)
    return vN, vE


CSV_FIELDS = [
    "timestamp_utc",
    "timestamp_unix",
    "origin_set",
    "gps_healthy",
    "gps_message_type",
    "gps_time_usec",
    "gps_time_boot_ms",
    "gps_fix_type",
    "gps_satellites_visible",
    "gps_hdop",
    "gps_vdop",
    "gps_latitude_deg",
    "gps_longitude_deg",
    "gps_altitude_m",
    "gps_alt_ellipsoid_m",
    "gps_relative_alt_m",
    "gps_speed_mps",
    "gps_speed_knots",
    "gps_speed_kmph",
    "gps_course_deg",
    "gps_heading_deg",
    "gps_vn_mps",
    "gps_ve_mps",
    "gps_vd_mps",
    "gps_hacc_m",
    "gps_vacc_m",
    "gps_vel_acc_mps",
    "gps_hdg_acc_deg",
    "gps_yaw_deg",
    "of_latitude_deg",
    "of_longitude_deg",
    "of_altitude_m",
    "of_heading_deg",
    "imu_ax_mps2",
    "imu_ay_mps2",
    "of_distance_mm",
    "of_distance_m",
    "of_strength",
    "of_precision",
    "of_distance_status",
    "of_status",
    "of_quality",
    "of_ok",
    "of_raw_vx_cms_at_1m",
    "of_raw_vy_cms_at_1m",
    "of_body_vx_mps",
    "of_body_vy_mps",
    "of_vn_mps",
    "of_ve_mps",
    "of_speed_mps",
]


def _make_log_path():
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(LOG_DIR, f"gps_flow_log_{stamp}.csv")


def run_main_loop(log_path):
    dt_target = 1.0 / UPDATE_HZ
    last_t    = time.time()
    loop_n    = 0

    print("[Main] Waiting for origin to be set...")
    while not state.origin_set:
        time.sleep(0.05)
    print(f"[Main] Origin ready - logging combined GPS + flow data to {log_path}")

    with open(log_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()

        while True:
            now_t = time.time()
            dt    = max(0.001, min(now_t - last_t, 0.5))  # clamp dt to [1ms, 500ms]
            last_t = now_t

            with state._lock:
                ax_b    = state.imu_ax
                ay_b    = state.imu_ay
                heading = state.heading_deg
                vx_b    = state.flow_vx
                vy_b    = state.flow_vy
                flow_ok = state.flow_ok
                quality = state.flow_quality

            if flow_ok and quality >= FLOW_QUALITY_MIN:
                aN, aE = _body_to_ned(ax_b, ay_b, heading)
                kf_N.predict(aN, dt)
                kf_E.predict(aE, dt)
                vN_flow, vE_flow = _flow_to_ned(vx_b, vy_b, heading)
                kf_N.update(vN_flow)
                kf_E.update(vE_flow)
            else:
                decay = math.exp(-dt / 0.3)
                kf_N.v *= decay
                kf_E.v *= decay
                kf_N.P = 1.0
                kf_E.P = 1.0
                vN_flow, vE_flow = _flow_to_ned(vx_b, vy_b, heading)

            vN = max(-7.0, min(7.0, kf_N.velocity))
            vE = max(-7.0, min(7.0, kf_E.velocity))
            state.update_estimated_velocity(vN, vE)

            snap = state.snapshot()
            utc = datetime.now(timezone.utc)
            row = {field: snap.get(field, "") for field in CSV_FIELDS}
            row["timestamp_utc"] = utc.isoformat()
            row["timestamp_unix"] = f"{now_t:.6f}"
            row["of_vn_mps"] = vN
            row["of_ve_mps"] = vE
            row["of_speed_mps"] = math.hypot(vN, vE)
            writer.writerow(row)
            csv_file.flush()

            loop_n += 1
            if loop_n % max(1, int(UPDATE_HZ * 2)) == 0:
                origin_src = "real GPS" if snap["gps_healthy"] else "default"
                flow_str   = f"OK q={quality}" if flow_ok else f"BAD q={quality}"
                print(
                    f"[{utc.strftime('%H:%M:%S')}]"
                    f"  gps=({snap['gps_latitude_deg']}, {snap['gps_longitude_deg']})"
                    f"  flow_body=({snap['of_body_vx_mps']:+.3f}, {snap['of_body_vy_mps']:+.3f})m/s"
                    f"  flow_ned=({vN:+.3f}, {vE:+.3f})m/s"
                    f"  hdg={snap['of_heading_deg']:.1f}deg"
                    f"  flow={flow_str}"
                    f"  origin={origin_src}"
                )

            used = time.time() - now_t
            time.sleep(max(0.0, dt_target - used))


# ─────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────
def main():
    global FLOW_PORT, FLOW_BAUD
    global MAV_PORT, MAV_BAUD
    global UPDATE_HZ, LOG_DIR

    parser = argparse.ArgumentParser(
        description="Log MAVLink GPS data and optical-flow GPS estimate to one CSV")
    parser.add_argument("--no-real-gps", action="store_true",
                        help="Skip MAVLink GPS origin; use default origin immediately")
    parser.add_argument("--flow-port", default=FLOW_PORT)
    parser.add_argument("--flow-baud", type=int, default=FLOW_BAUD)
    parser.add_argument("--mav-port", default=MAV_PORT)
    parser.add_argument("--mav-baud", type=int, default=MAV_BAUD)
    parser.add_argument("--default-lat",  type=float, default=DEFAULT_LAT)
    parser.add_argument("--default-lon",  type=float, default=DEFAULT_LON)
    parser.add_argument("--default-alt",  type=float, default=DEFAULT_ALT)
    parser.add_argument("--kf-q", type=float, default=KF_Q,
                        help="Kalman process noise (IMU trust). Default 0.05")
    parser.add_argument("--kf-r", type=float, default=KF_R,
                        help="Kalman measurement noise (flow trust). Default 0.02")
    parser.add_argument("--update-hz", type=float, default=UPDATE_HZ,
                        help="Combined CSV logging rate (default 10 Hz)")
    parser.add_argument("--log-dir", default=LOG_DIR,
                        help="Directory for the combined GPS/flow CSV log")
    args = parser.parse_args()

    FLOW_PORT = args.flow_port
    FLOW_BAUD = args.flow_baud
    MAV_PORT = args.mav_port
    MAV_BAUD = args.mav_baud
    state.lat      = args.default_lat
    state.lon      = args.default_lon
    state.baro_alt = args.default_alt
    kf_N.q = kf_E.q = args.kf_q
    kf_N.r = kf_E.r = args.kf_r
    UPDATE_HZ = args.update_hz
    LOG_DIR = args.log_dir

    print("=" * 64)
    print("  MAVLink GPS + Optical Flow Combined Logger")
    print(f"  KF_Q={args.kf_q}  KF_R={args.kf_r}  LOG={args.update_hz}Hz")
    print(f"  FLOW={FLOW_PORT}@{FLOW_BAUD}")
    print(f"  MAV={MAV_PORT}@{MAV_BAUD}")
    print("=" * 64)

    log_path = _make_log_path()

    if args.no_real_gps:
        state.use_default_origin()

    FlowReader().start()
    MAVLinkBridge().start()
    time.sleep(2.5)   # let MAVLink connect + first sensor readings arrive
    if not args.no_real_gps:
        deadline = time.time() + GPS_TIMEOUT
        while not state.origin_set and time.time() < deadline:
            time.sleep(0.05)
        if not state.origin_set:
            print("[GPS] MAVLink GPS timeout -> using default origin")
            state.use_default_origin()
    run_main_loop(log_path)


if __name__ == "__main__":
    main()
