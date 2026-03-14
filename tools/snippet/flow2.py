#!/usr/bin/env python3
"""
flow_gps.py  —  Optical Flow + IMU -> Fake GPS for Pixhawk
==========================================================

Pipeline (mirrors what ArduPilot's EKF does internally):

  MTF-01 (flow+ToF)  -->  vx, vy  (cm/s @ 1m, int16)
                                            |
  Pixhawk IMU (MAVLink) -->  ax, ay (milli-g, body frame)
                                            |
                                   [Kalman Filter 2D]
                                   Predict:  IMU accel
                                   Update:   Flow velocity
                                            |
  Pixhawk Baro (MAVLink) -->  alt_m        |
  Compass (MAVLink)      -->  heading      |
                                            v
                              Position Integrator
                              (Kalman vN,vE -> lat,lon)
                                            |
  Real GPS (NMEA) --> origin once          |
                                            v
                              GPSEmitter -> UBX + NMEA
                              -> Pixhawk GPS serial port

MTF-01 velocity scale (datasheet):
  flow_vel_x/y  are  int16  in  cm/s  at  1 m  height
  Formula:  actual_speed_cm_s = flow_vel_raw  x  distance_m
  To m/s:   vx = flow_vel_x * distance_m / 100.0
  Max spec: 7 m/s @ 1 m  =>  flow_vel_max = 700

Kalman filter (1-D per axis, identical structure):
  State    v  = velocity (m/s)
  Predict  v' = v + a * dt        (IMU accel, body->NED rotated)
           P' = P + Q * dt
  Update   K  = P' / (P' + R)
           v  = v' + K * (z - v') (z = flow velocity in NED)
           P  = (1 - K) * P'

Ports (edit to match your wiring):
  MTF-01            /dev/ttyAMA3   115200
  Real GPS (NMEA)   /dev/ttyAMA5   230400   (origin only)
  Pixhawk MAVLink   /dev/ttyACM0   115200   (IMU + baro + compass)
  Pixhawk GPS port  /dev/ttyAMA0   230400   (output)
"""

import argparse
import math
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

GPS_PORT  = "/dev/ttyAMA5"
GPS_BAUD  = 230400

MAV_PORT  = "/dev/ttyACM1"
MAV_BAUD  = 115200

OUT_PORT  = "/dev/ttyAMA0"
OUT_BAUD  = 230400

UPDATE_HZ       = 10      # GPS packet output rate to Pixhawk
GPS_TIMEOUT     = 10.0    # Seconds to wait for real GPS before using default
FLOW_QUALITY_MIN = 50     # 0-255: reject flow measurements below this

DEFAULT_LAT  = 12.971600
DEFAULT_LON  = 77.594600
DEFAULT_ALT  = 920.0     # meters MSL

NUM_SATS_FAKE = 10

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

    def update_flow(self, vx, vy, quality, ok):
        with self._lock:
            self.flow_vx = vx
            self.flow_vy = vy
            self.flow_quality = quality
            self.flow_ok = ok

    def update_imu(self, ax, ay):
        with self._lock:
            self.imu_ax = ax
            self.imu_ay = ay

    def update_baro(self, alt_m):
        with self._lock:
            self.baro_alt = alt_m

    def update_heading(self, deg):
        with self._lock:
            self.heading_deg = deg

    def update_position(self, lat, lon):
        with self._lock:
            self.lat = lat
            self.lon = lon


state = SharedState()


# ─────────────────────────────────────────────────────────────
# THREAD 1: GPS ORIGIN READER (NMEA)
# ─────────────────────────────────────────────────────────────
def _nmea_checksum_ok(sentence):
    if "*" not in sentence:
        return True
    body, cs = sentence[1:].rsplit("*", 1)
    calc = 0
    for c in body:
        calc ^= ord(c)
    return f"{calc:02X}" == cs.strip().upper()


def _nmea_to_deg(value, direction):
    v = float(value)
    d = int(v / 100)
    m = v - d * 100
    deg = d + m / 60.0
    return -deg if direction in ("S", "W") else deg


def _parse_gga(sentence):
    """Returns (lat, lon, alt_m, fix_quality) or None."""
    try:
        if not _nmea_checksum_ok(sentence):
            return None
        p = sentence.split(",")
        if len(p) < 10 or not p[2] or not p[4]:
            return None
        fix = int(p[6]) if p[6] else 0
        lat = _nmea_to_deg(p[2], p[3])
        lon = _nmea_to_deg(p[4], p[5])
        alt = float(p[9]) if p[9] else 0.0
        return lat, lon, alt, fix
    except Exception:
        return None


class GPSOriginReader(threading.Thread):
    name = "GPSOriginReader"
    daemon = True

    def run(self):
        deadline = time.time() + GPS_TIMEOUT
        print(f"[GPS] Waiting {GPS_TIMEOUT}s for real GPS fix on {GPS_PORT}...")
        try:
            ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1.0)
        except serial.SerialException as e:
            print(f"[GPS] Cannot open {GPS_PORT}: {e}  -> using default origin")
            state.use_default_origin()
            return

        buf = b""
        while True:
            try:
                buf += ser.read(256)
            except serial.SerialException:
                break
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("ascii", errors="ignore").strip()
                if s.startswith(("$GPGGA", "$GNGGA")):
                    r = _parse_gga(s)
                    if r and r[3] >= 1:
                        state.set_origin(r[0], r[1], r[2])
                        return
            if not state.origin_set and time.time() > deadline:
                print("[GPS] Timeout -> using default origin")
                state.use_default_origin()
                return


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
        dis_status   = d[4]
        flow_vel_x   = d[6]   # int16, unit: cm/s at 1 m height
        flow_vel_y   = d[7]   # int16, unit: cm/s at 1 m height
        flow_quality = d[8]   # 0-255, higher = better
        flow_status  = d[9]   # 1 = valid, 0 = invalid

        dist_valid = (dist_mm > 0) and (dis_status == 1)

        if dist_valid:
            dist_m = dist_mm / 1000.0
            vx_ms = (flow_vel_x * dist_m) / 100.0
            vy_ms = (flow_vel_y * dist_m) / 100.0
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

        state.update_flow(vx_ms, vy_ms, flow_quality, ok)

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


def _heading_from_mag(xmag, ymag):
    if abs(xmag) < 1e-9 and abs(ymag) < 1e-9:
        return None
    return (math.degrees(math.atan2(ymag, xmag)) + 360.0) % 360.0


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

        # Request high-rate IMU for good prediction, 20 Hz baro/compass
        req(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,       50)
        req(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,  20)
        req(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,      20)
        req(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3,      20)

        baro_types = {"SCALED_PRESSURE", "SCALED_PRESSURE2", "HIGHRES_IMU"}
        imu_types  = {"SCALED_IMU"}
        mag_types  = {"SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3", "RAW_IMU"}
        all_types  = list(baro_types | imu_types | mag_types)

        # First-detection flags for startup confirmation prints
        _baro_ok = False
        _imu_ok  = False
        _mag_ok  = False

        print("[MAV] Streaming IMU (50Hz) + baro + compass (20Hz)...")
        while True:
            msg = conn.recv_match(type=all_types, blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()

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

            if t in mag_types:
                xm = float(getattr(msg, "xmag", 0.0))
                ym = float(getattr(msg, "ymag", 0.0))
                if xm != 0.0 or ym != 0.0:
                    h = _heading_from_mag(xm, ym)
                    if h is not None:
                        state.update_heading(h)
                        if not _mag_ok:
                            _mag_ok = True
                            print(f"[MAV] Compass OK    -> heading={h:.1f}deg  (src={t})")


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


# ─────────────────────────────────────────────────────────────
# UBX / NMEA BUILDERS
# ─────────────────────────────────────────────────────────────
def _ubx_cksum(msg_class, msg_id, payload):
    ck_a = ck_b = 0
    for b in [msg_class, msg_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF, *payload]:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def _ubx(cls, mid, payload):
    ca, cb = _ubx_cksum(cls, mid, payload)
    return struct.pack("<BBBBH", 0xB5, 0x62, cls, mid, len(payload)) + payload + bytes([ca, cb])


def _ubx_nav_pvt(lat, lon, alt, spd, hdg, nsats, now):
    h = math.radians(hdg)
    tow = (now.hour * 3600 + now.minute * 60 + now.second) * 1000 + now.microsecond // 1000
    pl = struct.pack(
        "<IHBBBBBBIiBBBBiiiiIIiiiiiIIHBBBBBBi",
        tow,
        now.year, now.month, now.day,
        now.hour, now.minute, now.second,
        0x07,               # valid: time+date+fully resolved
        0,                  # tAcc
        0,                  # nano
        3,                  # gpsFix = 3D fix
        0x01, 0x00,         # flags, flags2
        nsats,
        int(lon * 1e7), int(lat * 1e7),
        int(alt * 1000),    # altEllipsoid (mm)
        int(alt * 1000),    # altMSL (mm)
        500,                # hAcc (mm)
        1000,               # vAcc (mm)
        int(spd * 1000 * math.cos(h)),   # velN (mm/s)
        int(spd * 1000 * math.sin(h)),   # velE (mm/s)
        0,                  # velD (mm/s)
        int(spd * 1000),    # gSpeed (mm/s)
        int(hdg * 1e5),     # headMot
        1000,               # sAcc (mm/s)
        10000,              # headAcc
        0,                  # pDOP
        0, 0, 0, 0, 0, 0,   # reserved / flags3
        0,                  # headVeh
    )
    return _ubx(0x01, 0x07, pl)


def _ubx_nav_posllh(lat, lon, alt, tow):
    pl = struct.pack(
        "<IiiiiII",
        tow,
        int(lon * 1e7), int(lat * 1e7),
        int(alt * 1000), int(alt * 1000),
        2000, 3000)
    return _ubx(0x01, 0x02, pl)


def _ubx_nav_velned(spd, hdg, tow):
    h = math.radians(hdg)
    pl = struct.pack(
        "<IiiiIIiII",
        tow,
        int(spd * 100 * math.cos(h)),
        int(spd * 100 * math.sin(h)),
        0,
        int(spd * 100), int(spd * 100),
        int(hdg * 1e5),
        50, 5000)
    return _ubx(0x01, 0x12, pl)


def _nmea_ck(s):
    c = 0
    for ch in s:
        c ^= ord(ch)
    return f"{c:02X}"


def _nmea_sentences(lat, lon, alt, spd, hdg, nsats, now):
    t = now.strftime("%H%M%S") + f".{now.microsecond // 10000:02d}"
    d = now.strftime("%d%m%y")

    def d2n(deg, is_lat):
        dd = int(abs(deg))
        m  = (abs(deg) - dd) * 60
        fmt = f"{dd:02d}{m:07.4f}" if is_lat else f"{dd:03d}{m:07.4f}"
        ns = ("N" if deg >= 0 else "S") if is_lat else ("E" if deg >= 0 else "W")
        return fmt, ns

    la, lad = d2n(lat, True)
    lo, lod = d2n(lon, False)

    gga = f"GPGGA,{t},{la},{lad},{lo},{lod},1,{nsats:02d},0.8,{alt:.1f},M,-34.0,M,,"
    rmc = f"GPRMC,{t},A,{la},{lad},{lo},{lod},{spd * 1.94384:.2f},{hdg:.1f},{d},,,A"

    return (f"${gga}*{_nmea_ck(gga)}\r\n").encode(), (f"${rmc}*{_nmea_ck(rmc)}\r\n").encode()


# ─────────────────────────────────────────────────────────────
# MAIN LOOP: INTEGRATOR + GPS EMITTER
# ─────────────────────────────────────────────────────────────
def run_main_loop():
    print(f"[Out] Opening output port {OUT_PORT} @ {OUT_BAUD}...")
    try:
        out = serial.Serial(OUT_PORT, OUT_BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"[Out] Cannot open: {e}")
        return

    dt_target = 1.0 / UPDATE_HZ
    last_t    = time.time()
    loop_n    = 0

    print("[Main] Waiting for origin to be set...")
    while not state.origin_set:
        time.sleep(0.05)
    print("[Main] Origin ready - starting Kalman integration loop")

    while True:
        now_t = time.time()
        dt    = max(0.001, min(now_t - last_t, 0.5))  # clamp dt to [1ms, 500ms]
        last_t = now_t

        # Snapshot sensor state (single lock)
        with state._lock:
            ax_b    = state.imu_ax
            ay_b    = state.imu_ay
            heading = state.heading_deg
            vx_b    = state.flow_vx
            vy_b    = state.flow_vy
            flow_ok = state.flow_ok
            quality = state.flow_quality
            lat     = state.lat
            lon     = state.lon
            alt     = state.baro_alt

        # ── 1. Kalman PREDICT using IMU (body -> NED) ────────
        # Only run IMU prediction when flow is valid.
        # Reason: Pixhawk IMU has ~5-10 mg bias at rest. With no flow
        # correction this integrates into ~0.5 m/s drift per second.
        # When flow is bad we have NO absolute velocity reference, so
        # the only safe assumption is the drone is stationary (v=0).
        if flow_ok and quality >= FLOW_QUALITY_MIN:
            aN, aE = _body_to_ned(ax_b, ay_b, heading)
            kf_N.predict(aN, dt)
            kf_E.predict(aE, dt)

            # ── 2. Kalman UPDATE using optical flow ──────────
            vN_flow, vE_flow = _flow_to_ned(vx_b, vy_b, heading)
            kf_N.update(vN_flow)
            kf_E.update(vE_flow)
        else:
            # Flow lost: decay velocity toward zero (exponential drag).
            # Tau=0.3s means velocity halves in ~0.2s — fast enough to
            # stop drift, slow enough not to jerk if flow briefly drops.
            decay = math.exp(-dt / 0.3)
            kf_N.v *= decay
            kf_E.v *= decay
            # Also reset covariance so next flow update is trusted quickly
            kf_N.P = 1.0
            kf_E.P = 1.0

        vN = max(-7.0, min(7.0, kf_N.velocity))   # clamp to MTF-01 max
        vE = max(-7.0, min(7.0, kf_E.velocity))

        # ── 3. Integrate position (dead-reckoning) ───────────
        # Haversine-lite: metres per degree along each axis
        dlat = (vN * dt) / 111_111.0
        dlon = (vE * dt) / (111_111.0 * math.cos(math.radians(lat)))
        state.update_position(lat + dlat, lon + dlon)

        with state._lock:
            lat = state.lat
            lon = state.lon

        speed_mps = math.hypot(vN, vE)

        # ── 4. Build and send GPS packets ────────────────────
        utc = datetime.now(timezone.utc)
        tow = (utc.hour * 3600 + utc.minute * 60 + utc.second) * 1000

        out.write(_ubx_nav_pvt(lat, lon, alt, speed_mps, heading, NUM_SATS_FAKE, utc))
        out.write(_ubx_nav_posllh(lat, lon, alt, tow))
        out.write(_ubx_nav_velned(speed_mps, heading, tow))
        gga, rmc = _nmea_sentences(lat, lon, alt, speed_mps, heading, NUM_SATS_FAKE, utc)
        out.write(gga)
        out.write(rmc)

        # ── 5. Console status (every 2 s) ────────────────────
        loop_n += 1
        if loop_n % (UPDATE_HZ * 2) == 0:
            origin_src = "real GPS" if state.gps_healthy else "default"
            flow_str   = f"OK q={quality}" if flow_ok else f"BAD q={quality}"
            print(
                f"[{utc.strftime('%H:%M:%S')}]"
                f"  lat={lat:.6f}  lon={lon:.6f}  alt={alt:.1f}m"
                f"  vN={vN:+.3f}  vE={vE:+.3f}  spd={speed_mps:.2f}m/s"
                f"  hdg={heading:.1f}deg"
                f"  flow={flow_str}"
                f"  KF_P={kf_N.P:.4f}"
                f"  origin={origin_src}"
            )

        # ── 6. Sleep remainder of tick ────────────────────────
        used = time.time() - now_t
        time.sleep(max(0.0, dt_target - used))


# ─────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Optical Flow + IMU Kalman -> GPS emulator for Pixhawk")
    parser.add_argument("--no-real-gps", action="store_true",
                        help="Skip real GPS; use default origin immediately")
    parser.add_argument("--default-lat",  type=float, default=DEFAULT_LAT)
    parser.add_argument("--default-lon",  type=float, default=DEFAULT_LON)
    parser.add_argument("--default-alt",  type=float, default=DEFAULT_ALT)
    parser.add_argument("--kf-q", type=float, default=KF_Q,
                        help="Kalman process noise (IMU trust). Default 0.05")
    parser.add_argument("--kf-r", type=float, default=KF_R,
                        help="Kalman measurement noise (flow trust). Default 0.02")
    parser.add_argument("--update-hz", type=float, default=UPDATE_HZ,
                        help="GPS packet output rate (default 10 Hz)")
    args = parser.parse_args()

    state.lat      = args.default_lat
    state.lon      = args.default_lon
    state.baro_alt = args.default_alt
    kf_N.q = kf_E.q = args.kf_q
    kf_N.r = kf_E.r = args.kf_r

    print("=" * 64)
    print("  Optical Flow + IMU Kalman  ->  GPS Emulator")
    print(f"  KF_Q={args.kf_q}  KF_R={args.kf_r}  OUT={args.update_hz}Hz")
    print("=" * 64)

    if not args.no_real_gps:
        GPSOriginReader().start()
    else:
        state.use_default_origin()

    FlowReader().start()
    MAVLinkBridge().start()
    time.sleep(2.5)   # let MAVLink connect + first sensor readings arrive
    run_main_loop()


if __name__ == "__main__":
    main()