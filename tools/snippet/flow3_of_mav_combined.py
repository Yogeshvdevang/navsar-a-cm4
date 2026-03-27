#!/usr/bin/env python3
"""Combined standalone script:

1. `flow3.py` logic:
   Optical flow + IMU + baro + heading -> fake GPS out over serial.
2. `of_mav2.py` logic:
   The same MTF-01 sample -> MAVLink OPTICAL_FLOW + DISTANCE_SENSOR.

This file is standalone and avoids collisions by using one shared optical-flow
reader and one MAVLink bridge thread that owns all Pixhawk MAVLink I/O.

Dependencies:
  pip install pyserial pymavlink
"""

import argparse
import math
import struct
import threading
import time
from datetime import datetime, timezone

import serial
from pymavlink import mavutil


FLOW_QUALITY_MIN = 50
MG_TO_MS2 = 9.80665 / 1000.0
FOCAL_LENGTH_PX = 16.0

MICOLINK_MSG_HEAD = 0xEF
MICOLINK_MSG_ID_RANGE_SENSOR = 0x51
MICOLINK_MAX_PAYLOAD_LEN = 64


def _bool_arg(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


def _pressure_to_alt(press_hpa, temp_c):
    if press_hpa <= 0:
        return None
    t_k = temp_c + 273.15
    return (t_k / 0.0065) * (1.0 - (press_hpa / 1013.25) ** (1.0 / 5.255))


def wrap_heading(deg):
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
    return wrap_heading(prev_deg + float(alpha) * delta)


def _body_to_ned(ax_body, ay_body, heading_deg):
    psi = math.radians(heading_deg)
    a_n = ax_body * math.cos(psi) - ay_body * math.sin(psi)
    a_e = ax_body * math.sin(psi) + ay_body * math.cos(psi)
    return a_n, a_e


def _flow_to_ned(vx_body, vy_body, heading_deg):
    psi = math.radians(heading_deg)
    v_n = vx_body * math.cos(psi) - vy_body * math.sin(psi)
    v_e = vx_body * math.sin(psi) + vy_body * math.cos(psi)
    return v_n, v_e


def _course_over_ground_deg(v_n, v_e):
    if math.hypot(v_n, v_e) <= 0.1:
        return None
    return math.degrees(math.atan2(v_e, v_n)) % 360.0


def _nmea_ck(text):
    checksum = 0
    for ch in text:
        checksum ^= ord(ch)
    return f"{checksum:02X}"


def _nmea_to_deg(value, direction):
    v = float(value)
    d = int(v / 100)
    m = v - d * 100
    deg = d + m / 60.0
    return -deg if direction in ("S", "W") else deg


def _parse_gga(sentence):
    try:
        if "*" in sentence:
            body, cs = sentence[1:].rsplit("*", 1)
            calc = 0
            for char in body:
                calc ^= ord(char)
            if f"{calc:02X}" != cs.strip().upper():
                return None
        parts = sentence.split(",")
        if len(parts) < 10 or not parts[2] or not parts[4]:
            return None
        fix = int(parts[6]) if parts[6] else 0
        lat = _nmea_to_deg(parts[2], parts[3])
        lon = _nmea_to_deg(parts[4], parts[5])
        alt = float(parts[9]) if parts[9] else 0.0
        return lat, lon, alt, fix
    except Exception:
        return None


def _ubx_cksum(msg_class, msg_id, payload):
    ck_a = 0
    ck_b = 0
    for b in [msg_class, msg_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF, *payload]:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def _ubx(cls, mid, payload):
    ck_a, ck_b = _ubx_cksum(cls, mid, payload)
    return struct.pack("<BBBBH", 0xB5, 0x62, cls, mid, len(payload)) + payload + bytes([ck_a, ck_b])


def _ubx_nav_pvt(lat, lon, alt, v_n, v_e, nsats, now):
    vel_n_mm = int(v_n * 1000)
    vel_e_mm = int(v_e * 1000)
    speed_mm = int(math.hypot(v_n, v_e) * 1000)
    if math.hypot(v_n, v_e) > 0.1:
        head_mot_1e5 = int((math.degrees(math.atan2(v_e, v_n)) % 360.0) * 1e5)
    else:
        head_mot_1e5 = 0

    tow = (now.hour * 3600 + now.minute * 60 + now.second) * 1000 + now.microsecond // 1000
    payload = struct.pack(
        "<IHBBBBBBIiBBBBiiiiIIiiiiiIIHBBBBBBi",
        tow,
        now.year,
        now.month,
        now.day,
        now.hour,
        now.minute,
        now.second,
        0x07,
        0,
        0,
        3,
        0x01,
        0x00,
        nsats,
        int(lon * 1e7),
        int(lat * 1e7),
        int(alt * 1000),
        int(alt * 1000),
        1500,
        2000,
        vel_n_mm,
        vel_e_mm,
        0,
        speed_mm,
        head_mot_1e5,
        3000,
        36000000,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    return _ubx(0x01, 0x07, payload)


def _ubx_nav_posllh(lat, lon, alt, tow):
    payload = struct.pack(
        "<IiiiiII",
        tow,
        int(lon * 1e7),
        int(lat * 1e7),
        int(alt * 1000),
        int(alt * 1000),
        2000,
        3000,
    )
    return _ubx(0x01, 0x02, payload)


def _ubx_nav_velned(v_n, v_e, tow):
    speed = math.hypot(v_n, v_e)
    course_deg = _course_over_ground_deg(v_n, v_e) or 0.0
    heading = math.radians(course_deg)
    payload = struct.pack(
        "<IiiiIIiII",
        tow,
        int(speed * 100 * math.cos(heading)),
        int(speed * 100 * math.sin(heading)),
        0,
        int(speed * 100),
        int(speed * 100),
        int(course_deg * 1e5),
        50,
        5000,
    )
    return _ubx(0x01, 0x12, payload)


def _nmea_sentences(lat, lon, alt, speed_mps, course_deg, nsats, now):
    timestamp = now.strftime("%H%M%S") + f".{now.microsecond // 10000:02d}"
    date = now.strftime("%d%m%y")

    def _deg_to_nmea(deg, is_lat):
        dd = int(abs(deg))
        minutes = (abs(deg) - dd) * 60
        fmt = f"{dd:02d}{minutes:07.4f}" if is_lat else f"{dd:03d}{minutes:07.4f}"
        hemi = ("N" if deg >= 0 else "S") if is_lat else ("E" if deg >= 0 else "W")
        return fmt, hemi

    lat_text, lat_dir = _deg_to_nmea(lat, True)
    lon_text, lon_dir = _deg_to_nmea(lon, False)
    gga = f"GPGGA,{timestamp},{lat_text},{lat_dir},{lon_text},{lon_dir},1,{nsats:02d},0.8,{alt:.1f},M,-34.0,M,,"
    course_text = "" if course_deg is None else f"{course_deg:.1f}"
    rmc = f"GPRMC,{timestamp},A,{lat_text},{lat_dir},{lon_text},{lon_dir},{speed_mps * 1.94384:.2f},{course_text},{date},,,A"
    return (
        f"${gga}*{_nmea_ck(gga)}\r\n".encode(),
        f"${rmc}*{_nmea_ck(rmc)}\r\n".encode(),
    )


class SharedState:
    def __init__(self, default_lat, default_lon, default_alt):
        self._lock = threading.RLock()
        self.origin_set = False
        self.gps_healthy = False
        self.lat = default_lat
        self.lon = default_lon
        self.baro_alt = default_alt
        self.heading_deg = 0.0
        self.flow_vx = 0.0
        self.flow_vy = 0.0
        self.flow_quality = 0
        self.flow_ok = False
        self.imu_ax = 0.0
        self.imu_ay = 0.0
        self.optical_sample = None

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

    def update_flow(self, vx, vy, quality, ok, sample):
        with self._lock:
            self.flow_vx = vx
            self.flow_vy = vy
            self.flow_quality = quality
            self.flow_ok = ok
            self.optical_sample = sample

    def update_imu(self, ax, ay):
        with self._lock:
            self.imu_ax = ax
            self.imu_ay = ay

    def update_baro(self, alt_m):
        with self._lock:
            self.baro_alt = alt_m

    def update_heading(self, deg, smoothing_alpha, max_delta_deg):
        with self._lock:
            self.heading_deg = smooth_heading_deg(
                self.heading_deg,
                deg,
                smoothing_alpha,
                max_delta_deg,
            )

    def update_position(self, lat, lon):
        with self._lock:
            self.lat = lat
            self.lon = lon


class VelocityKF:
    def __init__(self, q, r):
        self.v = 0.0
        self.P = 1.0
        self.q = q
        self.r = r

    def predict(self, accel_ms2, dt):
        self.v += accel_ms2 * dt
        self.P += self.q * dt

    def update(self, v_measured):
        k = self.P / (self.P + self.r)
        self.v = self.v + k * (v_measured - self.v)
        self.P = (1.0 - k) * self.P


class MicolinkMessage:
    def __init__(self):
        self.head = 0
        self.dev_id = 0
        self.sys_id = 0
        self.msg_id = 0
        self.seq = 0
        self.length = 0
        self.payload = bytearray(MICOLINK_MAX_PAYLOAD_LEN)
        self.checksum = 0
        self.status = 0
        self.payload_cnt = 0

    def reset(self):
        self.status = 0
        self.payload_cnt = 0


class OpticalFlowSample:
    def __init__(self, payload_bytes):
        data = struct.unpack("<IIBBBBhhBBH", payload_bytes[:24])
        self.time_ms = data[0]
        self.distance_mm = data[1]
        self.strength = data[2]
        self.precision = data[3]
        self.dis_status = data[4]
        self.reserved1 = data[5]
        self.flow_vx = data[6]
        self.flow_vy = data[7]
        self.flow_quality = data[8]
        self.flow_status = data[9]
        self.reserved2 = data[10]
        self.dist_ok = 1 if self.distance_mm > 0 and self.dis_status == 1 else 0
        self.dist_cm = self.distance_mm / 10.0 if self.dist_ok else 0.0
        self.height_m = self.distance_mm / 1000.0 if self.dist_ok else 0.0
        self.speed_x = self.flow_vx * self.height_m
        self.speed_y = self.flow_vy * self.height_m
        self.flow_ok = 1 if self.flow_status == 1 else 0

    def format_line(self, index):
        return (
            f"[{index:05d}] time_ms={self.time_ms} dist_mm={self.distance_mm} "
            f"dist_cm={self.dist_cm:.1f} dist_ok={self.dist_ok} "
            f"flow_vx={self.flow_vx} flow_vy={self.flow_vy} "
            f"flow_q={self.flow_quality} flow_ok={self.flow_ok} "
            f"speed_x={self.speed_x:.2f} speed_y={self.speed_y:.2f}"
        )


class MicolinkParser:
    def __init__(self):
        self.msg = MicolinkMessage()

    def calculate_checksum(self, msg):
        checksum = msg.head + msg.dev_id + msg.sys_id + msg.msg_id + msg.seq + msg.length
        for i in range(msg.length):
            checksum += msg.payload[i]
        return checksum & 0xFF

    def parse_char(self, data):
        msg = self.msg
        if msg.status == 0:
            if data == MICOLINK_MSG_HEAD:
                msg.head = data
                msg.status = 1
        elif msg.status == 1:
            msg.dev_id = data
            msg.status = 2
        elif msg.status == 2:
            msg.sys_id = data
            msg.status = 3
        elif msg.status == 3:
            msg.msg_id = data
            msg.status = 4
        elif msg.status == 4:
            msg.seq = data
            msg.status = 5
        elif msg.status == 5:
            msg.length = data
            if msg.length == 0:
                msg.status = 7
            elif msg.length > MICOLINK_MAX_PAYLOAD_LEN:
                msg.reset()
            else:
                msg.status = 6
        elif msg.status == 6:
            msg.payload[msg.payload_cnt] = data
            msg.payload_cnt += 1
            if msg.payload_cnt == msg.length:
                msg.payload_cnt = 0
                msg.status = 7
        elif msg.status == 7:
            msg.checksum = data
            msg.status = 0
            if self.calculate_checksum(msg) == msg.checksum:
                return True
            msg.reset()
        else:
            msg.reset()
        return False

    def decode_message(self):
        if self.msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR:
            return OpticalFlowSample(bytes(self.msg.payload[: self.msg.length]))
        return None


class GPSOriginReader(threading.Thread):
    daemon = True

    def __init__(self, state, port, baud, timeout_s):
        super().__init__(name="gps-origin")
        self.state = state
        self.port = port
        self.baud = int(baud)
        self.timeout_s = float(timeout_s)

    def run(self):
        deadline = time.time() + self.timeout_s
        print(f"[GPS] Waiting {self.timeout_s}s for real GPS fix on {self.port}...")
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1.0)
        except serial.SerialException as exc:
            print(f"[GPS] Cannot open {self.port}: {exc} -> using default origin")
            self.state.use_default_origin()
            return

        buf = b""
        while True:
            try:
                buf += ser.read(256)
            except serial.SerialException:
                break
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                sentence = line.decode("ascii", errors="ignore").strip()
                if sentence.startswith(("$GPGGA", "$GNGGA")):
                    result = _parse_gga(sentence)
                    if result and result[3] >= 1:
                        self.state.set_origin(result[0], result[1], result[2])
                        return
            if not self.state.origin_set and time.time() > deadline:
                print("[GPS] Timeout -> using default origin")
                self.state.use_default_origin()
                return


class MTF01FlowReader(threading.Thread):
    daemon = True

    def __init__(self, state, port, baud, rate_hz, heartbeat_interval_s, print_enabled, max_flow_raw):
        super().__init__(name="mtf01-reader")
        self.state = state
        self.port = port
        self.baud = int(baud)
        self.rate_hz = float(rate_hz)
        self.heartbeat_interval_s = float(heartbeat_interval_s)
        self.print_enabled = bool(print_enabled)
        self.max_flow_raw = None if max_flow_raw is None else float(max_flow_raw)
        self._ser = None
        self._running = True
        self._sequence = 0
        self._last_error = None
        self._msg_count = 0

    def stop(self):
        self._running = False

    def get_last_error(self):
        return self._last_error

    def _checksum(self, data):
        return sum(data) & 0xFF

    def _heartbeat(self):
        time_ms = int(time.time() * 1000) & 0xFFFFFFFF
        msg = bytearray([0xEF, 0x01, 0x00, 0x01, self._sequence & 0xFF, 0x0D])
        msg.extend(struct.pack("<I", time_ms))
        msg.extend(bytes([0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]))
        msg.append(self._checksum(msg))
        self._sequence += 1
        return bytes(msg)

    def run(self):
        try:
            print(f"[Flow] Opening {self.port} @ {self.baud}...")
            self._ser = serial.Serial(
                self.port,
                self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            self._ser.setDTR(True)
            self._ser.setRTS(True)
            time.sleep(0.1)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

            for _ in range(3):
                self._ser.write(self._heartbeat())
                self._ser.flush()
                time.sleep(0.1)

            time.sleep(0.3)
            self._ser.reset_input_buffer()

            parser = MicolinkParser()
            emit_delay = (1.0 / self.rate_hz) if self.rate_hz > 0 else 0.0
            last_emit = 0.0
            last_hb = 0.0
            first_raw = True

            while self._running:
                now = time.time()
                if now - last_hb >= self.heartbeat_interval_s:
                    self._ser.write(self._heartbeat())
                    self._ser.flush()
                    last_hb = now

                if self._ser.in_waiting == 0:
                    time.sleep(0.005)
                    continue

                byte = self._ser.read(1)
                if not byte or not parser.parse_char(byte[0]):
                    continue

                sample = parser.decode_message()
                if sample is None:
                    continue

                if (
                    self.max_flow_raw is not None
                    and (
                        abs(sample.flow_vx) >= self.max_flow_raw
                        or abs(sample.flow_vy) >= self.max_flow_raw
                    )
                ):
                    sample.flow_quality = 0
                    sample.flow_status = 0
                    sample.flow_ok = 0
                    sample.speed_x = 0.0
                    sample.speed_y = 0.0

                dist_valid = sample.dist_ok == 1
                if dist_valid:
                    dist_m = sample.distance_mm / 1000.0
                    vx_ms = (sample.flow_vy * dist_m) / 100.0
                    vy_ms = -(sample.flow_vx * dist_m) / 100.0
                else:
                    vx_ms = 0.0
                    vy_ms = 0.0

                ok = (sample.flow_status == 1) and (sample.flow_quality >= FLOW_QUALITY_MIN) and dist_valid
                self.state.update_flow(vx_ms, vy_ms, sample.flow_quality, ok, sample)

                if first_raw:
                    print(
                        f"[Flow] First frame: dist_mm={sample.distance_mm} "
                        f"flow_vx={sample.flow_vx} flow_vy={sample.flow_vy} "
                        f"quality={sample.flow_quality} ok={ok}"
                    )
                    first_raw = False

                if emit_delay == 0.0 or (now - last_emit) >= emit_delay:
                    last_emit = now
                    self._msg_count += 1
                    if self.print_enabled:
                        print(sample.format_line(self._msg_count))
        except Exception as exc:
            self._last_error = exc
        finally:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.setDTR(False)
                    self._ser.setRTS(False)
                    self._ser.close()
                except Exception:
                    pass


class PixhawkBridge(threading.Thread):
    daemon = True

    def __init__(
        self,
        state,
        mav_port,
        mav_baud,
        source_system,
        source_component,
        optical_send_interval_s,
        range_min_m,
        range_max_m,
        mav_print,
        heading_alpha,
        heading_max_delta_deg,
    ):
        super().__init__(name="pixhawk-bridge")
        self.state = state
        self.mav_port = mav_port
        self.mav_baud = int(mav_baud)
        self.source_system = int(source_system)
        self.source_component = int(source_component)
        self.optical_send_interval_s = float(optical_send_interval_s)
        self.range_min_m = float(range_min_m)
        self.range_max_m = float(range_max_m)
        self.mav_print = bool(mav_print)
        self.heading_alpha = float(heading_alpha)
        self.heading_max_delta_deg = float(heading_max_delta_deg)
        self._running = True
        self._last_send = 0.0
        self._last_optical_time_ms = None
        self._last_heartbeat = 0.0
        self._last_error = None
        self._flow_sent = 0
        self._range_sent = 0
        self.conn = None

    def stop(self):
        self._running = False

    def get_last_error(self):
        return self._last_error

    def _request_message(self, msg_id, hz):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1_000_000 / hz),
            0,
            0,
            0,
            0,
            0,
        )

    def _send_companion_heartbeat(self):
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )

    def _send_optical_messages(self, sample):
        time_usec = int(time.time() * 1_000_000)
        height_m = sample.height_m if sample.dist_ok else -1.0

        dt_s = None
        if self._last_optical_time_ms is not None and sample.time_ms is not None:
            dt_ms = sample.time_ms - self._last_optical_time_ms
            if dt_ms > 0:
                dt_s = dt_ms / 1000.0
        if dt_s is None or dt_s <= 0.0:
            dt_s = self.optical_send_interval_s if self.optical_send_interval_s > 0 else 0.02
        self._last_optical_time_ms = sample.time_ms

        comp_x = (sample.flow_vx * sample.height_m / FOCAL_LENGTH_PX) if sample.height_m > 0 else 0.0
        comp_y = (sample.flow_vy * sample.height_m / FOCAL_LENGTH_PX) if sample.height_m > 0 else 0.0
        quality = int(max(0, min(255, sample.flow_quality)))

        # Keep the legacy OPTICAL_FLOW behavior from of_mav2.py
        self.conn.mav.optical_flow_send(
            time_usec,
            0,
            int(sample.flow_vx),
            int(sample.flow_vy),
            float(comp_x),
            float(comp_y),
            quality,
            float(sample.height_m),
        )
        self._flow_sent += 1

        if sample.dist_ok:
            covariance = int(max(0, min(50, 50 - (quality // 5)))) if quality > 0 else 255
            self.conn.mav.distance_sensor_send(
                int(time.time() * 1000) % (2**32),
                int(self.range_min_m * 100),
                int(self.range_max_m * 100),
                int((sample.distance_mm / 1000.0) * 100),
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                0,
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
                covariance,
            )
            self._range_sent += 1

        # Also send OPTICAL_FLOW_RAD-style integrated payload from the main-project mode
        integrated_x = float(sample.flow_vx) * dt_s
        integrated_y = float(sample.flow_vy) * dt_s
        self.conn.mav.optical_flow_rad_send(
            int(time_usec),
            0,
            max(1, int(dt_s * 1_000_000)),
            integrated_x,
            integrated_y,
            0.0,
            0.0,
            0.0,
            0,
            quality,
            max(1, int(dt_s * 1_000_000)),
            float(height_m),
        )

        if self.mav_print and self._flow_sent % 50 == 0:
            print(
                f"[MAV] flow_sent={self._flow_sent} range_sent={self._range_sent} "
                f"raw=({sample.flow_vx},{sample.flow_vy}) q={quality} h={sample.height_m:.2f}m"
            )

    def run(self):
        try:
            print(f"[MAV] Connecting {self.mav_port} @ {self.mav_baud}...")
            is_serial = not self.mav_port.startswith(("udp", "tcp"))
            self.conn = mavutil.mavlink_connection(
                self.mav_port,
                baud=self.mav_baud if is_serial else None,
                source_system=self.source_system,
                source_component=self.source_component,
            )

            print("[MAV] Waiting for heartbeat...")
            try:
                self.conn.wait_heartbeat(timeout=15)
                print(
                    f"[MAV] Heartbeat received "
                    f"(sysid={self.conn.target_system} compid={self.conn.target_component})"
                )
            except Exception:
                print("[MAV] No heartbeat received, continuing anyway")

            self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, 50)
            self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, 20)
            self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2, 20)
            self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3, 20)
            self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)

            baro_types = {"SCALED_PRESSURE", "SCALED_PRESSURE2", "HIGHRES_IMU"}
            imu_types = {"SCALED_IMU"}
            heading_types = {"ATTITUDE"}
            all_types = list(baro_types | imu_types | heading_types)
            last_report = 0.0

            while self._running:
                now = time.time()
                if now - self._last_heartbeat >= 1.0:
                    self._send_companion_heartbeat()
                    self._last_heartbeat = now

                with self.state._lock:
                    sample = self.state.optical_sample
                if sample is not None and now - self._last_send >= self.optical_send_interval_s:
                    self._send_optical_messages(sample)
                    self._last_send = now

                msg = self.conn.recv_match(type=all_types, blocking=False)
                if msg is None:
                    time.sleep(0.01)
                    continue

                msg_type = msg.get_type()
                if msg_type in baro_types:
                    if msg_type == "HIGHRES_IMU":
                        alt = _pressure_to_alt(msg.abs_pressure, msg.temperature)
                    else:
                        alt = _pressure_to_alt(msg.press_abs, msg.temperature / 100.0)
                    if alt is not None:
                        self.state.update_baro(alt)

                if msg_type in imu_types:
                    self.state.update_imu(msg.xacc * MG_TO_MS2, msg.yacc * MG_TO_MS2)

                if msg_type in heading_types and getattr(msg, "yaw", None) is not None:
                    heading = wrap_heading(float(msg.yaw) * 180.0 / math.pi)
                    self.state.update_heading(
                        heading,
                        self.heading_alpha,
                        self.heading_max_delta_deg,
                    )

                if self.mav_print and now - last_report >= 5.0:
                    last_report = now
                    with self.state._lock:
                        print(
                            f"[MAV] heading={self.state.heading_deg:.1f} "
                            f"baro={self.state.baro_alt:.1f}m "
                            f"imu=({self.state.imu_ax:.3f},{self.state.imu_ay:.3f})"
                        )
        except Exception as exc:
            self._last_error = exc
        finally:
            if self.conn is not None:
                try:
                    self.conn.close()
                except Exception:
                    pass


def run_gps_output_loop(state, kf_n, kf_e, out_port, out_baud, update_hz, output_protocol, print_nmea, print_ubx):
    print(f"[Out] Opening output port {out_port} @ {out_baud}...")
    try:
        out = serial.Serial(out_port, out_baud, timeout=1)
    except serial.SerialException as exc:
        print(f"[Out] Cannot open: {exc}")
        return

    dt_target = 1.0 / update_hz
    last_t = time.time()
    loop_n = 0

    print("[Main] Waiting for origin to be set...")
    while not state.origin_set:
        time.sleep(0.05)
    print("[Main] Origin ready - starting integration loop")

    while True:
        now_t = time.time()
        dt = max(0.001, min(now_t - last_t, 0.5))
        last_t = now_t

        with state._lock:
            ax_b = state.imu_ax
            ay_b = state.imu_ay
            heading = state.heading_deg
            vx_b = state.flow_vx
            vy_b = state.flow_vy
            flow_ok = state.flow_ok
            quality = state.flow_quality
            lat = state.lat
            lon = state.lon
            alt = state.baro_alt

        if flow_ok and quality >= FLOW_QUALITY_MIN:
            a_n, a_e = _body_to_ned(ax_b, ay_b, heading)
            kf_n.predict(a_n, dt)
            kf_e.predict(a_e, dt)
            v_n_flow, v_e_flow = _flow_to_ned(vx_b, vy_b, heading)
            kf_n.update(v_n_flow)
            kf_e.update(v_e_flow)
        else:
            decay = math.exp(-dt / 0.3)
            kf_n.v *= decay
            kf_e.v *= decay
            kf_n.P = 1.0
            kf_e.P = 1.0

        v_n = max(-7.0, min(7.0, kf_n.v))
        v_e = max(-7.0, min(7.0, kf_e.v))

        cos_lat = math.cos(math.radians(lat))
        if abs(cos_lat) < 1e-6:
            cos_lat = 1e-6
        dlat = (v_n * dt) / 111_111.0
        dlon = (v_e * dt) / (111_111.0 * cos_lat)
        state.update_position(lat + dlat, lon + dlon)

        with state._lock:
            lat = state.lat
            lon = state.lon

        speed_mps = math.hypot(v_n, v_e)
        course_deg = _course_over_ground_deg(v_n, v_e)
        utc = datetime.now(timezone.utc)
        tow = (utc.hour * 3600 + utc.minute * 60 + utc.second) * 1000

        if output_protocol == "ubx":
            out.write(_ubx_nav_pvt(lat, lon, alt, v_n, v_e, 10, utc))
            out.write(_ubx_nav_posllh(lat, lon, alt, tow))
            out.write(_ubx_nav_velned(v_n, v_e, tow))
            if print_ubx and loop_n % max(1, int(update_hz)) == 0:
                print(
                    f"[UBX {utc.strftime('%H:%M:%S')}] "
                    f"lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m "
                    f"velN={v_n:+.3f} velE={v_e:+.3f}"
                )
        else:
            gga, rmc = _nmea_sentences(lat, lon, alt, speed_mps, course_deg, 10, utc)
            out.write(gga)
            out.write(rmc)
            if print_nmea and loop_n % max(1, int(update_hz)) == 0:
                course_text = "N/A" if course_deg is None else f"{course_deg:.1f}"
                print(
                    f"[NMEA {utc.strftime('%H:%M:%S')}] "
                    f"lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m "
                    f"speed={speed_mps:.2f} course={course_text}"
                )

        loop_n += 1
        if loop_n % max(1, int(update_hz * 2)) == 0:
            origin_src = "real GPS" if state.gps_healthy else "default"
            flow_text = f"OK q={quality}" if flow_ok else f"BAD q={quality}"
            print(
                f"[{utc.strftime('%H:%M:%S')}] lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m "
                f"vN={v_n:+.3f} vE={v_e:+.3f} spd={speed_mps:.2f}m/s "
                f"flow={flow_text} KF_P={kf_n.P:.4f} origin={origin_src}"
            )

        used = time.time() - now_t
        time.sleep(max(0.0, dt_target - used))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Combined flow3 + of_mav2 standalone bridge without collisions."
    )
    parser.add_argument("--flow-port", default="/dev/ttyAMA3")
    parser.add_argument("--flow-baud", type=int, default=115200)
    parser.add_argument("--flow-rate-hz", type=float, default=60.0)
    parser.add_argument("--flow-heartbeat-s", type=float, default=0.6)
    parser.add_argument("--flow-print", type=_bool_arg, default=True)
    parser.add_argument("--flow-max-raw", type=float, default=120.0)
    parser.add_argument("--gps-port", default="/dev/ttyAMA5")
    parser.add_argument("--gps-baud", type=int, default=230400)
    parser.add_argument("--gps-timeout-s", type=float, default=10.0)
    parser.add_argument("--no-real-gps", action="store_true")
    parser.add_argument("--mav-port", default="/dev/ttyACM0")
    parser.add_argument("--mav-baud", type=int, default=115200)
    parser.add_argument("--source-system", type=int, default=1)
    parser.add_argument("--source-component", type=int, default=192)
    parser.add_argument("--optical-send-interval-s", type=float, default=0.02)
    parser.add_argument("--range-min-m", type=float, default=0.01)
    parser.add_argument("--range-max-m", type=float, default=8.0)
    parser.add_argument("--mav-print", type=_bool_arg, default=True)
    parser.add_argument("--out-port", default="/dev/ttyAMA0")
    parser.add_argument("--out-baud", type=int, default=230400)
    parser.add_argument("--update-hz", type=float, default=5.0)
    parser.add_argument("--output-protocol", choices=["ubx", "nmea"], default="nmea")
    parser.add_argument("--print-nmea-debug", type=_bool_arg, default=False)
    parser.add_argument("--print-ubx-debug", type=_bool_arg, default=True)
    parser.add_argument("--default-lat", type=float, default=12.971600)
    parser.add_argument("--default-lon", type=float, default=77.594600)
    parser.add_argument("--default-alt", type=float, default=920.0)
    parser.add_argument("--kf-q", type=float, default=0.05)
    parser.add_argument("--kf-r", type=float, default=0.02)
    parser.add_argument("--heading-alpha", type=float, default=0.18)
    parser.add_argument("--heading-max-delta-deg", type=float, default=10.0)
    return parser.parse_args()


def main():
    args = parse_args()

    state = SharedState(args.default_lat, args.default_lon, args.default_alt)
    kf_n = VelocityKF(args.kf_q, args.kf_r)
    kf_e = VelocityKF(args.kf_q, args.kf_r)

    print("=" * 72)
    print("  Combined flow3 + of_mav2")
    print(
        f"  FLOW={args.flow_port}@{args.flow_baud}  MAV={args.mav_port}@{args.mav_baud}  "
        f"OUT={args.out_port}@{args.out_baud}  PROTO={args.output_protocol}"
    )
    print("=" * 72)

    if args.no_real_gps:
        state.use_default_origin()
    else:
        GPSOriginReader(state, args.gps_port, args.gps_baud, args.gps_timeout_s).start()

    flow_reader = MTF01FlowReader(
        state,
        args.flow_port,
        args.flow_baud,
        args.flow_rate_hz,
        args.flow_heartbeat_s,
        args.flow_print,
        args.flow_max_raw,
    )
    flow_reader.start()

    pixhawk_bridge = PixhawkBridge(
        state=state,
        mav_port=args.mav_port,
        mav_baud=args.mav_baud,
        source_system=args.source_system,
        source_component=args.source_component,
        optical_send_interval_s=args.optical_send_interval_s,
        range_min_m=args.range_min_m,
        range_max_m=args.range_max_m,
        mav_print=args.mav_print,
        heading_alpha=args.heading_alpha,
        heading_max_delta_deg=args.heading_max_delta_deg,
    )
    pixhawk_bridge.start()

    time.sleep(2.5)

    try:
        run_gps_output_loop(
            state=state,
            kf_n=kf_n,
            kf_e=kf_e,
            out_port=args.out_port,
            out_baud=args.out_baud,
            update_hz=args.update_hz,
            output_protocol=args.output_protocol,
            print_nmea=args.print_nmea_debug,
            print_ubx=args.print_ubx_debug,
        )
    finally:
        flow_reader.stop()
        pixhawk_bridge.stop()


if __name__ == "__main__":
    main()
