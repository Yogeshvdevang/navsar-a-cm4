#!/usr/bin/env python3
"""
flow_gps.py — Optical Flow → Fake GPS Emitter for Pixhawk
==========================================================
Architecture:
  Thread 1  GPSOriginReader  — reads real GPS (NMEA) for origin, then stays idle
  Thread 2  FlowReader       — reads MTF-01 optical flow sensor (Micolink protocol)
  Thread 3  MAVLinkBridge    — reads barometer altitude + compass heading from Pixhawk
  Main      Integrator       — integrates vx/vy with heading into lat/lon at 10 Hz
             GPSEmitter      — sends UBX-NAV-PVT + NMEA GGA/RMC to Pixhawk GPS port

Wiring (adjust ports to match your RP5):
  MTF-01 optical flow  →  /dev/ttyAMA3  @ 115200
  Real GPS (NMEA)      →  /dev/ttyAMA5  @ 230400   (used only for origin)
  Pixhawk MAVLink      →  /dev/ttyACM0  @ 115200   (baro + compass)
  Pixhawk GPS port     →  /dev/ttyAMA0  @ 230400   (output of this script)
"""

import math
import struct
import threading
import time
import serial
import argparse
from datetime import datetime, timezone
from pymavlink import mavutil

# ============================================================
# CONFIG  — edit these to match your setup
# ============================================================
FLOW_PORT    = "/dev/ttyAMA3"   # MTF-01 optical flow
FLOW_BAUD    = 115200

GPS_PORT     = "/dev/ttyAMA5"   # Real GPS sensor (NMEA, for origin only)
GPS_BAUD     = 230400

MAV_PORT     = "/dev/ttyACM0"   # Pixhawk MAVLink (baro + compass)
MAV_BAUD     = 115200

OUT_PORT     = "/dev/ttyAMA0"   # → Pixhawk GPS input port
OUT_BAUD     = 230400

UPDATE_HZ    = 10               # Rate of GPS packets sent to Pixhawk
GPS_TIMEOUT  = 10.0             # Seconds to wait for real GPS before using default

# Default origin — used if real GPS never gets a healthy fix
DEFAULT_LAT  = 12.971600
DEFAULT_LON  = 77.594600
DEFAULT_ALT  = 920.0            # meters MSL

# Flow quality threshold — below this, position is not updated
FLOW_QUALITY_MIN = 50

# MTF-01 velocity filters
FLOW_DEAD_ZONE  = 0.01   # m/s  — values below this are treated as zero (noise floor)
FLOW_MAX_SPEED  = 5.0    # m/s  — hard clamp; anything above is a glitch frame
FLOW_EMA_ALPHA  = 0.3    # EMA smoothing: 0=no update, 1=no smoothing (0.2–0.4 is good)

# Legacy scale multiplier (keep at 1.0 — real scale is now handled in to_state)
FLOW_SCALE = 1.0

NUM_SATS_FAKE = 10  # reported satellite count (keeps Pixhawk happy)
# ============================================================


# ============================================================
# SHARED STATE  (all fields protected by a single RLock)
# ============================================================
class SharedState:
    def __init__(self):
        self._lock = threading.RLock()

        # Origin (set once from real GPS or default)
        self.origin_set   = False
        self.lat          = DEFAULT_LAT
        self.lon          = DEFAULT_LON
        self.alt          = DEFAULT_ALT   # comes from baro after origin is set

        # Live sensors
        self.baro_alt     = DEFAULT_ALT   # meters MSL from Pixhawk baro
        self.heading_deg  = 0.0           # degrees from Pixhawk compass
        self.flow_vx      = 0.0           # m/s body-forward (already scaled by height)
        self.flow_vy      = 0.0           # m/s body-right
        self.flow_quality = 0             # 0-255
        self.flow_ok      = False         # True if sensor reports valid flow

        self.gps_healthy  = False         # True if real GPS gave a valid fix

    # Convenience wrappers
    def get(self):
        with self._lock:
            return (
                self.lat, self.lon, self.baro_alt,
                self.heading_deg,
                self.flow_vx, self.flow_vy,
                self.flow_quality, self.flow_ok,
            )

    def set_origin(self, lat, lon, alt):
        with self._lock:
            if not self.origin_set:
                self.lat = lat
                self.lon = lon
                self.alt = alt
                self.origin_set = True
                self.gps_healthy = True
                print(f"[Origin] Set from real GPS: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m")

    def update_flow(self, vx, vy, quality, ok):
        with self._lock:
            # EMA low-pass filter — damps single-frame spikes that survive the clamp.
            # New = α × raw + (1−α) × previous
            self.flow_vx = FLOW_EMA_ALPHA * (vx * FLOW_SCALE) + (1.0 - FLOW_EMA_ALPHA) * self.flow_vx
            self.flow_vy = FLOW_EMA_ALPHA * (vy * FLOW_SCALE) + (1.0 - FLOW_EMA_ALPHA) * self.flow_vy
            # Re-apply dead-zone after smoothing so residual filter memory doesn't creep
            if abs(self.flow_vx) < FLOW_DEAD_ZONE:
                self.flow_vx = 0.0
            if abs(self.flow_vy) < FLOW_DEAD_ZONE:
                self.flow_vy = 0.0
            self.flow_quality = quality
            self.flow_ok = ok

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


# ============================================================
# THREAD 1 — GPS ORIGIN READER  (NMEA parser)
# ============================================================
class GPSOriginReader(threading.Thread):
    """
    Reads NMEA sentences from the real GPS sensor.
    Parses GGA for lat/lon/alt. Once a healthy fix is obtained
    it sets the shared origin and then keeps the thread alive
    to update fallback data if needed.
    """
    name = "GPSOriginReader"
    daemon = True

    def run(self):
        deadline = time.time() + GPS_TIMEOUT
        print(f"[GPS] Waiting up to {GPS_TIMEOUT}s for real GPS fix on {GPS_PORT}…")
        try:
            ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1.0)
        except serial.SerialException as e:
            print(f"[GPS] Cannot open {GPS_PORT}: {e}  → will use default origin")
            _use_default_origin()
            return

        buf = b""
        while True:
            try:
                chunk = ser.read(256)
            except serial.SerialException:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                sentence = line.decode("ascii", errors="ignore").strip()
                if sentence.startswith("$GPGGA") or sentence.startswith("$GNGGA"):
                    result = _parse_gga(sentence)
                    if result:
                        lat, lon, alt, fix = result
                        if fix >= 1:
                            state.set_origin(lat, lon, alt)
                            # Keep thread running for potential re-acquisition
                            # but origin is already locked in

            if not state.origin_set and time.time() > deadline:
                print("[GPS] Timeout — using default origin")
                _use_default_origin()
                return


def _use_default_origin():
    with state._lock:
        if not state.origin_set:
            state.origin_set = True
            print(f"[Origin] Default: lat={DEFAULT_LAT} lon={DEFAULT_LON} alt={DEFAULT_ALT}m")


def _parse_gga(sentence):
    """Parse $GPGGA / $GNGGA. Returns (lat, lon, alt_m, fix_quality) or None."""
    try:
        # Verify checksum
        if "*" in sentence:
            body, cs = sentence[1:].rsplit("*", 1)
            calc = 0
            for c in body:
                calc ^= ord(c)
            if f"{calc:02X}" != cs.upper():
                return None
        parts = sentence.split(",")
        if len(parts) < 10:
            return None
        fix = int(parts[6]) if parts[6] else 0
        if not parts[2] or not parts[4]:
            return None
        lat = _nmea_to_deg(parts[2], parts[3])
        lon = _nmea_to_deg(parts[4], parts[5])
        alt = float(parts[9]) if parts[9] else 0.0
        return lat, lon, alt, fix
    except Exception:
        return None


def _nmea_to_deg(value, direction):
    v = float(value)
    d = int(v / 100)
    m = v - d * 100
    deg = d + m / 60.0
    if direction in ("S", "W"):
        deg = -deg
    return deg


# ============================================================
# THREAD 2 — MTF-01 OPTICAL FLOW READER  (Micolink protocol)
# Exact protocol classes from working oftical_flow.py — do not simplify.
# Only addition: on_data callback updates SharedState.
# ============================================================
MICOLINK_MSG_HEAD        = 0xEF
MICOLINK_MSG_ID_RANGE_SENSOR = 0x51
MICOLINK_MAX_PAYLOAD_LEN = 64


class MicolinkMessage:
    def __init__(self):
        self.head = 0
        self.dev_id = 0
        self.sys_id = 0
        self.msg_id = 0
        self.seq = 0
        self.len = 0
        self.payload = bytearray(MICOLINK_MAX_PAYLOAD_LEN)
        self.checksum = 0
        self.status = 0
        self.payload_cnt = 0

    def reset(self):
        self.status = 0
        self.payload_cnt = 0


class MTF01RangeSensor:
    def __init__(self, payload_bytes):
        data = struct.unpack('<IIBBBBhhBBH', payload_bytes[:24])
        self.time_ms      = data[0]
        self.distance     = data[1]
        self.strength     = data[2]
        self.precision    = data[3]
        self.dis_status   = data[4]
        self.reserved1    = data[5]
        self.flow_vel_x   = data[6]
        self.flow_vel_y   = data[7]
        self.flow_quality = data[8]
        self.flow_status  = data[9]
        self.reserved2    = data[10]

    def to_state(self):
        """Convert sensor reading to (vx, vy, quality, ok) for SharedState.

        Unit chain:
          flow_vel_x/y  →  int16, units = 0.001 rad/s  (milliradians/s)
          ÷ 1000        →  rad/s  (angular rate)
          × height_m    →  m/s   (linear velocity in body frame)

        Without the ÷1000 a raw value of 20 becomes 20 × 1.79 = 35.8 m/s
        instead of the correct 0.02 × 1.79 = 0.036 m/s.
        """
        dist_valid = self.distance > 0 and self.dis_status == 1
        height_m   = self.distance / 1000.0 if dist_valid else 0.3  # 30 cm fallback

        # Correct scale: mrad/s → rad/s → m/s
        vx = (self.flow_vel_x / 1000.0) * height_m
        vy = (self.flow_vel_y / 1000.0) * height_m

        # Dead-zone: sensor quantizes to ~0.001 rad/s steps; below 0.01 m/s is
        # indistinguishable from noise at hover — treat as zero.
        if abs(vx) < FLOW_DEAD_ZONE:
            vx = 0.0
        if abs(vy) < FLOW_DEAD_ZONE:
            vy = 0.0

        # Hard clamp: anything above the max credible drone speed is a glitch frame.
        vx = max(-FLOW_MAX_SPEED, min(FLOW_MAX_SPEED, vx))
        vy = max(-FLOW_MAX_SPEED, min(FLOW_MAX_SPEED, vy))

        ok = (self.flow_status == 1) and (self.flow_quality >= FLOW_QUALITY_MIN) and dist_valid
        return vx, vy, self.flow_quality, ok


class MicolinkParser:
    def __init__(self):
        self.msg = MicolinkMessage()

    def calculate_checksum(self, msg):
        checksum = msg.head + msg.dev_id + msg.sys_id + msg.msg_id + msg.seq + msg.len
        for i in range(msg.len):
            checksum += msg.payload[i]
        return checksum & 0xFF

    def parse_char(self, data):
        msg = self.msg
        if msg.status == 0:
            if data == MICOLINK_MSG_HEAD:
                msg.head = data
                msg.status += 1
        elif msg.status == 1:
            msg.dev_id = data
            msg.status += 1
        elif msg.status == 2:
            msg.sys_id = data
            msg.status += 1
        elif msg.status == 3:
            msg.msg_id = data
            msg.status += 1
        elif msg.status == 4:
            msg.seq = data
            msg.status += 1
        elif msg.status == 5:
            msg.len = data
            if msg.len == 0:
                msg.status += 2
            elif msg.len > MICOLINK_MAX_PAYLOAD_LEN:
                msg.reset()
            else:
                msg.status += 1
        elif msg.status == 6:
            msg.payload[msg.payload_cnt] = data
            msg.payload_cnt += 1
            if msg.payload_cnt == msg.len:
                msg.payload_cnt = 0
                msg.status += 1
        elif msg.status == 7:
            msg.checksum = data
            msg.status = 0
            if self.calculate_checksum(msg) == msg.checksum:
                return True
            else:
                msg.reset()
        else:
            msg.reset()
        return False

    def decode_message(self):
        if self.msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR:
            payload_bytes = bytes(self.msg.payload[:self.msg.len])
            return MTF01RangeSensor(payload_bytes)
        return None


class FlowReader(threading.Thread):
    """
    Reads MTF-01 via proven Micolink parser.
    Heartbeat runs in its own sub-thread (same pattern as working oftical_flow.py).
    Calls state.update_flow() on every valid frame.
    """
    name = "FlowReader"
    daemon = True

    def __init__(self):
        super().__init__()
        self._seq = 0
        self._ser = None
        self._running = False

    def _create_heartbeat(self):
        time_ms = int(time.time() * 1000) & 0xFFFFFFFF
        msg = bytearray([0xEF, 0x01, 0x00, 0x01, self._seq & 0xFF, 0x0D])
        msg += struct.pack('<I', time_ms)
        msg += bytes([0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        msg.append(sum(msg) & 0xFF)
        self._seq += 1
        return bytes(msg)

    def _heartbeat_loop(self):
        """Dedicated heartbeat thread — sends every 600 ms (same as working script)."""
        while self._running:
            try:
                self._ser.write(self._create_heartbeat())
                self._ser.flush()
            except Exception as e:
                if self._running:
                    print(f"[Flow] Heartbeat error: {e}")
                break
            time.sleep(0.6)

    def run(self):
        print(f"[Flow] Opening {FLOW_PORT} @ {FLOW_BAUD}…")
        try:
            self._ser = serial.Serial(
                FLOW_PORT, FLOW_BAUD,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=1,
                xonxoff=False, rtscts=False, dsrdtr=False,
            )
        except serial.SerialException as e:
            print(f"[Flow] Cannot open {FLOW_PORT}: {e}")
            return

        self._ser.setDTR(True)
        self._ser.setRTS(True)
        time.sleep(0.1)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        # Send 3 initial heartbeats then wait for sensor to wake
        for _ in range(3):
            self._ser.write(self._create_heartbeat())
            self._ser.flush()
            time.sleep(0.1)
        time.sleep(0.3)
        self._ser.reset_input_buffer()

        # Start dedicated heartbeat thread
        self._running = True
        hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True,
                                     name="FlowHeartbeat")
        hb_thread.start()

        parser = MicolinkParser()
        msg_count = 0
        start_time = time.time()
        data_received = False

        print("[Flow] Reading…")
        try:
            while True:
                if self._ser.in_waiting > 0:
                    raw = self._ser.read(1)
                    if not raw:
                        continue
                    if not data_received:
                        data_received = True
                        print(f"[Flow] First byte after {time.time()-start_time:.2f}s")

                    if parser.parse_char(raw[0]):
                        sensor = parser.decode_message()
                        if sensor is not None:
                            msg_count += 1
                            vx, vy, quality, ok = sensor.to_state()
                            state.update_flow(vx, vy, quality, ok)
                            if msg_count % 100 == 0:
                                print(f"[Flow] #{msg_count} vx={vx:.3f} vy={vy:.3f} "
                                      f"q={quality} ok={ok} "
                                      f"dist={sensor.distance}mm")
                else:
                    if not data_received and (time.time() - start_time) > 5:
                        print("[Flow] No data after 5s — toggling DTR/RTS…")
                        self._ser.setDTR(False); self._ser.setRTS(False)
                        time.sleep(0.1)
                        self._ser.reset_input_buffer()
                        start_time = time.time()
                    time.sleep(0.001)
        except Exception as e:
            print(f"[Flow] Reader crashed: {e}")
        finally:
            self._running = False
            if self._ser and self._ser.is_open:
                self._ser.setDTR(False); self._ser.setRTS(False)
                self._ser.close()
                print("[Flow] Port closed")


# ============================================================
# THREAD 3 — MAVLINK BRIDGE  (baro + compass)
# ============================================================
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
    """Reads barometer and compass from Pixhawk MAVLink, updates shared state."""
    name = "MAVLinkBridge"
    daemon = True

    def run(self):
        print(f"[MAV] Connecting to {MAV_PORT} @ {MAV_BAUD}…")
        try:
            conn = mavutil.mavlink_connection(MAV_PORT, baud=MAV_BAUD)
        except Exception as e:
            print(f"[MAV] Cannot connect: {e}")
            return

        print("[MAV] Waiting for heartbeat…")
        try:
            conn.wait_heartbeat(timeout=15)
            print("[MAV] Heartbeat received.")
        except Exception:
            print("[MAV] No heartbeat — continuing anyway")

        # Request streams
        for msg_id in [
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
        ]:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, 50_000, 0, 0, 0, 0, 0,   # 20 Hz
            )

        baro_types = {"SCALED_PRESSURE", "SCALED_PRESSURE2", "HIGHRES_IMU"}
        mag_types  = {"RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"}
        all_types  = list(baro_types | mag_types)

        print("[MAV] Reading baro + compass…")
        while True:
            msg = conn.recv_match(type=all_types, blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()

            if t in baro_types:
                if t == "HIGHRES_IMU":
                    p, tc = msg.abs_pressure, msg.temperature
                else:
                    p, tc = msg.press_abs, msg.temperature / 100.0
                alt = _pressure_to_alt(p, tc)
                if alt is not None:
                    state.update_baro(alt)

            elif t in mag_types:
                xm = float(getattr(msg, "xmag", 0.0))
                ym = float(getattr(msg, "ymag", 0.0))
                if xm != 0.0 or ym != 0.0:
                    h = _heading_from_mag(xm, ym)
                    if h is not None:
                        state.update_heading(h)


# ============================================================
# UBX / NMEA PACKET BUILDERS  (from gps_auto.py — unchanged)
# ============================================================
def _ubx_checksum(msg_class, msg_id, payload):
    ck_a = ck_b = 0
    for b in [msg_class, msg_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF] + list(payload):
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def _ubx(msg_class, msg_id, payload):
    length = len(payload)
    ck_a, ck_b = _ubx_checksum(msg_class, msg_id, payload)
    return struct.pack("<BBBBH", 0xB5, 0x62, msg_class, msg_id, length) + payload + bytes([ck_a, ck_b])


def _ubx_nav_pvt(lat, lon, alt, speed_mps, heading_deg, num_sats, now):
    hr = math.radians(heading_deg)
    vn = int(speed_mps * 1000 * math.cos(hr))
    ve = int(speed_mps * 1000 * math.sin(hr))
    tow = (now.hour * 3600 + now.minute * 60 + now.second) * 1000 + now.microsecond // 1000
    payload = struct.pack(
        "<IHBBBBBBIiBBBBiiiiIIiiiiiIIHBBBBBBi",
        tow, now.year, now.month, now.day,
        now.hour, now.minute, now.second,
        0x01,       # valid flags
        0,          # tAcc
        0,          # nano
        3,          # 3D fix
        0x01, 0x00, # flags, flags2
        num_sats,
        int(lon * 1e7), int(lat * 1e7),
        int(alt * 1000), int(alt * 1000),  # alt ellipsoid, alt MSL
        500, 1000,   # hAcc, vAcc (mm)
        vn, ve, 0,   # velN, velE, velD (mm/s)
        int(speed_mps * 1000),
        int(heading_deg * 1e5),
        1000, 10000, # sAcc, headAcc
        0,           # pDOP
        0, 0, 0, 0, 0, 0,  # reserved / flags3
        0,           # headVeh
    )
    return _ubx(0x01, 0x07, payload)


def _ubx_nav_posllh(lat, lon, alt, tow):
    payload = struct.pack(
        "<IiiiiII",
        tow,
        int(lon * 1e7), int(lat * 1e7),
        int(alt * 1000), int(alt * 1000),
        2000, 3000,  # hAcc, vAcc (mm)
    )
    return _ubx(0x01, 0x02, payload)


def _ubx_nav_velned(speed_mps, heading_deg, tow):
    hr = math.radians(heading_deg)
    vn = int(speed_mps * 100 * math.cos(hr))
    ve = int(speed_mps * 100 * math.sin(hr))
    payload = struct.pack(
        "<IiiiIIiII",
        tow, vn, ve, 0,
        int(speed_mps * 100), int(speed_mps * 100),
        int(heading_deg * 1e5),
        50, 5000,
    )
    return _ubx(0x01, 0x12, payload)


def _nmea_checksum(s):
    cs = 0
    for c in s:
        cs ^= ord(c)
    return f"{cs:02X}"


def _nmea_sentences(lat, lon, alt, speed_mps, heading_deg, num_sats, now):
    t_str = now.strftime("%H%M%S") + f".{now.microsecond // 10000:02d}"
    d_str = now.strftime("%d%m%y")

    def deg2nmea(deg, is_lat):
        d = int(abs(deg))
        m = (abs(deg) - d) * 60
        if is_lat:
            return f"{d:02d}{m:07.4f}", "N" if deg >= 0 else "S"
        return f"{d:03d}{m:07.4f}", "E" if deg >= 0 else "W"

    la, lad = deg2nmea(lat, True)
    lo, lod = deg2nmea(lon, False)

    gga_body = (f"GPGGA,{t_str},{la},{lad},{lo},{lod},1,{num_sats:02d},"
                f"0.8,{alt:.1f},M,-34.0,M,,")
    gga = f"${gga_body}*{_nmea_checksum(gga_body)}\r\n"

    knots = speed_mps * 1.94384
    rmc_body = (f"GPRMC,{t_str},A,{la},{lad},{lo},{lod},"
                f"{knots:.2f},{heading_deg:.1f},{d_str},,,A")
    rmc = f"${rmc_body}*{_nmea_checksum(rmc_body)}\r\n"

    return gga.encode(), rmc.encode()


# ============================================================
# INTEGRATOR  +  GPS EMITTER  (main loop)
# ============================================================
def _rotate_body_to_ned(vx_body, vy_body, heading_deg):
    """
    Rotate body-frame velocity to NED frame using compass heading.
    Body frame: X = forward, Y = right
    NED:        N = north,  E = east
    """
    h = math.radians(heading_deg)
    vN = vx_body * math.cos(h) - vy_body * math.sin(h)
    vE = vx_body * math.sin(h) + vy_body * math.cos(h)
    return vN, vE


def run_integrator_and_emitter():
    print(f"[Out] Opening GPS output port {OUT_PORT} @ {OUT_BAUD}…")
    try:
        out = serial.Serial(OUT_PORT, OUT_BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"[Out] Cannot open {OUT_PORT}: {e}")
        return

    dt   = 1.0 / UPDATE_HZ
    last = time.time()
    loop_count = 0

    print("[Main] Waiting for origin to be set…")
    while not state.origin_set:
        time.sleep(0.1)
    print("[Main] Origin ready — starting integration loop")

    while True:
        now_wall = time.time()
        elapsed  = now_wall - last
        last     = now_wall

        # --- read current sensor state ---
        with state._lock:
            lat       = state.lat
            lon       = state.lon
            alt       = state.baro_alt
            heading   = state.heading_deg
            vx        = state.flow_vx     # body forward (m/s)
            vy        = state.flow_vy     # body right   (m/s)
            quality   = state.flow_quality
            flow_ok   = state.flow_ok

        # --- integrate position if flow is valid ---
        if flow_ok and quality >= FLOW_QUALITY_MIN:
            vN, vE = _rotate_body_to_ned(vx, vy, heading)
            dlat = vN * elapsed / 111_111.0
            dlon = vE * elapsed / (111_111.0 * math.cos(math.radians(lat)))
            state.update_position(lat + dlat, lon + dlon)
            with state._lock:
                lat = state.lat
                lon = state.lon

        speed_mps = math.hypot(vx, vy)

        # --- build and send GPS packets ---
        now_utc = datetime.now(timezone.utc)
        tow_ms  = (now_utc.hour * 3600 + now_utc.minute * 60 + now_utc.second) * 1000

        pvt     = _ubx_nav_pvt(lat, lon, alt, speed_mps, heading, NUM_SATS_FAKE, now_utc)
        posllh  = _ubx_nav_posllh(lat, lon, alt, tow_ms)
        velned  = _ubx_nav_velned(speed_mps, heading, tow_ms)
        gga, rmc = _nmea_sentences(lat, lon, alt, speed_mps, heading, NUM_SATS_FAKE, now_utc)

        out.write(pvt)
        out.write(posllh)
        out.write(velned)
        out.write(gga)
        out.write(rmc)

        # --- console status every 2 seconds ---
        loop_count += 1
        if loop_count % (UPDATE_HZ * 2) == 0:
            fix_src = "GPS origin" if state.gps_healthy else "DEFAULT"
            flow_s  = f"ok q={quality}" if flow_ok else f"BAD q={quality}"
            print(
                f"[Status] lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m "
                f"hdg={heading:.1f}° spd={speed_mps:.2f}m/s "
                f"flow={flow_s} origin={fix_src}"
            )

        # --- sleep for remainder of tick ---
        elapsed_this_tick = time.time() - now_wall
        sleep_t = max(0.0, dt - elapsed_this_tick)
        time.sleep(sleep_t)


# ============================================================
# ENTRY POINT
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="Optical Flow → GPS emulator for Pixhawk")
    parser.add_argument("--no-real-gps", action="store_true",
                        help="Skip real GPS, use default origin immediately")
    parser.add_argument("--default-lat",  type=float, default=DEFAULT_LAT)
    parser.add_argument("--default-lon",  type=float, default=DEFAULT_LON)
    parser.add_argument("--default-alt",  type=float, default=DEFAULT_ALT)
    parser.add_argument("--update-hz",    type=float, default=UPDATE_HZ,
                        help="GPS packet rate to Pixhawk (default 10 Hz)")
    args = parser.parse_args()

    print("=" * 60)
    print("  Optical Flow → GPS Emulator")
    print("=" * 60)

    # Start background threads
    if not args.no_real_gps:
        GPSOriginReader().start()
    else:
        _use_default_origin()

    FlowReader().start()
    MAVLinkBridge().start()

    # Give MAVLink a moment to connect
    time.sleep(2.0)

    # Main loop (blocking)
    run_integrator_and_emitter()


if __name__ == "__main__":
    main()