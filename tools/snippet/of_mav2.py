#!/usr/bin/env python3
"""Standalone MTF-01 optical-flow to Pixhawk MAVLink bridge.

Sends:
  OPTICAL_FLOW     (#100)  — read by ArduPilot FLOW_TYPE=6, shows in opt_qua
  DISTANCE_SENSOR  (#132)  — read by ArduPilot RNGFND1_TYPE=10, shows in rangefinder1

Dependencies:
  pip install pyserial pymavlink
"""

import argparse
import struct
import threading
import time

import serial
from pymavlink import mavutil


MICOLINK_MSG_HEAD = 0xEF
MICOLINK_MSG_ID_RANGE_SENSOR = 0x51
MICOLINK_MAX_PAYLOAD_LEN = 64

FOCAL_LENGTH_PX = 16.0   # tune for your lens


def _bool_arg(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


# ─────────────────────────────────────────────────────────────
# Micolink parser  (unchanged from original)
# ─────────────────────────────────────────────────────────────

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


class OpticalFlowSample:
    def __init__(self, payload_bytes):
        data = struct.unpack("<IIBBBBhhBBH", payload_bytes[:24])
        self.time_ms      = data[0]
        self.distance_mm  = data[1]
        self.strength     = data[2]
        self.precision    = data[3]
        self.dis_status   = data[4]
        self.reserved1    = data[5]
        self.flow_vx      = data[6]
        self.flow_vy      = data[7]
        self.flow_quality = data[8]
        self.flow_status  = data[9]
        self.reserved2    = data[10]

        dist_valid        = self.distance_mm > 0 and self.dis_status == 1
        self.dist_ok      = 1 if dist_valid else 0
        self.dist_cm      = self.distance_mm / 10.0 if dist_valid else 0.0
        height_m          = self.distance_mm / 1000.0 if dist_valid else 0.0
        self.height_m     = height_m
        self.speed_x      = self.flow_vx * height_m
        self.speed_y      = self.flow_vy * height_m
        self.flow_ok      = 1 if self.flow_status == 1 else 0

    def format_line(self, index):
        return (
            f"[{index:05d}] time_ms={self.time_ms} "
            f"dist_mm={self.distance_mm} dist_cm={self.dist_cm:.1f} "
            f"dist_ok={self.dist_ok} strength={self.strength} "
            f"flow_vx={self.flow_vx} flow_vy={self.flow_vy} "
            f"flow_q={self.flow_quality} flow_ok={self.flow_ok} "
            f"speed_x={self.speed_x:.2f} speed_y={self.speed_y:.2f}"
        )


class MicolinkParser:
    def __init__(self):
        self.msg = MicolinkMessage()

    def calculate_checksum(self, msg):
        cs = msg.head + msg.dev_id + msg.sys_id + msg.msg_id + msg.seq + msg.len
        for i in range(msg.len):
            cs += msg.payload[i]
        return cs & 0xFF

    def parse_char(self, data):
        msg = self.msg
        if msg.status == 0:
            if data == MICOLINK_MSG_HEAD:
                msg.head = data; msg.status = 1
        elif msg.status == 1:
            msg.dev_id = data; msg.status = 2
        elif msg.status == 2:
            msg.sys_id = data; msg.status = 3
        elif msg.status == 3:
            msg.msg_id = data; msg.status = 4
        elif msg.status == 4:
            msg.seq = data; msg.status = 5
        elif msg.status == 5:
            msg.len = data
            if msg.len == 0:
                msg.status = 7
            elif msg.len > MICOLINK_MAX_PAYLOAD_LEN:
                msg.reset()
            else:
                msg.status = 6
        elif msg.status == 6:
            msg.payload[msg.payload_cnt] = data
            msg.payload_cnt += 1
            if msg.payload_cnt == msg.len:
                msg.payload_cnt = 0; msg.status = 7
        elif msg.status == 7:
            msg.checksum = data; msg.status = 0
            if self.calculate_checksum(msg) == msg.checksum:
                return True
            msg.reset()
        else:
            msg.reset()
        return False

    def decode_message(self):
        if self.msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR:
            return OpticalFlowSample(bytes(self.msg.payload[:self.msg.len]))
        return None


# ─────────────────────────────────────────────────────────────
# Sensor reader thread
# ─────────────────────────────────────────────────────────────

class MTF01OpticalFlowReader:
    def __init__(self, port, baudrate=115200, data_frequency=60.0,
                 heartbeat_interval_s=0.6, print_enabled=False, max_flow_raw=None):
        self.port                = port
        self.baudrate            = int(baudrate)
        self.data_frequency      = float(data_frequency)
        self.heartbeat_interval_s = float(heartbeat_interval_s)
        self.print_enabled       = bool(print_enabled)
        self.max_flow_raw        = None if max_flow_raw is None else float(max_flow_raw)
        self._ser                = None
        self._running            = False
        self._sequence           = 0
        self._thread             = None
        self._lock               = threading.Lock()
        self._last_sample        = None
        self._last_error         = None
        self._msg_count          = 0

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

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True, name="mtf01")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            try:
                self._ser.setDTR(False); self._ser.setRTS(False); self._ser.close()
            except Exception:
                pass

    def get_latest(self):
        with self._lock:
            return self._last_sample

    def get_last_error(self):
        with self._lock:
            return self._last_error

    def _run(self):
        try:
            self._ser = serial.Serial(
                self.port, self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=1,
                xonxoff=False, rtscts=False, dsrdtr=False,
            )
            self._ser.setDTR(True); self._ser.setRTS(True)
            time.sleep(0.1)
            self._ser.reset_input_buffer(); self._ser.reset_output_buffer()

            for _ in range(3):
                self._ser.write(self._heartbeat()); self._ser.flush(); time.sleep(0.1)
            time.sleep(0.3)
            self._ser.reset_input_buffer()

            parser           = MicolinkParser()
            last_emit        = 0.0
            last_hb          = 0.0
            delay            = (1.0 / self.data_frequency) if self.data_frequency > 0 else 0.0

            while self._running:
                now = time.time()
                if now - last_hb >= self.heartbeat_interval_s:
                    self._ser.write(self._heartbeat()); self._ser.flush(); last_hb = now

                if self._ser.in_waiting > 0:
                    byte = self._ser.read(1)
                    if byte and parser.parse_char(byte[0]):
                        sample = parser.decode_message()
                        if sample is None:
                            continue
                        if (self.max_flow_raw is not None and
                                (abs(sample.flow_vx) >= self.max_flow_raw or
                                 abs(sample.flow_vy) >= self.max_flow_raw)):
                            sample.flow_quality = 0
                            sample.flow_ok      = 0
                            sample.speed_x      = 0.0
                            sample.speed_y      = 0.0
                        if delay == 0.0 or (now - last_emit) >= delay:
                            last_emit = now
                            self._msg_count += 1
                            with self._lock:
                                self._last_sample = sample
                            if self.print_enabled:
                                print(sample.format_line(self._msg_count))
                else:
                    time.sleep(0.005)
        except Exception as exc:
            with self._lock:
                self._last_error = exc
        finally:
            self._running = False
            if self._ser and self._ser.is_open:
                try:
                    self._ser.setDTR(False); self._ser.setRTS(False); self._ser.close()
                except Exception:
                    pass


# ─────────────────────────────────────────────────────────────
# MAVLink interface — sends OPTICAL_FLOW (#100) + DISTANCE_SENSOR (#132)
# ─────────────────────────────────────────────────────────────

class MavlinkInterface:
    def __init__(self, device, baud=115200, heartbeat_timeout=15.0,
                 source_system=1, source_component=192):
        self.device = device
        self.baud   = int(baud)

        is_serial = not device.startswith(("udp", "tcp"))
        self.master = mavutil.mavlink_connection(
            device,
            baud             = self.baud if is_serial else None,
            source_system    = int(source_system),
            source_component = int(source_component),
        )

        print(f"[MAVLink] Waiting for heartbeat on {device} …")
        self.master.wait_heartbeat(timeout=heartbeat_timeout)
        print(f"[MAVLink] ✓ Heartbeat received "
              f"(sysid={self.master.target_system} "
              f"compid={self.master.target_component})")

        # MAVLink heartbeat thread (1 Hz) — required by ArduPilot
        self._running    = True
        self._hb_thread  = threading.Thread(target=self._hb_loop, daemon=True, name="mav-hb")
        self._hb_thread.start()

        self.flow_sent  = 0
        self.range_sent = 0

    def _hb_loop(self):
        while self._running:
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except Exception as e:
                print(f"[MAVLink] heartbeat error: {e}")
            time.sleep(1.0)

    def send_optical_flow(self, sample: OpticalFlowSample):
        """
        Send OPTICAL_FLOW message (#100).
        This is what ArduPilot FLOW_TYPE=6 reads to populate opt_qua
        in the Mission Planner Status window.

        Fields:
          flow_x / flow_y    — raw pixel flow (int16, dpix)
          flow_comp_m_x/y    — compensated velocity m/s
          quality            — 0-255
          ground_distance    — metres from lidar
        """
        time_usec  = int(time.time() * 1_000_000)
        h          = sample.height_m

        # Compensated velocity: pixel_flow × height / focal_length
        comp_x = (sample.flow_vx * h / FOCAL_LENGTH_PX) if h > 0 else 0.0
        comp_y = (sample.flow_vy * h / FOCAL_LENGTH_PX) if h > 0 else 0.0

        try:
            self.master.mav.optical_flow_send(
                time_usec,               # uint64  µs
                0,                       # uint8   sensor_id
                int(sample.flow_vx),     # int16   flow_x  [dpix]
                int(sample.flow_vy),     # int16   flow_y  [dpix]
                float(comp_x),           # float   flow_comp_m_x [m/s]
                float(comp_y),           # float   flow_comp_m_y [m/s]
                int(sample.flow_quality),# uint8   quality
                float(h),                # float   ground_distance [m]
            )
            self.flow_sent += 1
            if self.flow_sent % 200 == 0:
                print(f"[MAVLink] flow_sent={self.flow_sent}  range_sent={self.range_sent}")
        except Exception as e:
            print(f"[MAVLink] ⚠ optical_flow_send error: {e}")

    def send_distance_sensor(self, sample: OpticalFlowSample,
                              range_min_m=0.01, range_max_m=8.0):
        """
        Send DISTANCE_SENSOR message (#132).
        Populates rangefinder1 in Mission Planner Status window.
        """
        if not sample.dist_ok:
            return
        distance_m  = sample.distance_mm / 1000.0
        quality     = int(sample.flow_quality)
        covariance  = int(max(0, min(50, 50 - (quality // 5)))) if quality > 0 else 255

        try:
            self.master.mav.distance_sensor_send(
                int(time.time() * 1000) % (2**32),   # time_boot_ms
                int(range_min_m * 100),               # min_distance cm
                int(range_max_m * 100),               # max_distance cm
                int(distance_m * 100),                # current_distance cm
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                0,                                    # sensor_id
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,  # facing down
                covariance,
            )
            self.range_sent += 1
        except Exception as e:
            print(f"[MAVLink] ⚠ distance_sensor_send error: {e}")

    def close(self):
        self._running = False
        self._hb_thread.join(timeout=2)
        self.master.close()
        print(f"[MAVLink] Closed. flow_sent={self.flow_sent}  range_sent={self.range_sent}")


# ─────────────────────────────────────────────────────────────
# Main bridge loop
# ─────────────────────────────────────────────────────────────

class OpticalFlowBridge:
    def __init__(self, send_interval_s=0.02, print_enabled=False,
                 warn_interval_s=2.0, range_min_m=0.01, range_max_m=8.0):
        self.send_interval_s = float(send_interval_s)
        self.print_enabled   = bool(print_enabled)
        self.warn_interval_s = float(warn_interval_s)
        self.range_min_m     = float(range_min_m)
        self.range_max_m     = float(range_max_m)
        self._last_send      = 0.0
        self._last_warn      = 0.0

    def _warn(self, now, msg):
        if now - self._last_warn >= self.warn_interval_s:
            print(msg); self._last_warn = now

    def handle(self, now, sample, mav: MavlinkInterface):
        if mav is None:
            self._warn(now, "[Bridge] MAVLink unavailable"); return
        if sample is None:
            self._warn(now, "[Bridge] Waiting for sensor data …"); return
        if now - self._last_send < self.send_interval_s:
            return

        mav.send_optical_flow(sample)
        mav.send_distance_sensor(sample, self.range_min_m, self.range_max_m)

        if self.print_enabled:
            print(f"[Bridge] flow_vx={sample.flow_vx} flow_vy={sample.flow_vy} "
                  f"quality={sample.flow_quality} dist_cm={sample.dist_cm:.1f}")
        self._last_send = now


# ─────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="MTF-01 optical flow → Pixhawk MAVLink bridge")
    p.add_argument("--flow-port",          default="/dev/ttyAMA3")
    p.add_argument("--flow-baud",          type=int,   default=115200)
    p.add_argument("--flow-rate-hz",       type=float, default=60.0)
    p.add_argument("--flow-heartbeat-s",   type=float, default=0.6)
    p.add_argument("--flow-print",         type=_bool_arg, default=True)
    p.add_argument("--flow-max-raw",       type=float, default=120.0)
    p.add_argument("--mav-port",           default="/dev/ttyACM0")
    p.add_argument("--mav-baud",           type=int,   default=115200)
    p.add_argument("--source-system",      type=int,   default=1)
    p.add_argument("--source-component",   type=int,   default=192)
    p.add_argument("--send-interval-s",    type=float, default=0.02,
                   help="How often to send MAVLink messages (default 0.02 = 50 Hz)")
    p.add_argument("--range-min-m",        type=float, default=0.01)
    p.add_argument("--range-max-m",        type=float, default=8.0)
    p.add_argument("--mav-print",          type=_bool_arg, default=True)
    p.add_argument("--poll-interval-s",    type=float, default=0.005)
    return p.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("  MTF-01 → Pixhawk MAVLink Bridge")
    print("=" * 60)
    print(f"  Sensor : {args.flow_port} @ {args.flow_baud}")
    print(f"  Pixhawk: {args.mav_port}  @ {args.mav_baud}")
    print(f"  Rate   : {1.0/args.send_interval_s:.0f} Hz  "
          f"(send_interval={args.send_interval_s}s)")
    print()

    reader = MTF01OpticalFlowReader(
        port               = args.flow_port,
        baudrate           = args.flow_baud,
        data_frequency     = args.flow_rate_hz,
        heartbeat_interval_s = args.flow_heartbeat_s,
        print_enabled      = args.flow_print,
        max_flow_raw       = args.flow_max_raw,
    )

    mav = MavlinkInterface(
        device           = args.mav_port,
        baud             = args.mav_baud,
        source_system    = args.source_system,
        source_component = args.source_component,
    )

    bridge = OpticalFlowBridge(
        send_interval_s = args.send_interval_s,
        print_enabled   = args.mav_print,
        range_min_m     = args.range_min_m,
        range_max_m     = args.range_max_m,
    )

    reader.start()
    print("[Bridge] Running — press Ctrl+C to stop\n")

    try:
        while True:
            now    = time.time()
            sample = reader.get_latest()
            bridge.handle(now, sample, mav)

            err = reader.get_last_error()
            if err is not None:
                raise RuntimeError(f"Sensor reader failed: {err}")

            time.sleep(args.poll_interval_s)
    except KeyboardInterrupt:
        print("\n[Bridge] Stopping …")
    finally:
        reader.stop()
        mav.close()


if __name__ == "__main__":
    main()