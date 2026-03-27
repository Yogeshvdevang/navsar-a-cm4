#!/usr/bin/env python3
"""Standalone MTF-01 optical-flow to Pixhawk MAVLink bridge.

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


def _bool_arg(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


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
    """Parsed MTF-01 sample, matching the main project logic."""

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

        dist_valid = self.distance_mm > 0 and self.dis_status == 1
        self.dist_ok = 1 if dist_valid else 0
        self.dist_cm = self.distance_mm / 10.0 if dist_valid else 0.0
        height_m = self.distance_mm / 1000.0 if dist_valid else 0.0
        self.speed_x = self.flow_vx * height_m
        self.speed_y = self.flow_vy * height_m
        self.flow_ok = 1 if self.flow_status == 1 else 0

    def format_line(self, index):
        return (
            f"[{index:05d}] time_ms={self.time_ms} "
            f"dist_mm={self.distance_mm} dist_cm={self.dist_cm:.1f} "
            f"dist_ok={self.dist_ok} strength={self.strength} precision={self.precision} "
            f"flow_vx={self.flow_vx} flow_vy={self.flow_vy} "
            f"flow_q={self.flow_quality} flow_ok={self.flow_ok} "
            f"speed_x={self.speed_x:.2f} speed_y={self.speed_y:.2f}"
        )


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
            return OpticalFlowSample(bytes(self.msg.payload[: self.msg.len]))
        return None


class MTF01OpticalFlowReader:
    """Threaded MTF-01 reader with the same parsing and heartbeat behavior."""

    def __init__(
        self,
        port,
        baudrate=115200,
        data_frequency=60.0,
        heartbeat_interval_s=0.6,
        print_enabled=False,
        max_flow_raw=None,
    ):
        self.port = port
        self.baudrate = int(baudrate)
        self.data_frequency = float(data_frequency)
        self.heartbeat_interval_s = float(heartbeat_interval_s)
        self.print_enabled = bool(print_enabled)
        self.max_flow_raw = None if max_flow_raw is None else float(max_flow_raw)

        self._ser = None
        self._running = False
        self._sequence = 0
        self._thread = None
        self._lock = threading.Lock()
        self._last_sample = None
        self._last_error = None
        self._msg_count = 0

    def _calculate_checksum(self, data):
        return sum(data) & 0xFF

    def _create_heartbeat(self):
        time_ms = int(time.time() * 1000) & 0xFFFFFFFF
        msg = bytearray([0xEF, 0x01, 0x00, 0x01, self._sequence & 0xFF, 0x0D])
        msg.extend(struct.pack("<I", time_ms))
        msg.extend(bytes([0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]))
        msg.append(self._calculate_checksum(msg))
        self._sequence += 1
        return bytes(msg)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="mtf01-reader")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            try:
                self._ser.setDTR(False)
                self._ser.setRTS(False)
                self._ser.close()
            except Exception:
                pass

    def get_latest(self):
        with self._lock:
            return self._last_sample

    def get_last_error(self):
        with self._lock:
            return self._last_error

    def _store_sample(self, sample):
        with self._lock:
            self._last_sample = sample

    def _store_error(self, exc):
        with self._lock:
            self._last_error = exc

    def _run(self):
        try:
            self._ser = serial.Serial(
                self.port,
                self.baudrate,
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
                self._ser.write(self._create_heartbeat())
                self._ser.flush()
                time.sleep(0.1)

            time.sleep(0.3)
            self._ser.reset_input_buffer()

            parser = MicolinkParser()
            last_emit_time = 0.0
            last_heartbeat_time = 0.0
            delay = (1.0 / self.data_frequency) if self.data_frequency > 0 else 0.0

            while self._running:
                now = time.time()
                if now - last_heartbeat_time >= self.heartbeat_interval_s:
                    self._ser.write(self._create_heartbeat())
                    self._ser.flush()
                    last_heartbeat_time = now

                if self._ser.in_waiting > 0:
                    byte = self._ser.read(1)
                    if byte and parser.parse_char(byte[0]):
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

                        if delay == 0.0 or (now - last_emit_time) >= delay:
                            last_emit_time = now
                            self._msg_count += 1
                            self._store_sample(sample)
                            if self.print_enabled:
                                print(sample.format_line(self._msg_count))
                else:
                    time.sleep(0.01)
        except Exception as exc:
            self._store_error(exc)
        finally:
            self._running = False
            if self._ser and self._ser.is_open:
                try:
                    self._ser.setDTR(False)
                    self._ser.setRTS(False)
                    self._ser.close()
                except Exception:
                    pass


class MavlinkInterface:
    """Minimal standalone MAVLink sender for optical flow and range."""

    def __init__(
        self,
        device,
        baud=115200,
        heartbeat_timeout=15.0,
        source_system=1,
        source_component=None,
        rangefinder_component=None,
    ):
        self.device = device
        self.baud = int(baud)
        if source_component is None:
            source_component = getattr(mavutil.mavlink, "MAV_COMP_ID_PERIPHERAL", 193)
        if rangefinder_component is None:
            rangefinder_component = getattr(mavutil.mavlink, "MAV_COMP_ID_RANGEFINDER", 173)

        self.master = mavutil.mavlink_connection(
            device,
            baud=self.baud,
            source_system=int(source_system),
            source_component=int(source_component),
        )
        self._range_master = None
        if int(rangefinder_component) != int(source_component):
            self._range_master = mavutil.mavlink_connection(
                device,
                baud=self.baud,
                source_system=int(source_system),
                source_component=int(rangefinder_component),
            )

        print(f"[MAVLink] Waiting for heartbeat on {device} ...")
        self.master.wait_heartbeat(timeout=heartbeat_timeout)
        print(
            f"[MAVLink] Heartbeat received "
            f"(sysid={self.master.target_system} compid={self.master.target_component})"
        )

    def send_distance_sensor(
        self,
        distance_m,
        min_distance_m=0.01,
        max_distance_m=8.0,
        orientation=mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
        sensor_type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        sensor_id=0,
        covariance=255,
        time_boot_ms=None,
    ):
        master = self._range_master or self.master
        if time_boot_ms is None:
            time_boot_ms = int(time.time() * 1000) % (2**32)

        master.mav.distance_sensor_send(
            int(time_boot_ms),
            int(max(0.0, float(min_distance_m)) * 100),
            int(max(0.0, float(max_distance_m)) * 100),
            int(max(0.0, float(distance_m)) * 100),
            int(sensor_type),
            int(sensor_id),
            int(orientation),
            int(covariance),
        )

    def send_optical_flow_rad(
        self,
        integrated_x,
        integrated_y,
        integration_time_us,
        distance_m,
        quality,
        time_usec=None,
        sensor_id=0,
        integrated_xgyro=0.0,
        integrated_ygyro=0.0,
        integrated_zgyro=0.0,
        temperature=0,
        time_delta_distance_us=None,
    ):
        if time_usec is None:
            time_usec = int(time.time() * 1_000_000)
        if time_delta_distance_us is None:
            time_delta_distance_us = int(integration_time_us)

        self.master.mav.optical_flow_rad_send(
            int(time_usec),
            int(sensor_id),
            int(integration_time_us),
            float(integrated_x),
            float(integrated_y),
            float(integrated_xgyro),
            float(integrated_ygyro),
            float(integrated_zgyro),
            int(temperature),
            int(quality),
            int(time_delta_distance_us),
            float(distance_m),
        )


class OpticalFlowMavlinkMode:
    """Main-project optical_flow_mavlink behavior copied locally."""

    def __init__(
        self,
        send_interval_s,
        print_enabled=False,
        warn_interval_s=2.0,
        range_min_m=0.01,
        range_max_m=8.0,
    ):
        self.send_interval_s = float(send_interval_s)
        self.print_enabled = bool(print_enabled)
        self.warn_interval_s = float(warn_interval_s)
        self.range_min_m = float(range_min_m)
        self.range_max_m = float(range_max_m)
        self._last_send = 0.0
        self._last_warn = 0.0
        self._last_time_ms = None
        self.last_payload = None

    def _warn(self, now, message):
        if now - self._last_warn >= self.warn_interval_s:
            print(message)
            self._last_warn = now

    def handle(self, now, sample, mavlink_interface):
        if mavlink_interface is None:
            self._warn(now, "OFLOW->MAV: MAVLink unavailable.")
            return
        if sample is None:
            self._warn(now, "OFLOW->MAV: waiting for optical flow samples...")
            return
        if now - self._last_send < self.send_interval_s:
            return

        dt_s = None
        if self._last_time_ms is not None and sample.time_ms is not None:
            dt_ms = sample.time_ms - self._last_time_ms
            if dt_ms > 0:
                dt_s = dt_ms / 1000.0
        if dt_s is None or dt_s <= 0.0:
            dt_s = self.send_interval_s if self.send_interval_s > 0 else 0.02

        self._last_time_ms = sample.time_ms

        integration_time_us = max(1, int(dt_s * 1_000_000))
        integrated_x = float(sample.flow_vx) * dt_s
        integrated_y = float(sample.flow_vy) * dt_s
        quality = int(max(0, min(255, sample.flow_quality)))
        distance_m = float(sample.distance_mm) / 1000.0 if sample.dist_ok else -1.0

        covariance = 255
        if distance_m > 0 and quality > 0:
            covariance = int(max(0, min(50, 50 - (quality // 5))))

        mavlink_interface.send_optical_flow_rad(
            integrated_x=integrated_x,
            integrated_y=integrated_y,
            integration_time_us=integration_time_us,
            distance_m=distance_m,
            quality=quality,
        )
        if distance_m > 0:
            mavlink_interface.send_distance_sensor(
                distance_m=distance_m,
                min_distance_m=self.range_min_m,
                max_distance_m=self.range_max_m,
                covariance=covariance,
            )

        self.last_payload = {
            "time_s": now,
            "time_ms": sample.time_ms,
            "flow_vx": sample.flow_vx,
            "flow_vy": sample.flow_vy,
            "integrated_x": integrated_x,
            "integrated_y": integrated_y,
            "dist_cm": sample.dist_cm,
            "distance_m": distance_m,
            "quality": quality,
        }

        if self.print_enabled:
            print(
                "OFLOW->MAV: "
                f"dx={integrated_x:.4f} rad dy={integrated_y:.4f} rad "
                f"dist={distance_m:.2f} m q={quality}"
            )

        self._last_send = now


def parse_args():
    parser = argparse.ArgumentParser(
        description="Standalone MTF-01 optical flow to MAVLink bridge."
    )
    parser.add_argument("--flow-port", default="/dev/ttyAMA3", help="MTF-01 serial port.")
    parser.add_argument("--flow-baud", type=int, default=115200, help="MTF-01 baud rate.")
    parser.add_argument(
        "--flow-rate-hz",
        type=float,
        default=60.0,
        help="Maximum optical-flow sample rate to emit.",
    )
    parser.add_argument(
        "--flow-heartbeat-s",
        type=float,
        default=0.6,
        help="Heartbeat interval sent to the MTF-01.",
    )
    parser.add_argument(
        "--flow-print",
        type=_bool_arg,
        default=True,
        help="Print decoded optical-flow samples.",
    )
    parser.add_argument(
        "--flow-max-raw",
        type=float,
        default=120.0,
        help="Reject |flow_vx| or |flow_vy| at or above this raw value.",
    )
    parser.add_argument(
        "--mav-port",
        default="/dev/ttyACM1",
        help="Pixhawk MAVLink serial device or UDP endpoint.",
    )
    parser.add_argument("--mav-baud", type=int, default=115200, help="MAVLink baud rate.")
    parser.add_argument(
        "--source-system",
        type=int,
        default=1,
        help="MAVLink source system id.",
    )
    parser.add_argument(
        "--source-component",
        type=int,
        default=192,
        help="MAVLink source component id for optical flow messages.",
    )
    parser.add_argument(
        "--rangefinder-component",
        type=int,
        default=191,
        help="MAVLink source component id for distance-sensor messages.",
    )
    parser.add_argument(
        "--send-interval-s",
        type=float,
        default=0.05,
        help="OPTICAL_FLOW_RAD send interval.",
    )
    parser.add_argument(
        "--range-min-m",
        type=float,
        default=0.01,
        help="DISTANCE_SENSOR minimum range.",
    )
    parser.add_argument(
        "--range-max-m",
        type=float,
        default=8.0,
        help="DISTANCE_SENSOR maximum range.",
    )
    parser.add_argument(
        "--mav-print",
        type=_bool_arg,
        default=True,
        help="Print outgoing MAVLink optical-flow summary.",
    )
    parser.add_argument(
        "--poll-interval-s",
        type=float,
        default=0.01,
        help="Main loop sleep between checks.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    print("Starting standalone optical-flow MAVLink bridge")
    print(
        f"Flow: {args.flow_port} @ {args.flow_baud} | "
        f"MAVLink: {args.mav_port} @ {args.mav_baud}"
    )

    optical_reader = MTF01OpticalFlowReader(
        port=args.flow_port,
        baudrate=args.flow_baud,
        data_frequency=args.flow_rate_hz,
        heartbeat_interval_s=args.flow_heartbeat_s,
        print_enabled=args.flow_print,
        max_flow_raw=args.flow_max_raw,
    )

    mavlink = MavlinkInterface(
        device=args.mav_port,
        baud=args.mav_baud,
        source_system=args.source_system,
        source_component=args.source_component,
        rangefinder_component=args.rangefinder_component,
    )

    mode = OpticalFlowMavlinkMode(
        send_interval_s=args.send_interval_s,
        print_enabled=args.mav_print,
        range_min_m=args.range_min_m,
        range_max_m=args.range_max_m,
    )

    optical_reader.start()
    print("Bridge running. Press Ctrl+C to stop.")

    try:
        while True:
            now = time.time()
            sample = optical_reader.get_latest()
            mode.handle(now, sample, mavlink)

            last_error = optical_reader.get_last_error()
            if last_error is not None:
                raise RuntimeError(f"Optical flow reader failed: {last_error}")

            time.sleep(max(0.0, args.poll_interval_s))
    except KeyboardInterrupt:
        print("\nStopping bridge...")
    finally:
        optical_reader.stop()


if __name__ == "__main__":
    main()
