import argparse
from dataclasses import dataclass
import struct
import time
from typing import Generator, Optional

import serial
from pymavlink import mavutil

COM_PORT = "/dev/ttyAMA3"
BAUD_RATE = 115200

# SERIAL_PORT = "/dev/ttyAMA0"
# BAUDRATE = 9600


MICOLINK_MSG_HEAD = 0xEF
MICOLINK_MAX_PAYLOAD_LEN = 64
MICOLINK_MSG_ID_RANGE_SENSOR = 0x51
MICOLINK_RANGE_PAYLOAD_LEN = 0x14


@dataclass
class RangeFlowData:
    time_ms: int
    distance_mm: int
    strength: int
    precision: int
    distance_status: int
    flow_vel_x: int
    flow_vel_y: int
    flow_quality: int
    flow_status: int


class MicoLinkDecoder:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.status = 0
        self.dev_id = 0
        self.sys_id = 0
        self.msg_id = 0
        self.seq = 0
        self.length = 0
        self.payload = bytearray()
        self.raw = bytearray()

    def checksum_ok(self, raw_bytes: bytearray) -> bool:
        return (sum(raw_bytes[:-1]) & 0xFF) == raw_bytes[-1]

    def parse_byte(self, byte: int) -> Optional[RangeFlowData]:
        if self.status == 0:
            if byte == MICOLINK_MSG_HEAD:
                self.raw = bytearray([byte])
                self.status = 1
        elif self.status == 1:
            self.dev_id = byte
            self.raw.append(byte)
            self.status = 2
        elif self.status == 2:
            self.sys_id = byte
            self.raw.append(byte)
            self.status = 3
        elif self.status == 3:
            self.msg_id = byte
            self.raw.append(byte)
            self.status = 4
        elif self.status == 4:
            self.seq = byte
            self.raw.append(byte)
            self.status = 5
        elif self.status == 5:
            self.length = byte
            self.raw.append(byte)
            if self.length == 0:
                self.status = 7
            elif self.length > MICOLINK_MAX_PAYLOAD_LEN:
                self.reset()
            else:
                self.payload = bytearray()
                self.status = 6
        elif self.status == 6:
            self.payload.append(byte)
            self.raw.append(byte)
            if len(self.payload) == self.length:
                self.status = 7
        elif self.status == 7:
            self.raw.append(byte)
            if self.checksum_ok(self.raw):
                result = self.decode_message()
                self.reset()
                return result
            self.reset()
        return None

    def decode_message(self) -> Optional[RangeFlowData]:
        if self.msg_id != MICOLINK_MSG_ID_RANGE_SENSOR:
            return None
        if self.length != MICOLINK_RANGE_PAYLOAD_LEN or len(self.payload) != MICOLINK_RANGE_PAYLOAD_LEN:
            return None
        fmt = "<I I B B B B h h B B H".replace(" ", "")
        unpacked = struct.unpack(fmt, self.payload)
        return RangeFlowData(
            time_ms=unpacked[0],
            distance_mm=unpacked[1],
            strength=unpacked[2],
            precision=unpacked[3],
            distance_status=unpacked[4],
            flow_vel_x=unpacked[6],
            flow_vel_y=unpacked[7],
            flow_quality=unpacked[8],
            flow_status=unpacked[9],
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read optical flow data from serial")
    parser.add_argument("--port", default=COM_PORT)
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--no-heartbeat", action="store_true", help="Skip waiting for MAVLink heartbeat")
    parser.add_argument("--mavlink1", action="store_true", help="Force MAVLink1 framing")
    parser.add_argument(
        "--protocol",
        choices=("micolink", "mavlink"),
        default="micolink",
        help="Sensor protocol on the serial port",
    )
    return parser.parse_args()


def open_mavlink(
    port: str,
    baud: int,
    wait_heartbeat: bool = True,
    mavlink20: bool = True,
) -> mavutil.mavlink_connection:
    mav = mavutil.mavlink_connection(port, baud=baud, mavlink20=mavlink20)
    if wait_heartbeat:
        print("Waiting for heartbeat...")
        mav.wait_heartbeat()
        print("Connected!")
    else:
        print("Connected (heartbeat disabled).")
    return mav


def read_messages(
    mav: mavutil.mavlink_connection,
    blocking: bool = False,
    sleep_s: float = 0.01,
) -> Generator[object, None, None]:
    while True:
        msg = mav.recv_match(blocking=blocking)
        if msg is None:
            time.sleep(sleep_s)
            continue
        yield msg


def run_mavlink(args: argparse.Namespace) -> None:
    mav = open_mavlink(
        args.port,
        args.baud,
        wait_heartbeat=not args.no_heartbeat,
        mavlink20=not args.mavlink1,
    )
    for msg in read_messages(mav, blocking=False):
        msg_type = msg.get_type()
        if msg_type == "OPTICAL_FLOW_RAD":
            print(
                "OPTICAL_FLOW_RAD flow_x={:.6f} flow_y={:.6f} quality={}".format(
                    msg.integrated_x,
                    msg.integrated_y,
                    msg.quality,
                )
            )
        elif msg_type == "OPTICAL_FLOW":
            print(
                "OPTICAL_FLOW flow_vx={:.6f} flow_vy={:.6f} quality={}".format(
                    msg.flow_comp_m_x,
                    msg.flow_comp_m_y,
                    msg.quality,
                )
            )
        elif msg_type in ("BAD_DATA", "UNKNOWN"):
            continue


def run_micolink(args: argparse.Namespace) -> None:
    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.05)
    print(f"Listening (MicoLink) on {args.port} @ {args.baud}...")
    decoder = MicoLinkDecoder()
    try:
        while True:
            data = ser.read(256)
            for byte in data:
                parsed = decoder.parse_byte(byte)
                if parsed is None:
                    continue
                print(
                    "distance_mm={:>5} strength={:>3} precision={:>3} status={} "
                    "flow_vel_x={:>4} flow_vel_y={:>4} flow_quality={:>3} flow_status={}".format(
                        parsed.distance_mm,
                        parsed.strength,
                        parsed.precision,
                        parsed.distance_status,
                        parsed.flow_vel_x,
                        parsed.flow_vel_y,
                        parsed.flow_quality,
                        parsed.flow_status,
                    )
                )
    finally:
        ser.close()


def main() -> None:
    args = parse_args()
    if args.protocol == "micolink":
        run_micolink(args)
    else:
        run_mavlink(args)


if __name__ == "__main__":
    main()