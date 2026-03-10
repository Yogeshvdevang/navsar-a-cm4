#!/usr/bin/env python3
import argparse
from collections import deque
from dataclasses import dataclass
import struct
import time
from typing import Optional

import serial

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
    parser = argparse.ArgumentParser(description="Check optical flow sensor fields over MicoLink")
    parser.add_argument("--port", default="/dev/ttyAMA3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--count", type=int, default=0, help="Stop after N valid frames (0 = infinite)")
    parser.add_argument(
        "--flow-x-offset",
        type=int,
        default=0,
        help="Offset added to flow_vel_x before display",
    )
    parser.add_argument(
        "--flow-y-offset",
        type=int,
        default=0,
        help="Offset added to flow_vel_y before display",
    )
    parser.add_argument(
        "--deadband",
        type=int,
        default=0,
        help="Clamp |flow| below this to zero (applied after offset)",
    )
    parser.add_argument(
        "--avg-window",
        type=int,
        default=0,
        help="Moving average window size (0 = off)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.05)
    decoder = MicoLinkDecoder()
    seen = 0
    print(f"Listening on {args.port} @ {args.baud}...")
    print("flow_vel_x flow_vel_y")
    avg_x = deque()
    avg_y = deque()
    try:
        while True:
            data = ser.read(256)
            for byte in data:
                parsed = decoder.parse_byte(byte)
                if parsed is None:
                    continue
                flow_x = parsed.flow_vel_x + args.flow_x_offset
                flow_y = parsed.flow_vel_y + args.flow_y_offset

                if args.deadband:
                    if abs(flow_x) < args.deadband:
                        flow_x = 0
                    if abs(flow_y) < args.deadband:
                        flow_y = 0

                if args.avg_window and args.avg_window > 1:
                    avg_x.append(flow_x)
                    avg_y.append(flow_y)
                    if len(avg_x) > args.avg_window:
                        avg_x.popleft()
                    if len(avg_y) > args.avg_window:
                        avg_y.popleft()
                    flow_x = int(sum(avg_x) / len(avg_x))
                    flow_y = int(sum(avg_y) / len(avg_y))
                print("flow_vel_x={} flow_vel_y={}".format(flow_x, flow_y))
                seen += 1
                if args.count and seen >= args.count:
                    return
            time.sleep(0.01)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
