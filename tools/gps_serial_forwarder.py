"""GPS Serial Forwarder module. Provides gps serial forwarder utilities for NAVISAR."""

import argparse
import time

import serial


def main():
    """Forward raw GPS NMEA data between serial ports."""
    parser = argparse.ArgumentParser(description="Forward GPS serial data to Pixhawk.")
    parser.add_argument("--in-port", required=True, help="GPS input port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--out-port", required=True, help="Pixhawk output port (e.g. /dev/ttyUSB1)")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate")
    parser.add_argument("--stats-interval", type=float, default=5.0, help="Seconds between stats")
    args = parser.parse_args()

    gps_in = serial.Serial(args.in_port, args.baud, timeout=0.5)
    gps_out = serial.Serial(args.out_port, args.baud, timeout=0)
    print(
        f"Forwarding {args.in_port} -> {args.out_port} @ {args.baud} baud"
    )

    last_stats = time.time()
    bytes_in = 0
    lines_in = 0
    while True:
        line = gps_in.readline()
        if line:
            gps_out.write(line)
            bytes_in += len(line)
            lines_in += 1
        now = time.time()
        if now - last_stats >= args.stats_interval:
            print(f"Forwarded {lines_in} lines / {bytes_in} bytes")
            last_stats = now
            bytes_in = 0
            lines_in = 0


if __name__ == "__main__":
    main()
