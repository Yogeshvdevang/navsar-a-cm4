#!/usr/bin/env python3
"""
Read raw GPS serial data and save it to a .txt log file.
"""

import argparse
import string
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import serial


DEFAULT_PORT = "/dev/ttyAMA5"
DEFAULT_BAUD = 230400
DEFAULT_LOG_DIR = Path(__file__).resolve().parent / "logs"
RATE_WINDOW_SECONDS = 3.0


def parse_args():
    parser = argparse.ArgumentParser(description="Save raw GPS serial data to a TXT file.")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"GPS serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument(
        "--output",
        help="Output .txt file path. Default: tools/snippet/logs/gps_raw_<timestamp>.txt",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Log duration in seconds. Use 0 to run until Ctrl+C (default: 0)",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=256,
        help="Bytes to read per iteration (default: 256)",
    )
    parser.add_argument(
        "--rate-window",
        type=float,
        default=RATE_WINDOW_SECONDS,
        help="Window in seconds for estimating incoming data frequency (default: 3.0)",
    )
    return parser.parse_args()


def make_output_path(output_arg):
    if output_arg:
        return Path(output_arg).expanduser().resolve()

    DEFAULT_LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (DEFAULT_LOG_DIR / f"gps_raw_{stamp}.txt").resolve()


def printable_ascii(data):
    allowed = set(string.printable) - {"\x0b", "\x0c"}
    chars = []
    for byte in data:
        ch = chr(byte)
        chars.append(ch if ch in allowed and ch not in "\r\n\t" else ".")
    return "".join(chars)


def main():
    args = parse_args()
    output_path = make_output_path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[GPS] Opening {args.port} @ {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as exc:
        raise SystemExit(f"Cannot open GPS port: {exc}")

    print(f"[GPS] Logging raw data to: {output_path}")
    print("[GPS] Press Ctrl+C to stop." if args.duration <= 0 else f"[GPS] Logging for {args.duration:.1f}s")

    start = time.time()
    total_bytes = 0
    arrival_times = deque()
    last_rate_print = 0.0

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write(f"# GPS raw log\n")
        handle.write(f"# port={args.port} baud={args.baud}\n")
        handle.write(f"# started={datetime.now().isoformat()}\n")
        handle.write("# format: timestamp | hex | ascii\n")

        try:
            while True:
                if args.duration > 0 and (time.time() - start) >= args.duration:
                    break

                data = ser.read(args.chunk_size)
                if not data:
                    continue

                total_bytes += len(data)
                now = time.time()
                timestamp = datetime.now().isoformat(timespec="milliseconds")
                hex_data = data.hex(" ")
                ascii_data = printable_ascii(data)
                arrival_times.append(now)

                while arrival_times and (now - arrival_times[0]) > args.rate_window:
                    arrival_times.popleft()

                if len(arrival_times) >= 2:
                    elapsed = arrival_times[-1] - arrival_times[0]
                    estimated_hz = (len(arrival_times) - 1) / elapsed if elapsed > 0 else 0.0
                else:
                    estimated_hz = 0.0

                handle.write(f"{timestamp} | {hex_data} | {ascii_data}\n")
                handle.flush()

                print(
                    f"[{timestamp}] bytes={len(data):3d} "
                    f"estimated_rate={estimated_hz:.2f}Hz hex={hex_data}"
                )

                if now - last_rate_print >= 1.0 and estimated_hz > 0:
                    rounded_hz = round(estimated_hz)
                    print(
                        f"[GPS] Incoming data rate looks close to {rounded_hz} Hz "
                        f"(measured {estimated_hz:.2f} Hz over {args.rate_window:.1f}s)"
                    )
                    last_rate_print = now

        except KeyboardInterrupt:
            print("\n[GPS] Stopped by user.")
        finally:
            ser.close()

    print(f"[GPS] Saved {total_bytes} bytes to {output_path}")


if __name__ == "__main__":
    main()
