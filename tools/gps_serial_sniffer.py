"""GPS Serial Sniffer module. Provides gps serial sniffer utilities for NAVISAR."""

import argparse
import time

import serial


def _detect_format(sample):
    """Guess GPS message format from leading bytes."""
    if not sample:
        return "unknown"
    if sample.startswith(b"$"):
        return "nmea"
    if sample.startswith(b"\xb5\x62"):
        return "ubx"
    return "unknown"


def _safe_decode(sample):
    """Decode bytes as ASCII, returning empty on failure."""
    try:
        return sample.decode("ascii", errors="ignore").strip()
    except Exception:
        return ""


def main():
    """CLI tool to sniff GPS serial data format."""
    parser = argparse.ArgumentParser(description="Sniff GPS serial data format.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate")
    parser.add_argument("--duration", type=float, default=5.0, help="Seconds to sniff")
    parser.add_argument("--max-lines", type=int, default=50, help="Max lines to print")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    print(f"Sniffing {args.port} @ {args.baud} for {args.duration}s...")

    start = time.time()
    lines = 0
    last_format = "unknown"
    while time.time() - start < args.duration and lines < args.max_lines:
        raw = ser.readline()
        if not raw:
            continue
        fmt = _detect_format(raw)
        if fmt != "unknown":
            last_format = fmt
        if fmt == "nmea":
            print(f"NMEA: {_safe_decode(raw)}")
        elif fmt == "ubx":
            print(f"UBX: {raw[:20].hex()}...")
        else:
            text = _safe_decode(raw)
            if text:
                print(f"RAW: {text}")
            else:
                print(f"RAW: {raw[:20].hex()}...")
        lines += 1

    print(f"Detected format: {last_format}")


if __name__ == "__main__":
    main()
