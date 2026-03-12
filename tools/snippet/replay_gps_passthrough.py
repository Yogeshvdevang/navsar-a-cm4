#!/usr/bin/env python3
"""Replay a captured GPS passthrough log to the Pixhawk GPS port."""

from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

import serial


DEFAULT_TXT_FILE = Path("data/gps_passthrough_logs/20260221_141704.txt")
DEFAULT_FREQUENCY_HZ = 10.0
DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 230400
DEFAULT_LOOP = True
DEFAULT_USE_LOG_TIMING = True


def _decode_logged_payload(payload: bytes) -> bytes:
    """Convert logged escaped line endings back into raw serial bytes."""
    return payload.replace(b"\\r", b"\r").replace(b"\\n", b"\n")


def _format_payload_for_terminal(payload: bytes) -> str:
    """Render payload bytes in a readable terminal form."""
    text = payload.decode("latin-1", errors="replace")
    return (
        text.replace("\r", "\\r")
        .replace("\n", "\\n\n")
    )


def _parse_log_timestamp(raw_ts: bytes) -> float | None:
    """Parse ISO8601 timestamp from the passthrough log."""
    try:
        return datetime.fromisoformat(raw_ts.decode("ascii")).timestamp()
    except (UnicodeDecodeError, ValueError):
        return None


def _iter_log_payloads(log_path: Path):
    """Yield timestamp/payload pairs from a passthrough log file."""
    with log_path.open("rb") as handle:
        for raw_line in handle:
            line = raw_line.rstrip(b"\r\n")
            if not line:
                continue
            parts = line.split(b" | ", 1)
            if len(parts) != 2:
                continue
            ts_s = _parse_log_timestamp(parts[0])
            payload = _decode_logged_payload(parts[1])
            if payload:
                yield ts_s, payload


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay a GPS passthrough log to the Pixhawk GPS port.",
    )
    parser.add_argument(
        "--txt-file",
        default=str(DEFAULT_TXT_FILE),
        help="Path to the passthrough TXT log file.",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=DEFAULT_FREQUENCY_HZ,
        help="Replay frequency in Hz.",
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help="Pixhawk GPS serial port.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help="Pixhawk GPS serial baud rate.",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        default=DEFAULT_LOOP,
        help="Loop the file continuously.",
    )
    parser.add_argument(
        "--no-loop",
        action="store_false",
        dest="loop",
        help="Run through the file once and stop.",
    )
    parser.add_argument(
        "--use-log-timing",
        action="store_true",
        default=DEFAULT_USE_LOG_TIMING,
        help="Replay with the original timing from the log timestamps.",
    )
    parser.add_argument(
        "--fixed-rate",
        action="store_false",
        dest="use_log_timing",
        help="Ignore log timestamps and send one chunk per frequency interval.",
    )
    args = parser.parse_args()

    log_path = Path(args.txt_file)
    if not log_path.exists():
        raise FileNotFoundError(f"Log file not found: {log_path}")

    frequency_hz = max(0.1, float(args.frequency))
    interval_s = 1.0 / frequency_hz

    print(
        f"Replaying {log_path} -> {args.port} @ {args.baud} baud "
        f"mode={'log-timing' if args.use_log_timing else f'{frequency_hz:.2f}Hz-fixed'}"
    )

    with serial.Serial(args.port, args.baud, timeout=0) as ser:
        while True:
            chunk_count = 0
            byte_count = 0
            previous_ts_s = None
            for ts_s, payload in _iter_log_payloads(log_path):
                started = time.monotonic()
                if args.use_log_timing and ts_s is not None and previous_ts_s is not None:
                    delay_s = max(0.0, ts_s - previous_ts_s)
                    if delay_s > 0:
                        time.sleep(delay_s)
                ser.write(payload)
                chunk_count += 1
                byte_count += len(payload)
                print(f"[chunk {chunk_count:04d}] {len(payload)} bytes")
                print(_format_payload_for_terminal(payload))
                previous_ts_s = ts_s
                if not args.use_log_timing:
                    elapsed = time.monotonic() - started
                    sleep_s = interval_s - elapsed
                    if sleep_s > 0:
                        time.sleep(sleep_s)
            print(f"Replay pass complete: {chunk_count} chunks / {byte_count} bytes")
            if not args.loop:
                break


if __name__ == "__main__":
    main()
