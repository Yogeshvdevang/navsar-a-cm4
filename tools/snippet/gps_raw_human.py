#!/usr/bin/env python3
"""Read raw GPS NMEA data and print a human-readable live summary."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

ROOT_DIR = Path(__file__).resolve().parents[2]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from navisar.sensors.gps_serial import find_gps_port_and_baud

DEFAULT_PORT = "/dev/ttyAMA5"
DEFAULT_BAUD = 230400
DEFAULT_PRINT_INTERVAL_S = 1.0
DEFAULT_PROBE_SECONDS = 3.0


def _nmea_to_decimal(value: str, hemi: str) -> float | None:
    if not value or not hemi:
        return None
    try:
        raw = float(value)
    except ValueError:
        return None
    hemi = hemi.upper()
    deg = int(raw / 100)
    minutes = raw - (deg * 100)
    dec = deg + minutes / 60.0
    if hemi in {"S", "W"}:
        dec = -dec
    return dec


def _safe_int(value: str | None) -> int | None:
    try:
        return int(value) if value not in (None, "") else None
    except (TypeError, ValueError):
        return None


def _safe_float(value: str | None) -> float | None:
    try:
        return float(value) if value not in (None, "") else None
    except (TypeError, ValueError):
        return None


def _parse_gga(fields: list[str]) -> dict[str, float | int | None] | None:
    if len(fields) < 10:
        return None
    lat = _nmea_to_decimal(fields[2], fields[3])
    lon = _nmea_to_decimal(fields[4], fields[5])
    if lat is None or lon is None:
        return None
    return {
        "lat": lat,
        "lon": lon,
        "alt_m": _safe_float(fields[9]),
        "sats": _safe_int(fields[7]),
    }


def _parse_rmc(fields: list[str]) -> dict[str, float | None] | None:
    if len(fields) < 9 or fields[2] != "A":
        return None
    lat = _nmea_to_decimal(fields[3], fields[4])
    lon = _nmea_to_decimal(fields[5], fields[6])
    if lat is None or lon is None:
        return None
    speed_knots = _safe_float(fields[7])
    heading_deg = _safe_float(fields[8])
    return {
        "lat": lat,
        "lon": lon,
        "speed_mps": None if speed_knots is None else speed_knots * 0.514444,
        "heading_deg": heading_deg,
    }


def _parse_nmea(line: str) -> dict[str, float | int | None] | None:
    if not line.startswith("$"):
        return None
    if "*" in line:
        line = line.split("*", 1)[0]
    fields = line.split(",")
    if not fields:
        return None
    msg = fields[0][3:] if len(fields[0]) >= 6 else fields[0]
    if msg.endswith("GGA"):
        return _parse_gga(fields)
    if msg.endswith("RMC"):
        return _parse_rmc(fields)
    return None


def _format_coord(value: float | None, pos: str, neg: str) -> str:
    if value is None:
        return "n/a"
    hemi = pos if value >= 0 else neg
    return f"{abs(value):.7f}° {hemi}"


def _format_alt(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.4f} m"


def _format_speed(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.2f} m/s"


def _format_heading(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.1f}°"


def _format_sats(value: int | None) -> str:
    return "n/a" if value is None else str(value)


def _print_fix(snapshot: dict[str, float | int | None]) -> None:
    print(f"LAT : {_format_coord(snapshot.get('lat'), 'N', 'S')}")
    print(f"LON : {_format_coord(snapshot.get('lon'), 'E', 'W')}")
    print(f"ALT : {_format_alt(snapshot.get('alt_m'))}")
    print(f"SPD : {_format_speed(snapshot.get('speed_mps'))}")
    print(f"HDG : {_format_heading(snapshot.get('heading_deg'))}")
    print(f"SATS: {_format_sats(snapshot.get('sats'))}")
    print("-" * 32)


def _lock_serial(port: str | None, baud: str | int | None, probe_seconds: float) -> tuple[str, int]:
    port_is_auto = port is None or str(port).lower() == "auto"
    baud_is_auto = baud is None or str(baud).lower() == "auto"
    if not port_is_auto and not baud_is_auto:
        return str(port), int(baud)
    choice = find_gps_port_and_baud(
        port=None if port_is_auto else str(port),
        bauds=None if baud_is_auto else [int(baud)],
        probe_seconds=probe_seconds,
        verbose=True,
    )
    if not choice:
        raise RuntimeError("No GPS NMEA stream detected.")
    return choice


def main() -> None:
    parser = argparse.ArgumentParser(description="Show raw GPS data in human-readable form.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port or 'auto'.")
    parser.add_argument("--baud", default=DEFAULT_BAUD, help="Baud rate or 'auto'.")
    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_PRINT_INTERVAL_S,
        help="Print interval in seconds.",
    )
    parser.add_argument(
        "--probe-seconds",
        type=float,
        default=DEFAULT_PROBE_SECONDS,
        help="Probe duration when port/baud is auto.",
    )
    args = parser.parse_args()

    port, baud = _lock_serial(args.port, args.baud, args.probe_seconds)
    print(f"Listening for GPS on {port} @ {baud}")

    snapshot: dict[str, float | int | None] = {
        "lat": None,
        "lon": None,
        "alt_m": None,
        "speed_mps": None,
        "heading_deg": None,
        "sats": None,
    }
    last_print = 0.0

    with serial.Serial(port, baud, timeout=0.2) as ser:
        while True:
            raw = ser.readline()
            if raw:
                line = raw.decode("ascii", errors="ignore").strip()
                parsed = _parse_nmea(line)
                if parsed is not None:
                    snapshot.update({k: v for k, v in parsed.items() if v is not None})
            now = time.time()
            if now - last_print >= max(0.1, float(args.interval)):
                _print_fix(snapshot)
                last_print = now


if __name__ == "__main__":
    main()
