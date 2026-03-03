#!/usr/bin/env python3
"""Collect compass samples and write calibration JSON for NAVISAR.

Usage:
  python3 tools/compass_file_calibration.py --duration 60 --out tools/compass_calibration.json
"""

import argparse
import json
import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from navisar.sensors.compass import CompassReader


def compute_calibration(samples):
    if len(samples) < 50:
        raise ValueError("Not enough samples for calibration. Collect at least 50.")

    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    zs = [s[2] for s in samples]

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    z_min, z_max = min(zs), max(zs)

    x_off = (x_max + x_min) / 2.0
    y_off = (y_max + y_min) / 2.0
    z_off = (z_max + z_min) / 2.0

    x_rad = max((x_max - x_min) / 2.0, 1e-6)
    y_rad = max((y_max - y_min) / 2.0, 1e-6)
    z_rad = max((z_max - z_min) / 2.0, 1e-6)

    avg_rad = (x_rad + y_rad + z_rad) / 3.0
    x_scale = avg_rad / x_rad
    y_scale = avg_rad / y_rad
    z_scale = avg_rad / z_rad

    return {
        "offsets": {"x": x_off, "y": y_off, "z": z_off},
        "offsets_mg": [x_off, y_off, z_off],
        "scales": {"x": x_scale, "y": y_scale, "z": z_scale},
    }


def main():
    parser = argparse.ArgumentParser(description="Compass calibration JSON writer")
    parser.add_argument("--bus", type=int, default=1, help="Preferred I2C bus index")
    parser.add_argument("--duration", type=int, default=60, help="Capture duration in seconds")
    parser.add_argument("--rate-hz", type=float, default=15.0, help="Sample rate in Hz")
    parser.add_argument(
        "--out",
        default=str(ROOT_DIR / "tools" / "compass_calibration.json"),
        help="Output JSON path",
    )
    args = parser.parse_args()

    out_path = Path(args.out).expanduser()
    if not out_path.is_absolute():
        out_path = (ROOT_DIR / out_path).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    reader = CompassReader(preferred_bus=args.bus)
    interval = 1.0 / args.rate_hz if args.rate_hz > 0 else 1.0 / 15.0
    samples = []

    print("Calibration mode: rotate sensor through all orientations.")
    print(f"Compass: bus {reader.bus_index}, addr=0x{reader.addr:02X}")
    print(f"Collecting for {args.duration}s at {args.rate_hz:.1f} Hz...")

    start = time.time()
    next_print = start
    try:
        while time.time() - start < args.duration:
            try:
                _heading, (x, y, z) = reader.read()
                samples.append((x, y, z))
            except Exception as exc:
                print(f"I2C warning: {exc}")
            now = time.time()
            if now >= next_print:
                print(f"  {int(now - start):3d}s | samples={len(samples)}")
                next_print = now + 1.0
            time.sleep(interval)
    finally:
        reader.close()

    cal = compute_calibration(samples)
    payload = {
        **cal,
        "meta": {
            "address": f"0x{reader.addr:02X}",
            "bus": reader.bus_index,
            "method": "minmax_hard_soft_iron",
            "created_unix": time.time(),
        },
    }
    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print("\nCalibration saved.")
    print(f"File: {out_path}")
    print(
        "Offsets: "
        f"x={cal['offsets']['x']:.3f} y={cal['offsets']['y']:.3f} z={cal['offsets']['z']:.3f}"
    )
    print(
        "Scales:  "
        f"x={cal['scales']['x']:.6f} y={cal['scales']['y']:.6f} z={cal['scales']['z']:.6f}"
    )
    print("\nSet this in config/pixhawk.yaml:")
    print("compass:")
    print(f"  calibration_file: {out_path}")


if __name__ == "__main__":
    main()
