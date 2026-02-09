#!/usr/bin/env python3
"""Compass calibration tool (hard-iron + simple soft-iron) for NAVISAR.

Usage example:
  python3 tools/compass_calibration.py --samples 1500 --rate-hz 50

Rotate the sensor slowly through all axes while collecting samples.
The script prints offsets/scales and a YAML snippet to paste into
config/pixhawk.yaml under compass.calibration.
"""

import argparse
import os
import sys
import time

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_DIR = os.path.join(ROOT_DIR, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from navisar.sensors.compass import CompassReader


def compute_calibration(samples):
    xs = [v[0] for v in samples]
    ys = [v[1] for v in samples]
    zs = [v[2] for v in samples]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)

    off_x = (max_x + min_x) / 2.0
    off_y = (max_y + min_y) / 2.0
    off_z = (max_z + min_z) / 2.0

    range_x = (max_x - min_x) / 2.0
    range_y = (max_y - min_y) / 2.0
    range_z = (max_z - min_z) / 2.0

    avg_range = (range_x + range_y + range_z) / 3.0
    # Scale factors to normalize each axis to the average range.
    scale_x = 1.0 if range_x == 0 else avg_range / range_x
    scale_y = 1.0 if range_y == 0 else avg_range / range_y
    scale_z = 1.0 if range_z == 0 else avg_range / range_z

    return {
        "offsets_mg": [off_x, off_y, off_z],
        "scales": [scale_x, scale_y, scale_z],
        "min_mg": [min_x, min_y, min_z],
        "max_mg": [max_x, max_y, max_z],
    }


def main():
    parser = argparse.ArgumentParser(description="Compass calibration (HMC5883L/QMC5883L)")
    parser.add_argument("--bus", type=int, default=1, help="Preferred I2C bus index")
    parser.add_argument("--samples", type=int, default=1200, help="Number of samples to collect")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Sample rate (Hz)")
    args = parser.parse_args()

    reader = CompassReader(preferred_bus=args.bus)
    interval = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.02

    print("Collecting samples. Rotate the sensor through all axes...")
    print(f"Using I2C bus {reader.bus_index}, addr=0x{reader.addr:02X}")

    samples = []
    start = time.time()
    for i in range(args.samples):
        _heading, (x, y, z) = reader.read_milligauss()
        samples.append((x, y, z))
        if (i + 1) % 100 == 0:
            print(f"Collected {i + 1}/{args.samples} samples")
        time.sleep(interval)

    reader.close()

    cal = compute_calibration(samples)
    duration = time.time() - start
    print(f"\nDone. {args.samples} samples in {duration:.1f}s")
    print("\nCalibration results (milligauss):")
    print(f"  offsets_mg: {cal['offsets_mg']}")
    print(f"  scales:     {cal['scales']}")

    print("\nPaste this into config/pixhawk.yaml:")
    print("compass:")
    print("  calibration:")
    print(f"    offsets_mg: [{cal['offsets_mg'][0]:.2f}, {cal['offsets_mg'][1]:.2f}, {cal['offsets_mg'][2]:.2f}]")
    print(f"    scales:     [{cal['scales'][0]:.5f}, {cal['scales'][1]:.5f}, {cal['scales'][2]:.5f}]")
    print("    axis_map:   ['+x', '+y', '+z']  # adjust if your compass axes are rotated")


if __name__ == "__main__":
    main()
