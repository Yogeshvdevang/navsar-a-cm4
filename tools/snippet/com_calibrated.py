#!/usr/bin/env python3
"""
Compass reader with hard/soft-iron calibration support.

Usage:
  1) Collect calibration:
     python com_calibrated.py --calibrate --duration 60 --out compass_calibration.json
  2) Run with calibration:
     python com_calibrated.py --calibration compass_calibration.json
"""

import argparse
import json
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


I2C_BUS = 1
COMPASS_ADDR = 0x0C
HERTZ = 15
HEADING_WINDOW = 7


@dataclass
class Calibration:
    offsets: tuple[float, float, float]
    scales: tuple[float, float, float]

    def apply(self, x: int, y: int, z: int) -> tuple[float, float, float]:
        cx = (x - self.offsets[0]) * self.scales[0]
        cy = (y - self.offsets[1]) * self.scales[1]
        cz = (z - self.offsets[2]) * self.scales[2]
        return cx, cy, cz


def init_compass(bus: SMBus) -> None:
    bus.read_byte(COMPASS_ADDR)


def _is_bad_frame(x: int, y: int, z: int) -> bool:
    if (x == -1 and y == -1) or (x == -1 and z == -1) or (y == -1 and z == -1):
        return True
    if abs(x) >= 32760 or abs(y) >= 32760 or abs(z) >= 32760:
        return True
    return False


def read_raw_xyz(bus: SMBus, retries: int = 3) -> tuple[int, int, int]:
    last_error = None
    for _ in range(retries):
        try:
            bus.write_byte_data(COMPASS_ADDR, 0x0A, 0x01)
            for _ in range(10):
                st1 = bus.read_byte_data(COMPASS_ADDR, 0x02)
                if st1 & 0x01:
                    break
                time.sleep(0.003)
            else:
                raise RuntimeError("Compass data-ready timeout")

            data = bus.read_i2c_block_data(COMPASS_ADDR, 0x03, 7)
            x = (data[1] << 8) | data[0]
            y = (data[3] << 8) | data[2]
            z = (data[5] << 8) | data[4]
            x = x - 65536 if x > 32767 else x
            y = y - 65536 if y > 32767 else y
            z = z - 65536 if z > 32767 else z

            st2 = data[6]
            if st2 & 0x08:
                raise RuntimeError("Compass overflow (ST2)")
            if _is_bad_frame(x, y, z):
                raise RuntimeError(f"Rejected corrupt sample x={x} y={y} z={z}")
            return x, y, z
        except (OSError, RuntimeError) as err:
            last_error = err
            time.sleep(0.02)
    raise RuntimeError(f"I2C read failed after {retries} retries: {last_error}")


def circular_mean_deg(values: deque[float]) -> float:
    if not values:
        return 0.0
    sin_sum = sum(math.sin(math.radians(v)) for v in values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values)
    angle = math.degrees(math.atan2(sin_sum, cos_sum))
    return (angle + 360.0) % 360.0


def compute_calibration(samples: list[tuple[int, int, int]]) -> Calibration:
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

    return Calibration(
        offsets=(x_off, y_off, z_off),
        scales=(x_scale, y_scale, z_scale),
    )


def save_calibration(path: Path, cal: Calibration) -> None:
    payload = {
        "offsets": {"x": cal.offsets[0], "y": cal.offsets[1], "z": cal.offsets[2]},
        "scales": {"x": cal.scales[0], "y": cal.scales[1], "z": cal.scales[2]},
        "meta": {
            "address": f"0x{COMPASS_ADDR:02X}",
            "bus": I2C_BUS,
            "method": "minmax_hard_soft_iron",
            "created_unix": time.time(),
        },
    }
    path.write_text(json.dumps(payload, indent=2))


def load_calibration(path: Path) -> Calibration:
    data = json.loads(path.read_text())
    offsets = data["offsets"]
    scales = data["scales"]
    return Calibration(
        offsets=(float(offsets["x"]), float(offsets["y"]), float(offsets["z"])),
        scales=(float(scales["x"]), float(scales["y"]), float(scales["z"])),
    )


def heading_from_xyz(
    x: int, y: int, z: int, calibration: Calibration | None = None
) -> tuple[float, tuple[float, float, float]]:
    if calibration is None:
        cx, cy, cz = float(x), float(y), float(z)
    else:
        cx, cy, cz = calibration.apply(x, y, z)
    heading = math.degrees(math.atan2(cy, cx))
    heading = (heading + 360.0) % 360.0
    return heading, (cx, cy, cz)


def run_calibration(bus: SMBus, duration: int, out_file: Path) -> None:
    print(
        "Calibration mode: rotate sensor slowly through all orientations "
        "(360 deg yaw, pitch, roll)."
    )
    print(f"Collecting for {duration} seconds...")
    samples: list[tuple[int, int, int]] = []
    start = time.time()
    next_print = start
    while time.time() - start < duration:
        try:
            xyz = read_raw_xyz(bus)
            samples.append(xyz)
        except RuntimeError as err:
            print(f"I2C warning: {err}")
        now = time.time()
        if now >= next_print:
            elapsed = int(now - start)
            print(f"  {elapsed:3d}s | samples={len(samples)}")
            next_print = now + 1.0
        time.sleep(1.0 / HERTZ)

    cal = compute_calibration(samples)
    save_calibration(out_file, cal)
    print("\nCalibration saved.")
    print(f"File: {out_file}")
    print(
        "Offsets: "
        f"x={cal.offsets[0]:.3f} y={cal.offsets[1]:.3f} z={cal.offsets[2]:.3f}"
    )
    print(
        "Scales:  "
        f"x={cal.scales[0]:.6f} y={cal.scales[1]:.6f} z={cal.scales[2]:.6f}"
    )


def run_heading(bus: SMBus, calibration: Calibration | None) -> None:
    heading_window: deque[float] = deque(maxlen=HEADING_WINDOW)
    while True:
        try:
            x, y, z = read_raw_xyz(bus)
            heading, (cx, cy, cz) = heading_from_xyz(x, y, z, calibration=calibration)
            heading_window.append(heading)
            smooth = circular_mean_deg(heading_window)
            if calibration is None:
                print(f"Heading: {smooth:6.2f} deg | Raw: x={x} y={y} z={z}")
            else:
                print(
                    f"Heading: {smooth:6.2f} deg | Raw: x={x} y={y} z={z} "
                    f"| Cal: x={cx:7.2f} y={cy:7.2f} z={cz:7.2f}"
                )
        except RuntimeError as err:
            print(f"I2C warning: {err}")
        time.sleep(1.0 / HERTZ)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Collect samples and write calibration file.",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=60,
        help="Calibration capture duration in seconds.",
    )
    parser.add_argument(
        "--out",
        default="compass_calibration.json",
        help="Where to write new calibration JSON in --calibrate mode.",
    )
    parser.add_argument(
        "--calibration",
        default="compass_calibration.json",
        help="Calibration JSON to apply in normal heading mode.",
    )
    parser.add_argument(
        "--no-calibration",
        action="store_true",
        help="Ignore calibration file and run raw heading only.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bus = SMBus(I2C_BUS)
    print(f"Using I2C bus {I2C_BUS}, address 0x{COMPASS_ADDR:02X}")
    calibration = None
    calibration_path = Path(args.calibration)

    try:
        init_compass(bus)
        print("Compass detected.\n")
        if args.calibrate:
            run_calibration(bus, duration=args.duration, out_file=Path(args.out))
            return

        if not args.no_calibration and calibration_path.exists():
            calibration = load_calibration(calibration_path)
            print(f"Loaded calibration from {calibration_path}\n")
        elif args.no_calibration:
            print("Running without calibration (--no-calibration).\n")
        else:
            print(
                f"No calibration file at {calibration_path}. "
                "Running raw heading only.\n"
            )

        run_heading(bus, calibration=calibration)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
