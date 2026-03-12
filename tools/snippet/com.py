#!/usr/bin/env python3
"""
Read compass heading from I2C address 0x0C on bus 1.
"""

import math
import time
from collections import deque

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


I2C_BUS = 1
COMPASS_ADDR = 0x1E  #0x0C
HERTZ = 10
HEADING_WINDOW = 7


def init_compass(bus: SMBus) -> None:
    # Probe device at 0x0C.
    bus.read_byte(COMPASS_ADDR)


def _is_bad_frame(x: int, y: int, z: int) -> bool:
    # Corrupt I2C frames often show one value with two 0xFFFF axes.
    if (x == -1 and y == -1) or (x == -1 and z == -1) or (y == -1 and z == -1):
        return True
    # Reject clearly impossible saturation values.
    if abs(x) >= 32760 or abs(y) >= 32760 or abs(z) >= 32760:
        return True
    return False


def read_compass(bus: SMBus, retries: int = 3) -> tuple[float, tuple[int, int, int]]:
    # Trigger single measurement and read XYZ (little-endian pairs).
    # Register map follows AK8963-style layout at 0x0C.
    last_error = None
    for _ in range(retries):
        try:
            # Single measurement mode.
            bus.write_byte_data(COMPASS_ADDR, 0x0A, 0x01)
            # Wait for data-ready (ST1 bit0).
            for _ in range(10):
                st1 = bus.read_byte_data(COMPASS_ADDR, 0x02)
                if st1 & 0x01:
                    break
                time.sleep(0.003)
            else:
                raise RuntimeError("Compass data-ready timeout")

            # Read XYZ + ST2 for overflow check.
            data = bus.read_i2c_block_data(COMPASS_ADDR, 0x03, 7)
            break
        except (OSError, RuntimeError) as err:
            last_error = err
            time.sleep(0.02)
    else:
        raise RuntimeError(f"I2C read failed after {retries} retries: {last_error}")

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

    heading = math.degrees(math.atan2(y, x))
    heading = (heading + 360.0) % 360.0
    return heading, (x, y, z)

def circular_mean_deg(values: deque[float]) -> float:
    if not values:
        return 0.0
    sin_sum = sum(math.sin(math.radians(v)) for v in values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values)
    angle = math.degrees(math.atan2(sin_sum, cos_sum))
    return (angle + 360.0) % 360.0


def main() -> None:
    bus = SMBus(I2C_BUS)
    print(f"Using I2C bus {I2C_BUS}, address 0x{COMPASS_ADDR:02X}")
    interval = 1.0 / HERTZ
    heading_window: deque[float] = deque(maxlen=HEADING_WINDOW)

    try:
        init_compass(bus)
        print("Compass detected.\n")

        while True:
            try:
                heading, (x, y, z) = read_compass(bus)
                heading_window.append(heading)
                smooth = circular_mean_deg(heading_window)
                print(f"Heading: {smooth:6.2f} deg | Raw: x={x} y={y} z={z}")
            except RuntimeError as err:
                print(f"I2C warning: {err}")
            time.sleep(interval)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
