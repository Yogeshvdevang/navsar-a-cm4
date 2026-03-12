#!/usr/bin/env python3
"""Standalone compass monitor with configurable print frequency."""

from __future__ import annotations

import argparse
import glob
import math
import time

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


DEFAULT_I2C_BUS = 1
HMC5883L_ADDR = 0x1E
QMC5883L_ADDR = 0x0D
RUN_HZ = 50
RUN_RECOVERY_ENABLED = True
RUN_REINIT_ERROR_THRESHOLD = 5
RUN_ERROR_RETRY_DELAY_S = 0.05
RUN_REOPEN_DELAY_S = 0.4


def get_i2c_bus_index(preferred: int) -> int | None:
    if glob.glob(f"/dev/i2c-{preferred}"):
        return preferred
    paths = sorted(glob.glob("/dev/i2c-*"))
    if not paths:
        return None
    try:
        return int(paths[0].split("-")[-1])
    except ValueError:
        return None


def detect_compass(bus: SMBus) -> int | None:
    for addr in (HMC5883L_ADDR, QMC5883L_ADDR):
        try:
            bus.read_byte(addr)
            return addr
        except OSError:
            continue
    return None


def init_compass(bus: SMBus, addr: int) -> None:
    if addr == HMC5883L_ADDR:
        # 8-sample average, 15 Hz, normal measurement
        bus.write_byte_data(addr, 0x00, 0x70)
        # Gain = 1090 LSB/Gauss
        bus.write_byte_data(addr, 0x01, 0x20)
        # Continuous measurement mode
        bus.write_byte_data(addr, 0x02, 0x00)
        return
    if addr == QMC5883L_ADDR:
        # 200 Hz, 8G, continuous
        bus.write_byte_data(addr, 0x0B, 0x01)
        bus.write_byte_data(addr, 0x09, 0x1D)
        return
    raise ValueError("Unsupported compass address")


def read_compass(bus: SMBus, addr: int) -> tuple[float, tuple[int, int, int]]:
    if addr == HMC5883L_ADDR:
        data = bus.read_i2c_block_data(addr, 0x03, 6)
        x = (data[0] << 8) | data[1]
        z = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]
    elif addr == QMC5883L_ADDR:
        data = bus.read_i2c_block_data(addr, 0x00, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
    else:
        raise ValueError("Unsupported compass address")

    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z

    heading = math.degrees(math.atan2(y, x))
    heading = (heading + 360.0) % 360.0
    return heading, (x, y, z)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone compass monitor")
    parser.add_argument(
        "--hz",
        type=float,
        default=RUN_HZ,
        help=f"Print frequency in Hz (default: {RUN_HZ})",
    )
    parser.add_argument(
        "--i2c-bus",
        type=int,
        default=DEFAULT_I2C_BUS,
        help=f"Preferred I2C bus index (default: {DEFAULT_I2C_BUS})",
    )
    parser.add_argument(
        "--show-raw",
        action="store_true",
        help="Also print raw X,Y,Z values",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    hz = max(0.1, float(args.hz))
    sleep_s = 1.0 / hz

    bus_index = get_i2c_bus_index(int(args.i2c_bus))
    if bus_index is None:
        raise SystemExit("No I2C bus found under /dev/i2c-*")

    recovery_enabled = bool(RUN_RECOVERY_ENABLED)
    reinit_threshold = max(1, int(RUN_REINIT_ERROR_THRESHOLD))
    retry_delay_s = max(0.0, float(RUN_ERROR_RETRY_DELAY_S))
    reopen_delay_s = max(0.0, float(RUN_REOPEN_DELAY_S))

    bus = None
    compass_addr = None
    consecutive_errors = 0

    def reopen_compass() -> tuple[SMBus, int]:
        b = SMBus(bus_index)
        addr = detect_compass(b)
        if addr is None:
            b.close()
            raise RuntimeError(
                f"Compass not found on I2C bus {bus_index} (tried 0x{HMC5883L_ADDR:02X}, 0x{QMC5883L_ADDR:02X})"
            )
        init_compass(b, addr)
        return b, addr

    try:
        bus, compass_addr = reopen_compass()

        print(
            f"Compass monitor started on /dev/i2c-{bus_index}, addr=0x{compass_addr:02X}, hz={hz:.2f}"
        )
        print(
            "Recovery config: "
            f"enabled={recovery_enabled} "
            f"threshold={reinit_threshold} "
            f"retry_delay={retry_delay_s:.2f}s "
            f"reopen_delay={reopen_delay_s:.2f}s"
        )
        print("Press Ctrl+C to stop.\n")

        while True:
            ts = time.strftime("%H:%M:%S")
            try:
                heading, (x, y, z) = read_compass(bus, compass_addr)
                consecutive_errors = 0
                if args.show_raw:
                    print(f"[{ts}] heading={heading:7.2f} deg raw=({x:6d},{y:6d},{z:6d})")
                else:
                    print(f"[{ts}] heading={heading:7.2f} deg")
            except OSError as exc:
                consecutive_errors += 1
                print(f"[{ts}] compass read error: {exc}")
                if recovery_enabled and consecutive_errors >= reinit_threshold:
                    print(f"[{ts}] attempting compass reinit (consecutive_errors={consecutive_errors})")
                    try:
                        if bus is not None:
                            bus.close()
                    except Exception:
                        pass
                    time.sleep(reopen_delay_s)
                    try:
                        bus, compass_addr = reopen_compass()
                        consecutive_errors = 0
                        print(f"[{ts}] compass reinit successful at 0x{compass_addr:02X}")
                    except Exception as reinit_exc:
                        print(f"[{ts}] compass reinit failed: {reinit_exc}")
                time.sleep(retry_delay_s)
            time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("\nStopped compass monitor.")
    finally:
        if bus is not None:
            bus.close()


if __name__ == "__main__":
    main()
