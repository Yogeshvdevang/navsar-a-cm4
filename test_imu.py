#!/usr/bin/env python3
"""Standalone IMU smoke test."""

import argparse
import sys
import time


def build_parser():
    parser = argparse.ArgumentParser(description="Read IMU samples at 20 Hz.")
    parser.add_argument("--chip", default="MPU6050", choices=["MPU6050", "ICM42688P"])
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x68)
    parser.add_argument("--samples", type=int, default=50)
    return parser


def main():
    args = build_parser().parse_args()
    sys.path.insert(0, "src")
    from navisar.sensors.imu_driver import ImuDriver

    driver = ImuDriver(args.bus, args.address, args.chip)
    driver.init()
    print(
        f"Reading {args.samples} IMU samples from {args.chip} "
        f"bus={args.bus} addr=0x{args.address:02X}"
    )
    try:
        for idx in range(args.samples):
            sample = driver.read()
            print(
                f"[{idx:02d}] "
                f"gyro=({sample.gyro_x:+.4f}, {sample.gyro_y:+.4f}, {sample.gyro_z:+.4f}) rad/s "
                f"accel=({sample.accel_x:+.4f}, {sample.accel_y:+.4f}, {sample.accel_z:+.4f}) m/s^2 "
                f"roll={sample.roll_rad:+.4f} pitch={sample.pitch_rad:+.4f}"
            )
            time.sleep(0.05)
    finally:
        driver.close()


if __name__ == "__main__":
    main()
