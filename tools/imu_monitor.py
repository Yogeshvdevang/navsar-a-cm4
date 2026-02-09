#!/usr/bin/env python3
"""Print Pixhawk IMU data to the terminal."""

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil


def _parse_args():
    parser = argparse.ArgumentParser(description="Monitor Pixhawk IMU data.")
    parser.add_argument(
        "--device",
        type=str,
        default="/dev/ttyACM0",
        help="MAVLink serial device.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="MAVLink baud rate.",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=20.0,
        help="Requested HIGHRES_IMU message rate.",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=10.0,
        help="Terminal print rate.",
    )
    return parser.parse_args()


def _ensure_import_path():
    repo_root = Path(__file__).resolve().parents[1]
    src_path = repo_root / "src"
    if str(src_path) not in sys.path:
        sys.path.insert(0, str(src_path))


def main():
    args = _parse_args()
    _ensure_import_path()

    from navisar.pixhawk.mavlink_client import MavlinkInterface

    mav = MavlinkInterface(args.device, baud=args.baud)
    mav.request_message_interval(
        mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
        rate_hz=args.rate_hz,
    )
    mav.request_message_interval(
        mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
        rate_hz=args.rate_hz,
    )
    mav.request_message_interval(
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,
        rate_hz=args.rate_hz,
    )

    last_print = 0.0
    last_status = 0.0
    print_interval = 1.0 / max(args.print_hz, 1e-3)
    while True:
        imu = mav.recv_imu()
        now = time.time()
        if imu and (now - last_print) >= print_interval:
            last_print = now
            print(
                "accel[m/s^2]=({ax:.3f},{ay:.3f},{az:.3f}) "
                "gyro[rad/s]=({gx:.3f},{gy:.3f},{gz:.3f}) "
                "t={time_s:.3f}".format(**imu)
            )
        if imu is None:
            msg = mav.master.recv_match(type="RAW_IMU", blocking=False)
            if msg and (now - last_print) >= print_interval:
                last_print = now
                print(
                    "RAW_IMU accel[mg]=({},{},{}) gyro[mrad/s]=({},{},{}) t={}".format(
                        msg.xacc,
                        msg.yacc,
                        msg.zacc,
                        msg.xgyro,
                        msg.ygyro,
                        msg.zgyro,
                        msg.time_usec,
                    )
                )
        if imu is None:
            msg = mav.master.recv_match(type="SCALED_IMU", blocking=False)
            if msg and (now - last_print) >= print_interval:
                last_print = now
                print(
                    "SCALED_IMU accel[mg]=({},{},{}) gyro[mrad/s]=({},{},{}) t={}".format(
                        msg.xacc,
                        msg.yacc,
                        msg.zacc,
                        msg.xgyro,
                        msg.ygyro,
                        msg.zgyro,
                        msg.time_boot_ms,
                    )
                )
        if (now - last_status) >= 2.0 and imu is None:
            last_status = now
            print("Waiting for IMU data...")
        time.sleep(0.01)


if __name__ == "__main__":
    main()
