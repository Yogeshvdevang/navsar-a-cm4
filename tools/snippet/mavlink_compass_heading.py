#!/usr/bin/env python3
"""Read Pixhawk ATTITUDE yaw over MAVLink and print GUI-matched compass heading."""

import argparse
import time
from pathlib import Path

from pymavlink import mavutil


def _repo_root():
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "src" / "navisar").exists():
            return parent
    return current.parent


_ROOT = _repo_root()
ATTITUDE_MESSAGE_TYPES = ["ATTITUDE"]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read Pixhawk ATTITUDE yaw over MAVLink and print GUI compass heading."
    )
    parser.add_argument(
        "--device",
        default="/dev/ttyACM0",
        help="MAVLink serial device connected to the flight controller.",
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
        help="Requested MAVLink message rate for ATTITUDE messages.",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=10.0,
        help="Terminal print rate.",
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for the first MAVLink heartbeat.",
    )
    parser.add_argument(
        "--show-mag",
        action="store_true",
        help="Also print raw ATTITUDE yaw radians from the MAVLink message.",
    )
    parser.add_argument(
        "--heading-smoothing-alpha",
        type=float,
        default=0.18,
        help="GUI wrap-aware smoothing alpha used after each yaw sample.",
    )
    parser.add_argument(
        "--heading-max-delta-deg",
        type=float,
        default=10.0,
        help="GUI maximum per-sample heading step allowed by smoothing.",
    )
    return parser.parse_args()


def request_message_interval(master, msg_id, rate_hz):
    if rate_hz <= 0:
        return
    interval_us = int(1_000_000 / rate_hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def request_attitude_stream(master, rate_hz):
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, rate_hz)


def wrap_heading(deg):
    if deg is None:
        return None
    return (float(deg) % 360.0 + 360.0) % 360.0


def angle_delta_deg(new_deg, old_deg):
    return (new_deg - old_deg + 540.0) % 360.0 - 180.0


def smooth_heading_deg(prev_deg, new_deg, alpha, max_delta_deg=None):
    if prev_deg is None:
        return wrap_heading(new_deg)
    if new_deg is None:
        return prev_deg
    delta = angle_delta_deg(new_deg, prev_deg)
    if max_delta_deg is not None:
        limit = abs(float(max_delta_deg))
        delta = max(-limit, min(limit, delta))
    alpha = float(alpha)
    return wrap_heading(prev_deg + alpha * delta)


def main():
    args = parse_args()

    print(f"Connecting to {args.device} @ {args.baud}...")
    master = mavutil.mavlink_connection(args.device, baud=args.baud)

    try:
        master.wait_heartbeat(timeout=args.heartbeat_timeout)
    except Exception as exc:
        raise RuntimeError(
            f"Failed to receive MAVLink heartbeat on {args.device}"
        ) from exc

    print("Heartbeat received. Requesting ATTITUDE messages...")
    request_attitude_stream(master, args.rate_hz)

    print_interval_s = 1.0 / max(args.print_hz, 1e-3)
    last_print = 0.0
    waiting_since = time.time()
    last_heading = None

    while True:
        msg = master.recv_match(type=ATTITUDE_MESSAGE_TYPES, blocking=True, timeout=1.0)
        now = time.time()

        if msg is None:
            if now - waiting_since >= 2.0:
                print("Waiting for MAVLink ATTITUDE data...")
                waiting_since = now
            continue

        if now - last_print < print_interval_s:
            continue
        last_print = now

        yaw_rad = getattr(msg, "yaw", None)
        if yaw_rad is None:
            continue
        heading_deg = wrap_heading(float(yaw_rad) * 180.0 / 3.141592653589793)
        if args.heading_smoothing_alpha is not None:
            heading_deg = smooth_heading_deg(
                last_heading,
                heading_deg,
                args.heading_smoothing_alpha,
                args.heading_max_delta_deg,
            )
        last_heading = heading_deg

        if args.show_mag:
            print(
                f"heading={heading_deg:7.2f} deg  "
                f"src={msg.get_type():<11}  "
                f"yaw_rad={float(yaw_rad):8.4f}"
            )
        else:
            print(f"{heading_deg:.2f}")


if __name__ == "__main__":
    main()
