#!/usr/bin/env python3
"""Read the Pixhawk built-in compass over MAVLink and print heading + raw mag."""

import argparse
import math
import time

from pymavlink import mavutil


def parse_args():
    parser = argparse.ArgumentParser(
        description="Monitor Pixhawk built-in compass data over MAVLink."
    )
    parser.add_argument(
        "--device",
        default="/dev/ttyACM1",
        help="MAVLink serial device connected to the Pixhawk.",
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
        help="Requested message rate for magnetometer-bearing MAVLink messages.",
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
        help="Seconds to wait for the first autopilot heartbeat.",
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


def get_message_time_s(msg):
    time_usec = getattr(msg, "time_usec", 0) or 0
    if time_usec:
        return float(time_usec) * 1e-6
    time_boot_ms = getattr(msg, "time_boot_ms", 0) or 0
    if time_boot_ms:
        return float(time_boot_ms) * 1e-3
    return time.time()


def get_compass_instance(msg_type):
    mapping = {
        "RAW_IMU": 0,
        "SCALED_IMU": 0,
        "HIGHRES_IMU": 0,
        "SCALED_IMU2": 1,
        "SCALED_IMU3": 2,
    }
    return mapping.get(msg_type, -1)


def heading_from_mag(x_mg, y_mg):
    if abs(x_mg) < 1e-9 and abs(y_mg) < 1e-9:
        return None
    return (math.degrees(math.atan2(y_mg, x_mg)) + 360.0) % 360.0


def main():
    args = parse_args()

    print(f"Connecting to MAVLink on {args.device} @ {args.baud}...")
    master = mavutil.mavlink_connection(args.device, baud=args.baud)
    try:
        master.wait_heartbeat(timeout=args.heartbeat_timeout)
    except Exception as exc:
        raise RuntimeError("Failed to receive MAVLink heartbeat") from exc

    print("Heartbeat received. Requesting compass-capable MAVLink streams...")
    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
        args.rate_hz,
    )
    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,
        args.rate_hz,
    )
    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
        args.rate_hz,
    )
    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3,
        args.rate_hz,
    )
    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
        args.rate_hz,
    )

    print_interval_s = 1.0 / max(args.print_hz, 1e-3)
    last_print = 0.0
    last_status = 0.0
    seen_messages = 0

    while True:
        msg = master.recv_match(
            type=["RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3", "HIGHRES_IMU"],
            blocking=True,
            timeout=1.0,
        )
        now = time.time()

        if msg is None:
            if now - last_status >= 2.0:
                last_status = now
                print("Waiting for compass data...")
            continue

        x_mg = float(getattr(msg, "xmag", 0.0))
        y_mg = float(getattr(msg, "ymag", 0.0))
        z_mg = float(getattr(msg, "zmag", 0.0))
        if x_mg == 0.0 and y_mg == 0.0 and z_mg == 0.0:
            continue

        seen_messages += 1
        if now - last_print < print_interval_s:
            continue
        last_print = now

        heading_deg = heading_from_mag(x_mg, y_mg)
        ts = get_message_time_s(msg)
        msg_type = msg.get_type()
        instance = get_compass_instance(msg_type)
        heading_text = "n/a" if heading_deg is None else f"{heading_deg:6.2f} deg"

        print(
            f"t={ts:12.3f} src={msg_type:<11} compass={instance} "
            f"heading={heading_text} mag[mG]=({x_mg:8.1f}, {y_mg:8.1f}, {z_mg:8.1f}) "
            f"packets={seen_messages}"
        )


if __name__ == "__main__":
    main()

"""
we have to change this logic what we want in the end we want the altitude same as barometer and how we calculating it by like this 
final_altitude = optical_flow_height +(barometer_height - 
"""