import os
import time

from pymavlink import mavutil


DEVICE = os.getenv("MAVLINK_DEVICE", "/dev/ttyACM0")
BAUD = int(os.getenv("MAVLINK_BAUD", "115200"))
HEARTBEAT_TIMEOUT_S = float(os.getenv("MAVLINK_HEARTBEAT_TIMEOUT_S", "5.0"))
PRINT_INTERVAL_S = float(os.getenv("MAVLINK_PRINT_INTERVAL_S", "0.0"))
GPS_RAW_RATE_HZ = float(os.getenv("MAVLINK_GPS_RAW_RATE_HZ", "2.0"))
GLOBAL_POS_RATE_HZ = float(os.getenv("MAVLINK_GLOBAL_POS_RATE_HZ", "2.0"))
LIDAR_RATE_HZ = float(os.getenv("MAVLINK_LIDAR_RATE_HZ", "5.0"))


def _request_message_interval(master, msg_id, rate_hz):
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


def main():
    print(f"Connecting to Pixhawk on {DEVICE} @ {BAUD}...")
    master = mavutil.mavlink_connection(DEVICE, baud=BAUD)
    try:
        master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
    except Exception as exc:
        raise RuntimeError("Failed to receive MAVLink heartbeat") from exc

    print("Heartbeat received. Streaming GPS + LiDAR MAVLink messages...")
    _request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, GPS_RAW_RATE_HZ)
    _request_message_interval(
        master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, GLOBAL_POS_RATE_HZ
    )
    _request_message_interval(
        master, mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, LIDAR_RATE_HZ
    )
    last_print = 0.0
    while True:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        msg_type = msg.get_type()
        if msg_type not in ("GPS_RAW_INT", "GLOBAL_POSITION_INT", "DISTANCE_SENSOR"):
            continue
        now = time.time()
        if PRINT_INTERVAL_S <= 0.0 or (now - last_print) >= PRINT_INTERVAL_S:
            print("+++++++++++++++++++++++")
            print(msg)
            print("+++++++++++++++++++++")
            msg_dict = msg.to_dict()
            print(f"RAW[{msg_type}]: {msg}")
            print(f"FORMAT[{msg_type}]: {list(msg_dict.keys())}")
            last_print = now


if __name__ == "__main__":
    main()
