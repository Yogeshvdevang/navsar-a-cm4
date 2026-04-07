import argparse
import copy
import time

from pymavlink import mavutil


def parse_args():
    parser = argparse.ArgumentParser(description="Read Pixhawk barometer data via MAVLink.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port to Pixhawk")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--rate", type=float, default=5.0, help="Print rate in Hz")
    parser.add_argument(
        "--message",
        default="SCALED_PRESSURE",
        choices=["SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3", "HIGHRES_IMU"],
        help="MAVLink message to use for barometer data",
    )
    parser.add_argument(
        "--compute-alt",
        action="store_true",
        help="Compute altitude (m) from pressure and temperature",
    )
    parser.add_argument(
        "--output",
        default="raw",
        choices=["raw", "alt", "gps_input"],
        help="Output format: raw=pressure/temp, alt=altitude only, gps_input=GPS_INPUT-style alt field",
    )
    parser.add_argument("--wait-heartbeat", action="store_true", help="Wait for autopilot heartbeat")
    return parser.parse_args()


def pressure_to_alt_m(pressure_hpa, temp_c):
    if pressure_hpa <= 0:
        return None
    t_k = temp_c + 273.15
    return (t_k / 0.0065) * (1.0 - (pressure_hpa / 1013.25) ** (1.0 / 5.255))


def install_pymavlink_instance_workaround():
    """Guard against pymavlink messages with instance ids but no instance map."""
    original_add_message = mavutil.add_message

    def safe_add_message(messages, mtype, msg):
        if (
            getattr(msg, "_instance_field", None) is not None
            and getattr(msg, msg._instance_field, None) is not None
            and getattr(msg, "_instances", None) is None
        ):
            msg._instances = {}

        existing = messages.get(mtype)
        if (
            existing is not None
            and getattr(existing, "_instance_field", None) is not None
            and getattr(existing, "_instances", None) is None
        ):
            existing._instances = {}

        try:
            return original_add_message(messages, mtype, msg)
        except TypeError as exc:
            if "NoneType" not in str(exc) or "item assignment" not in str(exc):
                raise

            instance_field = getattr(msg, "_instance_field", None)
            instance_value = getattr(msg, instance_field, None) if instance_field else None
            if instance_field is None or instance_value is None:
                raise

            if mtype not in messages:
                messages[mtype] = copy.copy(msg)
                messages[mtype]._instances = {instance_value: msg}
            else:
                if getattr(messages[mtype], "_instances", None) is None:
                    messages[mtype]._instances = {}
                messages[mtype]._instances[instance_value] = msg
                prev_instances = messages[mtype]._instances
                messages[mtype] = copy.copy(msg)
                messages[mtype]._instances = prev_instances

            messages[f"{mtype}[{instance_value}]"] = copy.copy(msg)

    mavutil.add_message = safe_add_message


def read_baro(args):
    mavutil.mavlink.MAVLINK20 = 1
    install_pymavlink_instance_workaround()
    conn = mavutil.mavlink_connection(args.port, baud=args.baud)

    if args.wait_heartbeat:
        print("Waiting for autopilot heartbeat...")
        conn.wait_heartbeat(timeout=15)
        print("Heartbeat received.")

    msg_type = args.message
    dt = 1.0 / max(args.rate, 0.1)
    print(f"Listening for {msg_type}...")
    while True:
        msg = conn.recv_match(type=msg_type, blocking=True, timeout=1.0)
        if msg is None:
            continue

        if msg_type.startswith("SCALED_PRESSURE"):
            pressure_hpa = msg.press_abs
            temp_c = msg.temperature / 100.0
        else:
            pressure_hpa = msg.abs_pressure
            temp_c = msg.temperature

        alt_m = None
        if args.compute_alt or args.output in ("alt", "gps_input"):
            alt_m = pressure_to_alt_m(pressure_hpa, temp_c)

        if args.output == "raw":
            out = f"pressure_hpa={pressure_hpa:.2f} temp_c={temp_c:.2f}"
            if alt_m is not None:
                out += f" alt_m={alt_m:.2f}"
            elif args.compute_alt:
                out += " alt_m=nan"
        elif args.output == "alt":
            out = "alt_m=nan" if alt_m is None else f"alt_m={alt_m:.2f}"
        else:
            out = "alt=nan" if alt_m is None else f"alt={alt_m:.2f}"
        print(out)
        time.sleep(dt)


def main():
    args = parse_args()
    read_baro(args)
    print(pressure_to_alt_m(args))


if __name__ == "__main__":
    main()
