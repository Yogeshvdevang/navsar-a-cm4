import argparse
import math
import time


from pymavlink import mavutil



def parse_args():
    parser = argparse.ArgumentParser(description="Send fake GPS via MAVLink.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port to Pixhawk")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--rate", type=float, default=5.0, help="Update rate in Hz")
    parser.add_argument("--lat", type=float, default=12.9716, help="Start latitude (deg)")
    parser.add_argument("--lon", type=float, default=77.5946, help="Start longitude (deg)")
    parser.add_argument("--alt", type=float, default=920.0, help="Start altitude (m)")
    parser.add_argument("--speed", type=float, default=1.5, help="Ground speed (m/s)")
    parser.add_argument("--heading", type=float, default=90.0, help="Heading (deg, 0=north)")
    parser.add_argument("--sats", type=int, default=10, help="Satellites visible")
    parser.add_argument("--gps-id", type=int, default=0, help="GPS id")
    parser.add_argument("--system-id", type=int, default=1, help="MAVLink system id")
    parser.add_argument("--no-heartbeat", action="store_true", help="Disable heartbeat")
    parser.add_argument("--wait-heartbeat", action="store_true", help="Wait for autopilot heartbeat")
    parser.add_argument("--print-raw", action="store_true", help="Print raw MAVLink bytes")
    return parser.parse_args()


def send_fake_gps(args):
    mavutil.mavlink.MAVLINK20 = 1
    conn = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=args.system_id,
        source_component=mavutil.mavlink.MAV_COMP_ID_GPS,
    )

    if args.wait_heartbeat:
        print("Waiting for autopilot heartbeat...")
        conn.wait_heartbeat(timeout=15)
        print("Heartbeat received.")

    lat = args.lat
    lon = args.lon
    alt = args.alt
    speed = args.speed
    heading = args.heading
    sats = args.sats
    dt = 1.0 / max(args.rate, 0.1)

    last_hb = 0.0
    print("Sending MAVLink GPS_INPUT to Pixhawk...")
    while True:
        now = time.time()

        heading_rad = math.radians(heading)
        vn = speed * math.cos(heading_rad)
        ve = speed * math.sin(heading_rad)
        vd = 0.0

        lat += (vn * dt) / 111111.0
        lon += (ve * dt) / (111111.0 * math.cos(math.radians(lat)))

        alt += math.sin(now * 0.5) * 0.05

        if not args.no_heartbeat and now - last_hb > 1.0:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GPS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            last_hb = now

        gps_msg = conn.mav.gps_input_encode(
            int(now * 1e6),
            args.gps_id,
            0,
            0,
            0,
            3,
            int(lat * 1e7),
            int(lon * 1e7),
            float(alt),
            0.7,
            1.0,
            float(vn),
            float(ve),
            float(vd),
            0.1,
            0.5,
            0.8,
            sats,
        )
        if args.print_raw:
            raw = gps_msg.pack(conn.mav)
            #print(f"GPS_INPUT RAW: {raw.hex()}")
        print("+++++++++++++++++++++++++++++++++++++++++++++++++")
        print(gps_msg)
        print("+++++++++++++++++++++++++++++++++++++++++++++++++")
        conn.mav.send(gps_msg)

        time.sleep(dt)


def main():
    args = parse_args()
    send_fake_gps(args)


if __name__ == "__main__":
    main()
