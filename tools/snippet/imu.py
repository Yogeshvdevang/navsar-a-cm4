import math
import os
import time

from pymavlink import mavutil


DEVICE = os.getenv("MAVLINK_DEVICE", "/dev/ttyACM0")
BAUD = int(os.getenv("MAVLINK_BAUD", "115200"))
HEARTBEAT_TIMEOUT_S = float(os.getenv("MAVLINK_HEARTBEAT_TIMEOUT_S", "5.0"))
IMU_RATE_HZ = float(os.getenv("MAVLINK_IMU_RATE_HZ", "50.0"))
PRINT_INTERVAL_S = float(os.getenv("MAVLINK_PRINT_INTERVAL_S", "0.1"))
GRAVITY_M_S2 = 9.80665
BIAS_CALIB_S = float(os.getenv("IMU_BIAS_CALIB_S", "3.0"))
VEL_DAMPING = float(os.getenv("IMU_VEL_DAMPING", "0.98"))



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


def _rotation_body_to_ned(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return (
        (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        (-sp, sr * cp, cr * cp),
    )


def _mat_vec_mul(mat, vec):
    return (
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    )


def _get_msg_time_s(msg):
    time_usec = getattr(msg, "time_usec", 0) or 0
    if time_usec:
        return time_usec * 1e-6
    time_boot_ms = getattr(msg, "time_boot_ms", 0) or 0
    if time_boot_ms:
        return time_boot_ms * 1e-3
    return None


def main():
    print(f"Connecting to MAVLink on {DEVICE} @ {BAUD}...")
    master = mavutil.mavlink_connection(DEVICE, baud=BAUD)
    try:
        master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
    except Exception as exc:
        raise RuntimeError("Failed to receive MAVLink heartbeat") from exc

    print("Heartbeat received. Streaming IMU acceleration + attitude...")
    _request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, IMU_RATE_HZ)
    _request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, IMU_RATE_HZ)
    _request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, IMU_RATE_HZ)

    last_time_s = None
    roll = pitch = yaw = 0.0
    have_attitude = False
    vx = vy = vz = 0.0
    last_print = 0.0
    last_msg_time = time.time()
    bias_samples = []
    bias = None
    bias_frame = None
    calib_start = None

    while True:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            if time.time() - last_msg_time > 5.0:
                print("No IMU messages yet... check stream/rates.")
                last_msg_time = time.time()
            continue

        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            roll = msg.roll
            pitch = msg.pitch
            yaw = msg.yaw
            have_attitude = True
            continue

        if msg_type not in ("HIGHRES_IMU", "RAW_IMU"):
            continue

        last_msg_time = time.time()
        msg_time_s = _get_msg_time_s(msg)
        if msg_time_s is None:
            continue
        if last_time_s is None:
            last_time_s = msg_time_s
            continue
        dt = msg_time_s - last_time_s
        if dt <= 0.0 or dt > 1.0:
            last_time_s = msg_time_s
            continue
        last_time_s = msg_time_s

        if msg_type == "HIGHRES_IMU":
            ax, ay, az = msg.xacc, msg.yacc, msg.zacc
        else:
            ax = (msg.xacc / 1000.0) * GRAVITY_M_S2
            ay = (msg.yacc / 1000.0) * GRAVITY_M_S2
            az = (msg.zacc / 1000.0) * GRAVITY_M_S2

        if have_attitude:
            r_body_to_ned = _rotation_body_to_ned(roll, pitch, yaw)
            ax, ay, az = _mat_vec_mul(r_body_to_ned, (ax, ay, az))
            az -= GRAVITY_M_S2
            current_frame = "NED"
        else:
            current_frame = "BODY"

        if bias is not None and bias_frame == "BODY" and current_frame == "NED":
            bias = None
            bias_samples.clear()
            bias_frame = None
            calib_start = None

        if bias is None:
            if calib_start is None:
                calib_start = time.time()
                bias_frame = current_frame
            if time.time() - calib_start <= BIAS_CALIB_S:
                bias_samples.append((ax, ay, az))
                continue
            if bias_samples:
                bx = sum(s[0] for s in bias_samples) / len(bias_samples)
                by = sum(s[1] for s in bias_samples) / len(bias_samples)
                bz = sum(s[2] for s in bias_samples) / len(bias_samples)
                bias = (bx, by, bz)
                print(
                    f"IMU bias calibrated ({bias_frame}): "
                    f"{bias[0]:.4f}, {bias[1]:.4f}, {bias[2]:.4f}"
                )
            else:
                bias = (0.0, 0.0, 0.0)

        ax -= bias[0]
        ay -= bias[1]
        az -= bias[2]

        vx = vx * VEL_DAMPING + ax * dt
        vy = vy * VEL_DAMPING + ay * dt
        vz = vz * VEL_DAMPING + az * dt

        now = time.time()
        if PRINT_INTERVAL_S > 0.0 and (now - last_print) < PRINT_INTERVAL_S:
            continue
        last_print = now
        frame = "NED" if have_attitude else "BODY"
        print(f"{frame} Vx: {vx:.3f} | Vy: {vy:.3f} | Vz: {vz:.3f} m/s")


if __name__ == "__main__":
    main()
