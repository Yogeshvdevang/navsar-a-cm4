"""IMU-only velocity estimation for VIO fallback mode."""

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


class ImuVelocityEstimator:
    """Estimate velocity by integrating IMU acceleration."""

    def __init__(
        self,
        gravity_m_s2=GRAVITY_M_S2,
        bias_calib_s=BIAS_CALIB_S,
        vel_damping=VEL_DAMPING,
    ):
        self.gravity_m_s2 = float(gravity_m_s2)
        self.bias_calib_s = float(bias_calib_s)
        self.vel_damping = float(vel_damping)
        self.last_time_s = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.have_attitude = False
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.bias_samples = []
        self.bias = None
        self.bias_frame = None
        self.calib_start = None

    def _calibrate_bias(self, ax, ay, az, current_frame):
        if self.bias is not None:
            return
        if self.calib_start is None:
            self.calib_start = time.time()
            self.bias_frame = current_frame
        if time.time() - self.calib_start <= self.bias_calib_s:
            self.bias_samples.append((ax, ay, az))
            return
        if self.bias_samples:
            bx = sum(s[0] for s in self.bias_samples) / len(self.bias_samples)
            by = sum(s[1] for s in self.bias_samples) / len(self.bias_samples)
            bz = sum(s[2] for s in self.bias_samples) / len(self.bias_samples)
            self.bias = (bx, by, bz)
            print(
                f"IMU bias calibrated ({self.bias_frame}): "
                f"{self.bias[0]:.4f}, {self.bias[1]:.4f}, {self.bias[2]:.4f}"
            )
        else:
            self.bias = (0.0, 0.0, 0.0)

    def update_attitude(self, roll, pitch, yaw):
        """Update attitude from an external source."""
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = float(yaw)
        self.have_attitude = True

    def process_message(self, msg):
        """Process a MAVLink message and return velocity tuple when updated."""
        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            self.roll = msg.roll
            self.pitch = msg.pitch
            self.yaw = msg.yaw
            self.have_attitude = True
            return None
        if msg_type not in ("HIGHRES_IMU", "RAW_IMU"):
            return None

        msg_time_s = _get_msg_time_s(msg)
        if msg_time_s is None:
            return None
        if self.last_time_s is None:
            self.last_time_s = msg_time_s
            return None
        dt = msg_time_s - self.last_time_s
        if dt <= 0.0 or dt > 1.0:
            self.last_time_s = msg_time_s
            return None
        self.last_time_s = msg_time_s

        if msg_type == "HIGHRES_IMU":
            ax, ay, az = msg.xacc, msg.yacc, msg.zacc
        else:
            ax = (msg.xacc / 1000.0) * self.gravity_m_s2
            ay = (msg.yacc / 1000.0) * self.gravity_m_s2
            az = (msg.zacc / 1000.0) * self.gravity_m_s2

        if self.have_attitude:
            r_body_to_ned = _rotation_body_to_ned(self.roll, self.pitch, self.yaw)
            ax, ay, az = _mat_vec_mul(r_body_to_ned, (ax, ay, az))
            az -= self.gravity_m_s2
            current_frame = "NED"
        else:
            current_frame = "BODY"

        if (
            self.bias is not None
            and self.bias_frame == "BODY"
            and current_frame == "NED"
        ):
            self.bias = None
            self.bias_samples.clear()
            self.bias_frame = None
            self.calib_start = None

        self._calibrate_bias(ax, ay, az, current_frame)
        if self.bias is None:
            return None

        ax -= self.bias[0]
        ay -= self.bias[1]
        az -= self.bias[2]

        self.vx = self.vx * self.vel_damping + ax * dt
        self.vy = self.vy * self.vel_damping + ay * dt
        self.vz = self.vz * self.vel_damping + az * dt

        return self.vx, self.vy, self.vz, current_frame


def run_with_master(
    master,
    print_interval_s=PRINT_INTERVAL_S,
    imu_rate_hz=IMU_RATE_HZ,
    print_enabled=True,
):
    """Stream IMU velocities from an existing MAVLink master."""
    _request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, imu_rate_hz)
    _request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, imu_rate_hz)
    _request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, imu_rate_hz)

    estimator = ImuVelocityEstimator()
    last_print = 0.0
    last_msg_time = time.time()

    while True:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            if time.time() - last_msg_time > 5.0 and print_enabled:
                print("No IMU messages yet... check stream/rates.")
                last_msg_time = time.time()
            continue

        last_msg_time = time.time()
        result = estimator.process_message(msg)
        if result is None:
            continue
        vx, vy, vz, frame = result
        if not print_enabled:
            continue
        now = time.time()
        if print_interval_s > 0.0 and (now - last_print) < print_interval_s:
            continue
        last_print = now
        print(f"{frame} Vx: {vx:.3f} | Vy: {vy:.3f} | Vz: {vz:.3f} m/s")


def run():
    """Stream IMU velocities and print Vx/Vy/Vz."""
    print(f"Connecting to MAVLink on {DEVICE} @ {BAUD}...")
    master = mavutil.mavlink_connection(DEVICE, baud=BAUD)
    try:
        master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
    except Exception as exc:
        raise RuntimeError("Failed to receive MAVLink heartbeat") from exc

    print("Heartbeat received. Streaming IMU acceleration + attitude...")
    run_with_master(master)


if __name__ == "__main__":
    run()
