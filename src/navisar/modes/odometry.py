"""Mode for sending MAVLink ODOMETRY."""

import numpy as np

from navisar.modes.common import EnuVelocityTracker


def quat_from_rpy(roll, pitch, yaw):
    """Convert roll/pitch/yaw (rad) to a quaternion."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [float(qw), float(qx), float(qy), float(qz)]


class OdometryMode:
    """Send MAVLink ODOMETRY from ENU camera drift."""
    def __init__(self, send_interval_s, print_interval_s, warn_interval_s=2.0):
        self.send_interval_s = float(send_interval_s)
        self.print_interval_s = float(print_interval_s)
        self.warn_interval_s = float(warn_interval_s)
        self._last_send = 0.0
        self._last_print = 0.0
        self._last_warn = 0.0
        self._vel_tracker = EnuVelocityTracker()
        self.last_payload = None

    def _warn(self, now, message):
        if now - self._last_warn >= self.warn_interval_s:
            print(message)
            self._last_warn = now

    def handle(self, now, x_m, y_m, z_m, mavlink_interface, attitude):
        """Send MAVLink odometry if ready."""
        if mavlink_interface is None:
            self._warn(now, "ODOM(NED) SEND: MAVLink unavailable.")
            return
        if now - self._last_send < self.send_interval_s:
            return
        if attitude is None:
            self._warn(now, "ODOM(NED) SEND: waiting for ATTITUDE data from Pixhawk...")
            return

        roll = attitude["roll"]
        pitch = attitude["pitch"]
        yaw = attitude["yaw"]
        q = quat_from_rpy(roll, pitch, yaw)

        # Convert ENU (VO) into NED for MAVLink odometry.
        x_ned = y_m
        y_ned = x_m
        z_ned = -z_m

        vx_enu, vy_enu, vz_enu = self._vel_tracker.velocity_and_update(
            now, x_m, y_m, z_m
        )
        vx = vy_enu
        vy = vx_enu
        vz = -vz_enu

        mavlink_interface.send_odometry(
            x_ned,
            y_ned,
            z_ned,
            q,
            vx,
            vy,
            vz,
            roll_rate=0.0,
            pitch_rate=0.0,
            yaw_rate=0.0,
        )
        self.last_payload = {
            "time_s": now,
            "x_ned": x_ned,
            "y_ned": y_ned,
            "z_ned": z_ned,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "q": q,
        }

        if now - self._last_print >= self.print_interval_s:
            print(
                "ODOM(NED) SEND: "
                f"X={x_ned:.2f} Y={y_ned:.2f} Z={z_ned:.2f} | "
                f"Vx={vx:.2f} Vy={vy:.2f} Vz={vz:.2f} | "
                f"R={roll:.2f} P={pitch:.2f} Y={yaw:.2f} | "
                f"Q=[{q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f}]"
            )
            self._last_print = now

        self._last_send = now
