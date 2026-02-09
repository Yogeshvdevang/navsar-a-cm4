"""Sensor fusion using a UKF over IMU + camera measurements."""

import math
import time

import numpy as np

try:
    from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
except ImportError as exc:  # pragma: no cover - runtime dependency
    UnscentedKalmanFilter = None
    MerweScaledSigmaPoints = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


def _rotation_matrix(roll, pitch, yaw):
    """Return body->world rotation matrix for roll/pitch/yaw (rad)."""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


class SensorFusion:
    """UKF-based fusion of IMU (predict) and camera (update) data."""
    def __init__(
        self,
        accel_noise=0.5,
        gyro_noise=0.1,
        camera_pos_noise=0.5,
        camera_vel_noise=0.5,
        gravity=9.80665,
        alpha=0.3,
        beta=2.0,
        kappa=0.0,
    ):
        if UnscentedKalmanFilter is None:
            raise ImportError(
                "filterpy is required for UKF; install it to use SensorFusion."
            ) from _IMPORT_ERROR

        self.gravity = float(gravity)
        self._last_imu_time = None
        self._initialized = False

        # State: [x,y,z, vx,vy,vz, roll,pitch,yaw, bax,bay,baz, bgx,bgy,bgz]
        self.dim_x = 15
        self.dim_z = 6  # camera provides position + velocity

        points = MerweScaledSigmaPoints(
            n=self.dim_x, alpha=alpha, beta=beta, kappa=kappa
        )
        self.ukf = UnscentedKalmanFilter(
            dim_x=self.dim_x,
            dim_z=self.dim_z,
            dt=0.01,
            fx=self._fx,
            hx=self._hx_camera,
            points=points,
        )

        # Initial state and covariance.
        self.ukf.x = np.zeros(self.dim_x, dtype=float)
        self.ukf.P = np.eye(self.dim_x, dtype=float) * 1.0

        # Process noise for accel/gyro and bias random walk.
        q = np.zeros(self.dim_x)
        q[0:3] = 0.01
        q[3:6] = accel_noise
        q[6:9] = gyro_noise
        q[9:12] = 0.01
        q[12:15] = 0.01
        self.ukf.Q = np.diag(q)

        r = np.zeros(self.dim_z)
        r[0:3] = camera_pos_noise
        r[3:6] = camera_vel_noise
        self.ukf.R = np.diag(r)

    @staticmethod
    def _is_finite_array(values):
        return np.all(np.isfinite(values))

    def _reset_filter(self):
        self.ukf.x = np.zeros(self.dim_x, dtype=float)
        self.ukf.P = np.eye(self.dim_x, dtype=float)
        self._initialized = False
        self._last_imu_time = None

    def _fx(self, x, dt, u):
        """Process model: integrate IMU to update pose/velocity."""
        ax, ay, az, gx, gy, gz = u

        # Unpack state.
        px, py, pz = x[0:3]
        vx, vy, vz = x[3:6]
        roll, pitch, yaw = x[6:9]
        bax, bay, baz = x[9:12]
        bgx, bgy, bgz = x[12:15]

        # Bias-corrected IMU.
        acc_body = np.array([ax - bax, ay - bay, az - baz], dtype=float)
        gyro = np.array([gx - bgx, gy - bgy, gz - bgz], dtype=float)

        # Integrate orientation (small-angle approx).
        roll += gyro[0] * dt
        pitch += gyro[1] * dt
        yaw += gyro[2] * dt

        # Convert acceleration to world frame (ENU).
        rot = _rotation_matrix(roll, pitch, yaw)
        acc_world = rot @ acc_body + np.array([0.0, 0.0, self.gravity])

        # Integrate velocity and position.
        vx += acc_world[0] * dt
        vy += acc_world[1] * dt
        vz += acc_world[2] * dt
        px += vx * dt
        py += vy * dt
        pz += vz * dt

        x_out = np.array(
            [
                px,
                py,
                pz,
                vx,
                vy,
                vz,
                roll,
                pitch,
                yaw,
                bax,
                bay,
                baz,
                bgx,
                bgy,
                bgz,
            ],
            dtype=float,
        )
        return x_out

    def _hx_camera(self, x):
        """Measurement model for camera position + velocity."""
        return x[0:6]

    def update_imu(self, imu_data):
        """Predict using IMU data; expects ax/ay/az (m/s^2) and gx/gy/gz (rad/s)."""
        now = imu_data.get("time_s", time.time())
        if self._last_imu_time is None:
            self._last_imu_time = now
            return
        dt_raw = float(now - self._last_imu_time)
        if dt_raw <= 0.0 or dt_raw > 0.5:
            self._last_imu_time = now
            return
        dt = max(1e-3, dt_raw)
        self._last_imu_time = now
        u = np.array(
            [
                imu_data["ax"],
                imu_data["ay"],
                imu_data["az"],
                imu_data["gx"],
                imu_data["gy"],
                imu_data["gz"],
            ],
            dtype=float,
        )
        if not self._is_finite_array(u):
            return
        try:
            self.ukf.predict(dt=dt, u=u)
        except np.linalg.LinAlgError:
            # Reset covariance to recover from numerical issues.
            self._reset_filter()
            return
        if not self._is_finite_array(self.ukf.x) or not self._is_finite_array(self.ukf.P):
            self._reset_filter()

    def update_camera(self, camera_data):
        """Update with camera position/velocity measurements."""
        z = np.array(
            [
                camera_data["x"],
                camera_data["y"],
                camera_data["z"],
                camera_data["vx"],
                camera_data["vy"],
                camera_data["vz"],
            ],
            dtype=float,
        )
        if not self._is_finite_array(z):
            return
        if not self._initialized:
            self.ukf.x[0:6] = z
            self._initialized = True
        try:
            self.ukf.update(z)
        except np.linalg.LinAlgError:
            self._reset_filter()
            return
        if not self._is_finite_array(self.ukf.x) or not self._is_finite_array(self.ukf.P):
            self._reset_filter()

    def set_state(self, x, P=None):
        """Set the UKF state (and optionally covariance)."""
        self.ukf.x = np.array(x, dtype=float)
        if P is not None:
            self.ukf.P = np.array(P, dtype=float)

    def fused_state(self):
        """Return the fused state dict."""
        x = self.ukf.x
        return {
            "x": float(x[0]),
            "y": float(x[1]),
            "z": float(x[2]),
            "vx": float(x[3]),
            "vy": float(x[4]),
            "vz": float(x[5]),
            "roll": float(x[6]),
            "pitch": float(x[7]),
            "yaw": float(x[8]),
            "bax": float(x[9]),
            "bay": float(x[10]),
            "baz": float(x[11]),
            "bgx": float(x[12]),
            "bgy": float(x[13]),
            "bgz": float(x[14]),
        }
