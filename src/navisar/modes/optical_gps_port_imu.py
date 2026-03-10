"""IMU-augmented optical-flow GPS port mode."""

from __future__ import annotations

import copy
import math
import time

from navisar.modes.optical_flow_gps_port import OpticalFlowGpsPortMode


HEALTH_WINDOW_S = 0.1


class OpticalGpsPortImuMode(OpticalFlowGpsPortMode):
    """Preprocess optical-flow samples with IMU data, then reuse base pipeline."""

    def __init__(
        self,
        gps_port_mode,
        *,
        imu_enabled=False,
        imu_f_pixels=16.0,
        imu_beta=0.05,
        gyro_comp_enabled=True,
        accel_fuse_enabled=True,
        tilt_corr_enabled=True,
        imu_provider=None,
        attitude_provider=None,
        **kwargs,
    ):
        super().__init__(gps_port_mode=gps_port_mode, **kwargs)
        self.imu_enabled = bool(imu_enabled)
        self.imu_f_pixels = float(imu_f_pixels)
        self.imu_beta = max(0.0, min(1.0, float(imu_beta)))
        self.gyro_comp_enabled = bool(gyro_comp_enabled)
        self.accel_fuse_enabled = bool(accel_fuse_enabled)
        self.tilt_corr_enabled = bool(tilt_corr_enabled)
        self.accel_vx = 0.0
        self.accel_vy = 0.0
        self._imu_last_mono = None
        self._imu_last_warn = 0.0
        self._imu_last_stats = 0.0
        self._imu_attempts = 0
        self._imu_successes = 0
        self._last_imu_sample = None
        self._imu_provider = imu_provider
        self._attitude_provider = attitude_provider
        if self.imu_enabled and self._imu_provider is None:
            self.imu_enabled = False
            print("Warning: optical_gps_port_imu has no Pixhawk IMU provider; falling back to optical GPS port mode.")
        elif self.imu_enabled:
            print("IMU enabled for optical_gps_port_imu via Pixhawk MAVLink IMU/ATTITUDE.")

    def _warn_imu(self, now_mono: float, message: str):
        if now_mono - self._imu_last_warn >= self.warn_interval_s:
            print(message)
            self._imu_last_warn = now_mono

    def _log_imu_stats(self, now_mono: float):
        if not self.imu_enabled:
            return
        if now_mono - self._imu_last_stats >= 5.0:
            success_rate = (
                1.0
                if self._imu_attempts <= 0
                else float(self._imu_successes) / float(self._imu_attempts)
            )
            healthy = int(self._last_imu_sample is not None)
            print(
                "IMU health: "
                f"success_rate={success_rate * 100.0:.1f}% healthy={healthy}"
            )
            self._imu_last_stats = now_mono

    def _imu_dt(self, sample) -> float:
        now_mono = time.monotonic()
        dt_s = None
        if self._imu_last_mono is not None and now_mono > self._imu_last_mono:
            dt_s = now_mono - self._imu_last_mono
        elif sample is not None and self._last_time_ms is not None and sample.time_ms is not None:
            dt_ms = sample.time_ms - self._last_time_ms
            if dt_ms > 0:
                dt_s = dt_ms / 1000.0
        self._imu_last_mono = now_mono
        return now_mono, max(0.0, 0.0 if dt_s is None else dt_s)

    def _read_imu_sample(self, now_mono: float):
        if not self.imu_enabled or self._imu_provider is None:
            return None
        try:
            self._imu_attempts += 1
            imu = self._imu_provider()
            attitude = self._attitude_provider() if self._attitude_provider is not None else None
            if not isinstance(imu, dict):
                raise ValueError("missing Pixhawk IMU sample")
            imu_time_s = imu.get("time_s")
            imu_age_ok = (
                imu_time_s is not None and (time.time() - float(imu_time_s)) <= HEALTH_WINDOW_S
            )
            if not imu_age_ok:
                raise ValueError("stale Pixhawk IMU sample")
            roll_rad = None
            pitch_rad = None
            if isinstance(attitude, dict):
                att_time_s = attitude.get("time_s")
                if (
                    att_time_s is not None
                    and (time.time() - float(att_time_s)) <= HEALTH_WINDOW_S
                ):
                    roll_rad = float(attitude.get("roll", 0.0))
                    pitch_rad = float(attitude.get("pitch", 0.0))
            if roll_rad is None or pitch_rad is None:
                ax = float(imu.get("ax", 0.0))
                ay = float(imu.get("ay", 0.0))
                az = float(imu.get("az", 0.0))
                roll_rad = math.atan2(ay, az)
                pitch_rad = math.atan2(-ax, math.hypot(ay, az))
            sample = {
                "gyro_x": float(imu.get("gx", 0.0)),
                "gyro_y": float(imu.get("gy", 0.0)),
                "gyro_z": float(imu.get("gz", 0.0)),
                "accel_x": float(imu.get("ax", 0.0)),
                "accel_y": float(imu.get("ay", 0.0)),
                "accel_z": float(imu.get("az", 0.0)),
                "roll_rad": roll_rad,
                "pitch_rad": pitch_rad,
                "timestamp_monotonic": now_mono,
            }
            self._imu_successes += 1
            self._last_imu_sample = sample
            return sample
        except Exception as exc:
            cached = self._last_imu_sample
            if (
                cached is not None
                and now_mono - float(cached["timestamp_monotonic"]) <= HEALTH_WINDOW_S
            ):
                self._warn_imu(now_mono, f"Warning: IMU read failed ({exc}); using cached sample.")
                return cached
            self._warn_imu(now_mono, f"Warning: IMU read failed ({exc}); skipping IMU correction.")
            return None

    def _preprocess_sample(self, sample, dt_s: float, imu_sample):
        processed = copy.copy(sample)
        processed.speed_x = float(sample.speed_x)
        processed.speed_y = float(sample.speed_y)
        if (
            imu_sample is not None
            and self.gyro_comp_enabled
            and dt_s > 0.0
        ):
            processed.speed_x = (
                processed.speed_x - (float(imu_sample["gyro_x"]) * dt_s * self.imu_f_pixels)
            )
            processed.speed_y = (
                processed.speed_y - (float(imu_sample["gyro_y"]) * dt_s * self.imu_f_pixels)
            )

        scale = self.speed_scale
        vx_body = processed.speed_x * scale
        vy_body = processed.speed_y * scale
        if (
            imu_sample is not None
            and self.accel_fuse_enabled
            and dt_s > 0.0
        ):
            self.accel_vx += float(imu_sample["accel_x"]) * dt_s
            self.accel_vy += float(imu_sample["accel_y"]) * dt_s
            beta = self.imu_beta
            vx_body = (1.0 - beta) * vx_body + beta * self.accel_vx
            vy_body = (1.0 - beta) * vy_body + beta * self.accel_vy
            self.accel_vx = vx_body
            self.accel_vy = vy_body
        else:
            self.accel_vx = vx_body
            self.accel_vy = vy_body

        if abs(scale) > 1e-9:
            processed.speed_x = vx_body / scale
            processed.speed_y = vy_body / scale
        else:
            processed.speed_x = 0.0
            processed.speed_y = 0.0

        if processed.dist_ok:
            corrected_distance_mm = float(sample.distance_mm)
            if imu_sample is not None and self.tilt_corr_enabled:
                corrected_distance_mm = corrected_distance_mm * math.cos(
                    float(imu_sample["roll_rad"])
                ) * math.cos(float(imu_sample["pitch_rad"]))
            corrected_distance_mm = max(0.0, corrected_distance_mm)
            processed.distance_mm = int(round(corrected_distance_mm))
            processed.dist_cm = processed.distance_mm / 10.0 if processed.distance_mm > 0 else 0.0
        return processed

    def handle(
        self,
        now,
        sample,
        origin,
        alt_override_m=None,
        heading_deg=None,
        send_heading=True,
        heading_only=False,
    ):
        now_mono, imu_dt_s = self._imu_dt(sample)
        self._log_imu_stats(now_mono)
        if sample is None or not self.imu_enabled:
            return super().handle(
                now,
                sample,
                origin,
                alt_override_m=alt_override_m,
                heading_deg=heading_deg,
                send_heading=send_heading,
                heading_only=heading_only,
            )
        imu_sample = self._read_imu_sample(now_mono)
        processed_sample = self._preprocess_sample(sample, imu_dt_s, imu_sample)
        return super().handle(
            now,
            processed_sample,
            origin,
            alt_override_m=None,
            heading_deg=heading_deg,
            send_heading=send_heading,
            heading_only=heading_only,
        )
