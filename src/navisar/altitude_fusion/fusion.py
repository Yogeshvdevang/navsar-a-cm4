"""Production AMSL fusion core: baro propagation + lidar AGL correction via ground state."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class FusionConfig:
    """Tunable parameters for AMSL/AGL fusion."""

    lidar_min_m: float = 0.08
    lidar_max_m: float = 8.0
    max_tilt_deg: float = 20.0
    lidar_timeout_s: float = 0.2
    max_agl_rate_mps: float = 6.0
    baro_lpf_alpha: float = 0.15
    lidar_lpf_alpha: float = 0.35
    gps_alt_lpf_alpha: float = 0.20
    ground_update_gamma_strong: float = 0.05
    ground_update_gamma_weak: float = 0.01
    correction_gain_strong: float = 0.15
    correction_gain_weak: float = 0.03
    innovation_max_m: float = 0.75
    ground_jump_strong_m: float = 0.8
    ground_jump_reject_m: float = 2.0
    lidar_mount_roll_rad: float = 0.0
    lidar_mount_pitch_rad: float = 0.0
    lidar_quality_min: float = 20.0
    innovation_reject_m: float = 3.0


@dataclass
class MavSample:
    """One MAVLink-aligned input snapshot."""

    timestamp_s: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    alt_amsl_m: float
    relative_alt_m: float
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None


@dataclass
class LidarSample:
    """One lidar sample from Pi-connected range sensor."""

    timestamp_s: float
    distance_m: float
    quality: Optional[float] = None
    healthy: bool = True


@dataclass
class FusionInput:
    """Fusion cycle input: MAV sample and optional lidar sample."""

    mav: MavSample
    lidar: Optional[LidarSample] = None


@dataclass
class FusionOutput:
    """Fusion cycle output and observability fields."""

    timestamp_s: float
    mode: str
    h_amsl_pred_m: float
    h_amsl_est_m: float
    h_amsl_lidar_m: Optional[float]
    h_ground_est_m: float
    h_ground_meas_m: Optional[float]
    h_agl_filt_m: Optional[float]
    baro_rel_filt_m: float
    lidar_valid: bool
    lidar_confidence: float
    correction_gain: float
    ground_gamma: float
    innovation_m: Optional[float]
    innovation_clamped_m: Optional[float]
    validity_flags: Dict[str, bool] = field(default_factory=dict)


class AltitudeFusion:
    """State machine + equations for production AMSL fusion."""

    MODE_STARTUP = "startup_ground_lock"
    MODE_SAME_SURFACE = "same_surface_low_altitude"
    MODE_UNCERTAIN = "uncertain_terrain_transition"
    MODE_LIDAR_INVALID = "lidar_invalid_baro_only"

    def __init__(self, config: Optional[FusionConfig] = None):
        self.cfg = config or FusionConfig()
        self._initialized = False

        self._h_amsl_est_m = 0.0
        self._h_ground_est_m = 0.0
        self._h_gps_out_m = 0.0

        self._baro_rel_filt_m: Optional[float] = None
        self._baro_rel_prev_filt_m: Optional[float] = None
        self._h_agl_filt_m: Optional[float] = None
        self._h_agl_prev_m: Optional[float] = None

        self._last_valid_lidar_t_s: Optional[float] = None
        self._last_step_t_s: Optional[float] = None
        self._mode = self.MODE_STARTUP

    @property
    def initialized(self) -> bool:
        return self._initialized

    @staticmethod
    def _clip(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def _tilt_corrected_agl(self, dist_m: float, roll_rad: float, pitch_rad: float) -> float:
        cfg = self.cfg
        return dist_m * math.cos(roll_rad + cfg.lidar_mount_roll_rad) * math.cos(
            pitch_rad + cfg.lidar_mount_pitch_rad
        )

    def _lpf(self, prev: Optional[float], value: float, alpha: float) -> float:
        if prev is None:
            return value
        return alpha * value + (1.0 - alpha) * prev

    def _initialize(self, fusion_input: FusionInput) -> None:
        mav = fusion_input.mav
        lidar = fusion_input.lidar

        self._h_amsl_est_m = mav.alt_amsl_m
        self._h_gps_out_m = mav.alt_amsl_m
        self._baro_rel_filt_m = mav.relative_alt_m
        self._baro_rel_prev_filt_m = mav.relative_alt_m

        if lidar is not None and lidar.healthy:
            h_agl = self._tilt_corrected_agl(lidar.distance_m, mav.roll_rad, mav.pitch_rad)
            self._h_agl_filt_m = h_agl
            self._h_agl_prev_m = h_agl
            self._h_ground_est_m = self._h_amsl_est_m - h_agl
            self._last_valid_lidar_t_s = lidar.timestamp_s
        else:
            self._h_ground_est_m = self._h_amsl_est_m

        self._mode = self.MODE_STARTUP
        self._last_step_t_s = mav.timestamp_s
        self._initialized = True

    def _compute_lidar_validity(
        self,
        now_s: float,
        mav: MavSample,
        lidar: Optional[LidarSample],
        h_agl_raw_m: Optional[float],
        h_amsl_pred_m: float,
    ) -> tuple[bool, float, Dict[str, bool], Optional[float]]:
        flags = {
            "has_lidar": lidar is not None,
            "healthy": False,
            "range_ok": False,
            "tilt_ok": False,
            "fresh_ok": False,
            "quality_ok": False,
            "agl_rate_ok": False,
            "innovation_ok": False,
        }
        if lidar is None or h_agl_raw_m is None:
            return False, 0.0, flags, None

        cfg = self.cfg
        roll_deg = abs(math.degrees(mav.roll_rad))
        pitch_deg = abs(math.degrees(mav.pitch_rad))

        flags["healthy"] = bool(lidar.healthy)
        flags["range_ok"] = cfg.lidar_min_m < lidar.distance_m < cfg.lidar_max_m
        flags["tilt_ok"] = roll_deg <= cfg.max_tilt_deg and pitch_deg <= cfg.max_tilt_deg
        flags["fresh_ok"] = (now_s - lidar.timestamp_s) <= cfg.lidar_timeout_s

        if lidar.quality is None:
            flags["quality_ok"] = True
            quality_factor = 1.0
        else:
            flags["quality_ok"] = float(lidar.quality) >= cfg.lidar_quality_min
            quality_factor = self._clip(float(lidar.quality) / 100.0, 0.0, 1.0)

        agl_rate_ok = True
        if self._h_agl_prev_m is not None and self._last_step_t_s is not None:
            dt = max(1e-3, now_s - self._last_step_t_s)
            agl_rate = abs((h_agl_raw_m - self._h_agl_prev_m) / dt)
            agl_rate_ok = agl_rate <= cfg.max_agl_rate_mps
        flags["agl_rate_ok"] = agl_rate_ok

        h_amsl_lidar = self._h_ground_est_m + h_agl_raw_m
        innovation_abs = abs(h_amsl_lidar - h_amsl_pred_m)
        flags["innovation_ok"] = innovation_abs <= cfg.innovation_reject_m

        all_valid = all(flags.values())
        if not all_valid:
            return False, 0.0, flags, innovation_abs

        tilt_factor = self._clip(1.0 - max(roll_deg, pitch_deg) / cfg.max_tilt_deg, 0.0, 1.0)
        range_span = max(1e-6, cfg.lidar_max_m - cfg.lidar_min_m)
        center = 0.5 * (cfg.lidar_max_m + cfg.lidar_min_m)
        dist_factor = 1.0 - abs(lidar.distance_m - center) / (0.5 * range_span)
        dist_factor = self._clip(dist_factor, 0.2, 1.0)
        innov_factor = self._clip(1.0 - innovation_abs / cfg.innovation_reject_m, 0.0, 1.0)

        confidence = quality_factor * tilt_factor * dist_factor * innov_factor
        return True, confidence, flags, innovation_abs

    def step(self, fusion_input: FusionInput) -> FusionOutput:
        """Advance one fusion cycle using current MAV + lidar inputs."""
        if not self._initialized:
            self._initialize(fusion_input)

        mav = fusion_input.mav
        lidar = fusion_input.lidar
        now_s = mav.timestamp_s

        self._baro_rel_filt_m = self._lpf(
            self._baro_rel_filt_m,
            mav.relative_alt_m,
            self.cfg.baro_lpf_alpha,
        )

        if self._baro_rel_prev_filt_m is None:
            delta_baro_m = 0.0
        else:
            delta_baro_m = self._baro_rel_filt_m - self._baro_rel_prev_filt_m
        h_amsl_pred_m = self._h_amsl_est_m + delta_baro_m

        h_agl_raw_m = None
        if lidar is not None:
            h_agl_raw_m = self._tilt_corrected_agl(lidar.distance_m, mav.roll_rad, mav.pitch_rad)

        lidar_valid, lidar_conf, validity_flags, _innovation_abs = self._compute_lidar_validity(
            now_s,
            mav,
            lidar,
            h_agl_raw_m,
            h_amsl_pred_m,
        )

        h_ground_meas_m = None
        h_amsl_lidar_m = None
        innovation_m = None
        innovation_clamped_m = None
        correction_gain = 0.0
        ground_gamma = 0.0

        if lidar_valid and h_agl_raw_m is not None:
            self._h_agl_filt_m = self._lpf(self._h_agl_filt_m, h_agl_raw_m, self.cfg.lidar_lpf_alpha)
            h_ground_meas_m = h_amsl_pred_m - self._h_agl_filt_m
            e_ground_m = h_ground_meas_m - self._h_ground_est_m
            jump_abs = abs(e_ground_m)

            if jump_abs < self.cfg.ground_jump_strong_m:
                base_gain = self.cfg.correction_gain_strong
                base_gamma = self.cfg.ground_update_gamma_strong
                self._mode = self.MODE_SAME_SURFACE
            elif jump_abs < self.cfg.ground_jump_reject_m:
                base_gain = self.cfg.correction_gain_weak
                base_gamma = self.cfg.ground_update_gamma_weak
                self._mode = self.MODE_UNCERTAIN
            else:
                base_gain = 0.0
                base_gamma = 0.0
                self._mode = self.MODE_UNCERTAIN

            correction_gain = base_gain * lidar_conf
            ground_gamma = base_gamma * lidar_conf

            if ground_gamma > 0.0:
                self._h_ground_est_m = self._h_ground_est_m + ground_gamma * e_ground_m

            h_amsl_lidar_m = self._h_ground_est_m + self._h_agl_filt_m
            innovation_m = h_amsl_lidar_m - h_amsl_pred_m
            innovation_clamped_m = self._clip(
                innovation_m,
                -self.cfg.innovation_max_m,
                self.cfg.innovation_max_m,
            )
            self._h_amsl_est_m = h_amsl_pred_m + correction_gain * innovation_clamped_m
            self._last_valid_lidar_t_s = now_s
            self._h_agl_prev_m = h_agl_raw_m
        else:
            self._h_amsl_est_m = h_amsl_pred_m
            self._mode = self.MODE_LIDAR_INVALID

        self._h_gps_out_m = self._lpf(
            self._h_gps_out_m,
            self._h_amsl_est_m,
            self.cfg.gps_alt_lpf_alpha,
        )

        self._baro_rel_prev_filt_m = self._baro_rel_filt_m
        self._last_step_t_s = now_s

        return FusionOutput(
            timestamp_s=now_s,
            mode=self._mode,
            h_amsl_pred_m=h_amsl_pred_m,
            h_amsl_est_m=self._h_amsl_est_m,
            h_amsl_lidar_m=h_amsl_lidar_m,
            h_ground_est_m=self._h_ground_est_m,
            h_ground_meas_m=h_ground_meas_m,
            h_agl_filt_m=self._h_agl_filt_m,
            baro_rel_filt_m=self._baro_rel_filt_m,
            lidar_valid=lidar_valid,
            lidar_confidence=lidar_conf,
            correction_gain=correction_gain,
            ground_gamma=ground_gamma,
            innovation_m=innovation_m,
            innovation_clamped_m=innovation_clamped_m,
            validity_flags=validity_flags,
        )

    def gps_output_altitude_m(self) -> float:
        """Return smoothed AMSL altitude for GPS-like output stream."""
        return self._h_gps_out_m
