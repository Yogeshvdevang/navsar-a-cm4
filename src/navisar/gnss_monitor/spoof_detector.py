"""Spoof Detector module. Provides spoof detector utilities for NAVISAR."""

from __future__ import annotations

import math
import time


class SpoofDetector:
    """Basic GNSS spoofing detector using drift and jump heuristics."""

    def __init__(
        self,
        drift_threshold_m: float = 5.0,
        max_speed_mps: float = 25.0,
        consecutive_required: int = 3,
        cooldown_s: float = 2.0,
        min_fix_type: int = 3,
        min_dt_s: float = 0.2,
    ) -> None:
        self.drift_threshold_m = drift_threshold_m
        self.max_speed_mps = max_speed_mps
        self.consecutive_required = max(1, int(consecutive_required))
        self.cooldown_s = max(0.0, float(cooldown_s))
        self.min_fix_type = min_fix_type
        self.min_dt_s = max(0.0, float(min_dt_s))
        self._last_gps_local = None
        self._last_gps_time = None
        self._drift_hits = 0
        self._speed_hits = 0
        self._last_alert_time = 0.0

    def update(
        self,
        gps_local,
        gps_time,
        gps_fix_type,
        drift_m,
        timestamp=None,
    ):
        """Return (spoofed, reason) when heuristics indicate spoofing."""
        if gps_local is None or gps_time is None:
            return False, ""
        if gps_fix_type is not None and gps_fix_type < self.min_fix_type:
            return False, ""
        if timestamp is None:
            timestamp = time.time()

        drift_flag = drift_m is not None and drift_m > self.drift_threshold_m
        if drift_flag:
            self._drift_hits += 1
        else:
            self._drift_hits = 0

        speed_flag = False
        if (
            self._last_gps_local is not None
            and self._last_gps_time is not None
            and gps_time > self._last_gps_time
        ):
            dt = gps_time - self._last_gps_time
            if dt >= self.min_dt_s:
                dx = gps_local[0] - self._last_gps_local[0]
                dy = gps_local[1] - self._last_gps_local[1]
                dz = gps_local[2] - self._last_gps_local[2]
                speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
                speed_flag = speed > self.max_speed_mps
        if speed_flag:
            self._speed_hits += 1
        else:
            self._speed_hits = 0

        self._last_gps_local = gps_local
        self._last_gps_time = gps_time

        if (
            self._drift_hits >= self.consecutive_required
            or self._speed_hits >= self.consecutive_required
        ):
            if timestamp - self._last_alert_time >= self.cooldown_s:
                self._last_alert_time = timestamp
                if self._speed_hits >= self.consecutive_required:
                    return True, "gps jump speed exceeds threshold"
                return True, "gps drift exceeds threshold"

        return False, ""
