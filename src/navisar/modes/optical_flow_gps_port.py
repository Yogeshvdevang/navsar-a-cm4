"""Mode for sending optical flow data as GPS over serial."""

from navisar.modes.gps_port import GpsPortMode


def _finite(value):
    return value is not None and float(value) == float(value) and abs(float(value)) != float("inf")


class OpticalFlowGpsPortMode:
    """Integrate MTF-01 optical flow into ENU and emit GPS over serial."""

    def __init__(
        self,
        gps_port_mode: GpsPortMode,
        min_quality: int = 30,
        max_speed_mps: float = 2.0,
        deadband_mps: float = 0.05,
        smoothing_alpha: float = 0.2,
        stationary_speed_mps: float = 0.02,
        stationary_samples: int = 15,
        stationary_quality_min: int = 40,
        speed_scale: float = 1.0,
        warn_interval_s: float = 2.0,
        max_gap_s: float = 1.0,
    ):
        self.gps_port_mode = gps_port_mode
        self.min_quality = int(min_quality)
        self.max_speed_mps = float(max_speed_mps)
        self.deadband_mps = float(deadband_mps)
        self.smoothing_alpha = float(smoothing_alpha)
        self.stationary_speed_mps = float(stationary_speed_mps)
        self.stationary_samples = int(stationary_samples)
        self.stationary_quality_min = int(stationary_quality_min)
        self.speed_scale = float(speed_scale)
        self.warn_interval_s = float(warn_interval_s)
        self.max_gap_s = float(max_gap_s)
        self._last_warn = 0.0
        self._last_time_s = None
        self._last_time_ms = None
        self._x_m = 0.0
        self._y_m = 0.0
        self._z_m = 0.0
        self._vx_f = 0.0
        self._vy_f = 0.0
        self._stationary_count = 0
        self._origin = None
        self.last_payload = None

    def _warn(self, now, message):
        if now - self._last_warn >= self.warn_interval_s:
            print(message)
            self._last_warn = now

    def _reset(self, origin):
        self._last_time_s = None
        self._last_time_ms = None
        self._x_m = 0.0
        self._y_m = 0.0
        self._z_m = 0.0
        self._vx_f = 0.0
        self._vy_f = 0.0
        self._stationary_count = 0
        self._origin = origin

    def handle(
        self,
        now,
        sample,
        origin,
        alt_override_m=None,
        heading_deg=None,
        heading_only=False,
    ):
        """Integrate optical flow and send GPS sentences over serial."""
        if sample is None:
            self._warn(now, "OFLOW->PORT: waiting for optical flow samples...")
            return
        if origin is None:
            self._warn(now, "OFLOW->PORT: missing gps_origin in pixhawk.yaml.")
            return
        if self._origin != origin:
            self._reset(origin)

        dt_s = None
        if self._last_time_ms is not None and sample.time_ms is not None:
            dt_ms = sample.time_ms - self._last_time_ms
            if dt_ms > 0:
                dt_s = dt_ms / 1000.0
        if dt_s is None:
            if self._last_time_s is not None and now > self._last_time_s:
                dt_s = now - self._last_time_s
        if dt_s is None or dt_s <= 0.0:
            dt_s = 0.0

        if dt_s > self.max_gap_s:
            # Large gap, reset integration to avoid jumps.
            self._last_time_s = now
            self._last_time_ms = sample.time_ms
            return

        if sample.flow_ok and sample.dist_ok and dt_s > 0.0:
            quality = int(sample.flow_quality)
            if quality < self.min_quality:
                vx_mps, vy_mps = 0.0, 0.0
            else:
                scale = self.speed_scale
                vx_mps = float(sample.speed_x) * scale
                vy_mps = float(sample.speed_y) * scale
            if _finite(vx_mps) and _finite(vy_mps):
                if (
                    quality >= self.stationary_quality_min
                    and abs(vx_mps) <= self.stationary_speed_mps
                    and abs(vy_mps) <= self.stationary_speed_mps
                ):
                    self._stationary_count += 1
                else:
                    self._stationary_count = 0

                if self._stationary_count >= self.stationary_samples:
                    self._vx_f = 0.0
                    self._vy_f = 0.0
                else:
                    alpha = max(0.0, min(1.0, self.smoothing_alpha))
                    self._vx_f = alpha * vx_mps + (1.0 - alpha) * self._vx_f
                    self._vy_f = alpha * vy_mps + (1.0 - alpha) * self._vy_f
                    if abs(self._vx_f) < self.deadband_mps:
                        self._vx_f = 0.0
                    if abs(self._vy_f) < self.deadband_mps:
                        self._vy_f = 0.0
                    if abs(self._vx_f) > self.max_speed_mps:
                        self._vx_f = (
                            self.max_speed_mps if self._vx_f > 0 else -self.max_speed_mps
                        )
                    if abs(self._vy_f) > self.max_speed_mps:
                        self._vy_f = (
                            self.max_speed_mps if self._vy_f > 0 else -self.max_speed_mps
                        )
                self._x_m += self._vx_f * dt_s
                self._y_m += self._vy_f * dt_s
        else:
            self._vx_f = 0.0
            self._vy_f = 0.0
            self._stationary_count = 0

        self._last_time_s = now
        self._last_time_ms = sample.time_ms

        alt_m = alt_override_m
        if alt_m is not None and _finite(alt_m):
            self._z_m = float(alt_m)
        elif sample.dist_ok:
            self._z_m = float(sample.distance_mm) / 1000.0

        if alt_m is None and sample.dist_ok:
            alt_m = self._z_m

        self.gps_port_mode.handle(
            now,
            self._x_m,
            self._y_m,
            self._z_m,
            (origin[0], origin[1], None),
            alt_override_m=alt_m,
            heading_deg=heading_deg,
            heading_only=heading_only,
        )
        self.last_payload = self.gps_port_mode.last_payload
