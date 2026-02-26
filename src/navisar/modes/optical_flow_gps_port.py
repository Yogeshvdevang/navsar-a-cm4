"""Mode for sending optical flow data as GPS over serial."""

import math

from navisar.modes.gps_port import GpsPortMode


def _finite(value):
    return value is not None and float(value) == float(value) and abs(float(value)) != float("inf")


def _clamp(value, low, high):
    return max(low, min(high, value))


class OpticalFlowGpsPortMode:
    """Integrate MTF-01 optical flow into ENU and emit GPS over serial."""

    def __init__(
        self,
        gps_port_mode: GpsPortMode,
        min_quality: int = 50,
        max_speed_mps: float = 2.0,
        deadband_mps: float = 0.01,
        smoothing_alpha: float = 0.2,
        stationary_speed_mps: float = 0.02,
        stationary_samples: int = 15,
        stationary_quality_min: int = 40,
        speed_scale: float = 1.0,
        altitude_smoothing_alpha: float = 0.18,
        altitude_jump_limit_m: float = 0.06,
        altitude_deadband_m: float = 0.004,
        altitude_min_m: float = 0.05,
        altitude_max_m: float = 8.0,
        warn_interval_s: float = 2.0,
        max_gap_s: float = 1.0,
        print_enabled: bool = True,
        lat_scale: float = 1.0,
        lon_scale: float = 1.0,
        alt_offset_m: float = 0.0,
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
        self.altitude_smoothing_alpha = float(altitude_smoothing_alpha)
        self.altitude_jump_limit_m = float(altitude_jump_limit_m)
        self.altitude_deadband_m = float(altitude_deadband_m)
        self.altitude_min_m = float(altitude_min_m)
        self.altitude_max_m = float(altitude_max_m)
        self.warn_interval_s = float(warn_interval_s)
        self.max_gap_s = float(max_gap_s)
        self.print_enabled = bool(print_enabled)
        self.lat_scale = float(lat_scale)
        self.lon_scale = float(lon_scale)
        self.alt_offset_m = float(alt_offset_m)
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
        self._alt_filtered_m = None
        self.last_payload = None

    def set_gps_calibration(self, lat_scale=None, lon_scale=None, alt_offset_m=None):
        """Update runtime calibration used for GPS output mapping."""
        if lat_scale is not None:
            self.lat_scale = float(lat_scale)
        if lon_scale is not None:
            self.lon_scale = float(lon_scale)
        if alt_offset_m is not None:
            self.alt_offset_m = float(alt_offset_m)

    def _warn(self, now, message):
        if now - self._last_warn >= self.warn_interval_s:
            print(message)
            self._last_warn = now

    @staticmethod
    def _body_to_enu(vx_body_mps, vy_body_mps, heading_deg):
        """Rotate body-frame XY speed into ENU (east, north) using heading."""
        psi = math.radians(float(heading_deg) % 360.0)
        v_north = vx_body_mps * math.cos(psi) - vy_body_mps * math.sin(psi)
        v_east = vx_body_mps * math.sin(psi) + vy_body_mps * math.cos(psi)
        return v_east, v_north

    def _reset(self, origin):
        self._last_time_s = None
        self._last_time_ms = None
        self._x_m = 0.0
        self._y_m = 0.0
        self._z_m = 0.0
        self._vx_f = 0.0
        self._vy_f = 0.0
        self._stationary_count = 0
        self._alt_filtered_m = None
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
                vx_body_mps = float(sample.speed_x) * scale
                vy_body_mps = float(sample.speed_y) * scale
                if _finite(heading_deg):
                    vx_mps, vy_mps = self._body_to_enu(
                        vx_body_mps,
                        vy_body_mps,
                        float(heading_deg),
                    )
                else:
                    # Optical-flow-only fallback: treat sensor XY as ENU directly.
                    vx_mps, vy_mps = vx_body_mps, vy_body_mps
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

        raw_alt_m = None
        raw_dist_mm = None
        if alt_override_m is not None and _finite(alt_override_m):
            raw_alt_m = float(alt_override_m)
        elif sample.dist_ok:
            raw_alt_m = float(sample.distance_mm) / 1000.0
            raw_dist_mm = int(sample.distance_mm)

        if raw_alt_m is not None and _finite(raw_alt_m):
            if alt_override_m is not None and _finite(alt_override_m):
                # Caller provided final altitude value (offset + sensor distance).
                self._z_m = raw_alt_m
            else:
                raw_alt_m = _clamp(raw_alt_m, self.altitude_min_m, self.altitude_max_m)
                if self._alt_filtered_m is None:
                    self._alt_filtered_m = raw_alt_m
                else:
                    jump_limit = max(0.0, self.altitude_jump_limit_m)
                    if jump_limit > 0.0:
                        delta = raw_alt_m - self._alt_filtered_m
                        if abs(delta) > jump_limit:
                            raw_alt_m = self._alt_filtered_m + (
                                jump_limit if delta > 0 else -jump_limit
                            )
                    alpha = _clamp(self.altitude_smoothing_alpha, 0.0, 1.0)
                    filtered_alt = alpha * raw_alt_m + (1.0 - alpha) * self._alt_filtered_m
                    if abs(filtered_alt - self._alt_filtered_m) < self.altitude_deadband_m:
                        filtered_alt = self._alt_filtered_m
                    self._alt_filtered_m = filtered_alt
                self._z_m = self._alt_filtered_m

        alt_m = self._z_m if _finite(self._z_m) else None

        # Calibrate optical ENU before converting to GPS.
        x_out_m = self._x_m * self.lon_scale
        y_out_m = self._y_m * self.lat_scale
        z_out_m = self._z_m + self.alt_offset_m
        alt_out_m = alt_m + self.alt_offset_m if alt_m is not None else None

        self.gps_port_mode.handle(
            now,
            x_out_m,
            y_out_m,
            z_out_m,
            (origin[0], origin[1], None),
            alt_override_m=alt_out_m,
            nav_pvt_alt_mm_override=raw_dist_mm,
            heading_deg=heading_deg,
            heading_only=heading_only,
            apply_final_altitude_offset=False,
        )
        port_payload = self.gps_port_mode.last_payload
        if port_payload is None:
            self.last_payload = None
            return

        flow_payload = {
            "time_ms": sample.time_ms,
            "distance_mm": sample.distance_mm,
            "flow_vx": sample.flow_vx,
            "flow_vy": sample.flow_vy,
            "flow_quality": int(sample.flow_quality),
            "flow_ok": int(sample.flow_ok),
            "dist_ok": int(sample.dist_ok),
            "speed_x_mps_raw": float(sample.speed_x),
            "speed_y_mps_raw": float(sample.speed_y),
            "speed_x_mps_used": float(self._vx_f),
            "speed_y_mps_used": float(self._vy_f),
            "x_m": float(self._x_m),
            "y_m": float(self._y_m),
            "z_m": float(self._z_m),
            "dt_s": float(dt_s),
        }

        payload = dict(port_payload)
        payload["optical_flow"] = flow_payload
        payload["calibration"] = {
            "lat_scale": float(self.lat_scale),
            "lon_scale": float(self.lon_scale),
            "alt_offset_m": float(self.alt_offset_m),
            "x_out_m": float(x_out_m),
            "y_out_m": float(y_out_m),
            "z_out_m": float(z_out_m),
        }
        self.last_payload = payload

        ubx_payload = payload.get("ubx") or {}
        if self.print_enabled and ubx_payload.get("pvt_hex"):
            print(
                "OFLOW->UBX:\n"
                f"time_ms={flow_payload['time_ms']} dt_s={flow_payload['dt_s']:.3f} "
                f"dist_mm={flow_payload['distance_mm']} dist_ok={flow_payload['dist_ok']} "
                f"flow_ok={flow_payload['flow_ok']} flow_q={flow_payload['flow_quality']}\n"
                f"raw_vx={flow_payload['flow_vx']} raw_vy={flow_payload['flow_vy']} "
                f"raw_speed_x={flow_payload['speed_x_mps_raw']:.3f} "
                f"raw_speed_y={flow_payload['speed_y_mps_raw']:.3f}\n"
                f"used_vx={flow_payload['speed_x_mps_used']:.3f} "
                f"used_vy={flow_payload['speed_y_mps_used']:.3f} "
                f"enu=({flow_payload['x_m']:.3f}, {flow_payload['y_m']:.3f}, {flow_payload['z_m']:.3f})"
            )
            print(f"UBX RAW NAV-PVT: {ubx_payload.get('pvt_hex')}")
            print(f"UBX RAW NAV-POSLLH: {ubx_payload.get('posllh_hex')}")
            print(f"UBX RAW NAV-VELNED: {ubx_payload.get('velned_hex')}")
            print(f"UBX RAW NAV-SOL: {ubx_payload.get('sol_hex')}")
            print(f"UBX RAW NAV-STATUS: {ubx_payload.get('status_hex')}")
            print(f"UBX RAW NAV-DOP: {ubx_payload.get('dop_hex')}")
            print("-" * 50)
