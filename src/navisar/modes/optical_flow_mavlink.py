"""Mode for sending optical flow data via MAVLink."""


class OpticalFlowMavlinkMode:
    """Send MAVLink OPTICAL_FLOW_RAD from MTF-01 samples."""

    def __init__(
        self,
        send_interval_s,
        print_enabled=False,
        warn_interval_s=2.0,
        range_min_m=0.01,
        range_max_m=8.0,
    ):
        self.send_interval_s = float(send_interval_s)
        self.print_enabled = bool(print_enabled)
        self.warn_interval_s = float(warn_interval_s)
        self.range_min_m = float(range_min_m)
        self.range_max_m = float(range_max_m)
        self._last_send = 0.0
        self._last_warn = 0.0
        self._last_time_ms = None
        self.last_payload = None

    def _warn(self, now, message):
        if now - self._last_warn >= self.warn_interval_s:
            print(message)
            self._last_warn = now

    def handle(self, now, sample, mavlink_interface):
        """Send OPTICAL_FLOW_RAD if ready."""
        if mavlink_interface is None:
            self._warn(now, "OFLOW->MAV: MAVLink unavailable.")
            return
        if sample is None:
            self._warn(now, "OFLOW->MAV: waiting for optical flow samples...")
            return
        if now - self._last_send < self.send_interval_s:
            return

        dt_s = None
        if self._last_time_ms is not None and sample.time_ms is not None:
            dt_ms = sample.time_ms - self._last_time_ms
            if dt_ms > 0:
                dt_s = dt_ms / 1000.0
        if dt_s is None or dt_s <= 0.0:
            dt_s = self.send_interval_s if self.send_interval_s > 0 else 0.02

        self._last_time_ms = sample.time_ms

        integration_time_us = max(1, int(dt_s * 1_000_000))
        integrated_x = float(sample.flow_vx) * dt_s
        integrated_y = float(sample.flow_vy) * dt_s
        quality = int(max(0, min(255, sample.flow_quality)))
        distance_m = float(sample.distance_mm) / 1000.0 if sample.dist_ok else -1.0

        covariance = 255
        if distance_m > 0 and quality > 0:
            # Lower covariance means higher confidence for ArduPilot.
            covariance = int(max(0, min(50, 50 - (quality // 5))))

        payload_flow = mavlink_interface.send_optical_flow_rad(
            integrated_x=integrated_x,
            integrated_y=integrated_y,
            integration_time_us=integration_time_us,
            distance_m=distance_m,
            quality=quality,
        )
        payload_range = None
        if distance_m > 0:
            payload_range = mavlink_interface.send_distance_sensor(
                distance_m=distance_m,
                min_distance_m=self.range_min_m,
                max_distance_m=self.range_max_m,
                covariance=covariance,
            )

        self.last_payload = {
            "time_s": now,
            "time_ms": sample.time_ms,
            "flow_vx": sample.flow_vx,
            "flow_vy": sample.flow_vy,
            "integrated_x": integrated_x,
            "integrated_y": integrated_y,
            "dist_cm": sample.dist_cm,
            "distance_m": distance_m,
            "quality": quality,
            "payload": payload_flow,
            "range_payload": payload_range,
        }

        if self.print_enabled:
            print(
                "OFLOW->MAV: "
                f"dx={integrated_x:.4f} rad dy={integrated_y:.4f} rad "
                f"dist={distance_m:.2f} m q={quality}"
            )

        self._last_send = now
