"""Barometer module. Provides barometer utilities for NAVISAR."""

import math
import time


class BarometerHeightEstimator:
    """Track height from MAVLink barometer/altitude messages."""
    def __init__(self, mavlink_interface, fallback_m=1.0):
        self.mavlink_interface = mavlink_interface
        self.fallback_m = float(fallback_m)
        self.current_m = None
        self.raw_press_hpa = None
        self.raw_alt_m = None
        self.last_valid_m = self.fallback_m
        self.last_msg_time = None
        self._base_alt_m = None

    @staticmethod
    def _pressure_to_alt_m(press_hpa):
        # Standard atmosphere approximation.
        return 44330.0 * (1.0 - (press_hpa / 1013.25) ** 0.1903)

    def update(self):
        """Fetch the latest barometer message."""
        if self.mavlink_interface is None:
            return
        msg = self.mavlink_interface.recv_barometer()
        if msg is None:
            return
        self.last_msg_time = time.time()
        press_hpa = msg.get("press_hpa")
        alt_m = msg.get("alt_m")
        if press_hpa is not None:
            self.raw_press_hpa = press_hpa
            alt_m = self._pressure_to_alt_m(press_hpa)
        if alt_m is None:
            return
        self.raw_alt_m = alt_m
        if self._base_alt_m is None:
            self._base_alt_m = alt_m
        height_m = alt_m - self._base_alt_m
        if math.isfinite(height_m):
            self.current_m = height_m
            self.last_valid_m = height_m

    def get_height_m(self):
        """Return the best-known height estimate."""
        if self.current_m is not None:
            return self.current_m
        return self.last_valid_m
