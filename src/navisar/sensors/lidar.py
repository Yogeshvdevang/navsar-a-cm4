"""LiDAR module. Provides lidar utilities for NAVISAR."""

import time


class LidarHeightEstimator:
    """Track LiDAR height from MAVLink distance sensor messages."""
    def __init__(
        self,
        mavlink_interface,
        min_m=0.2,
        max_m=10.0,
        fallback_m=1.0,
        distance_divisor=100.0,
    ):
        """Configure valid range and conversion from sensor units."""
        self.mavlink_interface = mavlink_interface
        self.min_m = min_m
        self.max_m = max_m
        self.current_m = None
        self.raw_distance = None
        self.last_valid_m = fallback_m
        self.last_msg = None
        self.last_msg_time = None
        self.distance_divisor = distance_divisor

    def update(self):
        """Fetch the latest distance sensor message."""
        if self.mavlink_interface is None:
            return
        msg = self.mavlink_interface.recv_distance_sensor()
        if msg is None:
            return
        self.last_msg = msg
        self.last_msg_time = time.time()
        self.raw_distance = msg.current_distance
        height_m = msg.current_distance / self.distance_divisor
        self.current_m = height_m
        if self.min_m < height_m < self.max_m:
            self.last_valid_m = height_m

    def get_height_m(self):
        """Return the best-known height estimate."""
        # Prefer the most recent raw reading when available, even if out of range.
        if self.current_m is not None:
            return self.current_m
        return self.last_valid_m
