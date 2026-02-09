"""GPS vs odometry selector with drift-based gating."""

import math
import time


class PositionSourceSelector:
    """Track GPS/odometry sources and select the best position."""
    def __init__(
        self,
        drift_threshold_m=5.0,
        gps_timeout_s=2.0,
        min_fix_type=3,
    ):
        """Configure drift thresholds and GPS availability rules."""
        self.drift_threshold_m = drift_threshold_m
        self.gps_timeout_s = gps_timeout_s
        self.min_fix_type = min_fix_type
        self._gps_origin = None
        self._gps_local = None
        self._gps_fix_type = None
        self._gps_time = None
        self._odom = None
        self._odom_time = None

    def update_gps(self, lat, lon, alt_m, fix_type, timestamp=None):
        """Update GPS position in local ENU coordinates."""
        if lat is None or lon is None:
            return
        if timestamp is None:
            timestamp = time.time()

        if self._gps_origin is None:
            # First valid fix anchors the local ENU frame.
            self._gps_origin = (lat, lon, alt_m)

        x_m, y_m = self._ll_to_local(lat, lon, self._gps_origin)
        z_m = 0.0 if alt_m is None else alt_m
        self._gps_local = (x_m, y_m, z_m)
        self._gps_fix_type = fix_type
        self._gps_time = timestamp

    def update_odometry(self, x_m, y_m, z_m, timestamp=None):
        """Update odometry position in local coordinates."""
        if timestamp is None:
            timestamp = time.time()
        self._odom = (x_m, y_m, z_m)
        self._odom_time = timestamp

    def gps_available(self, now=None):
        """Return True when a recent, valid GPS fix exists."""
        if now is None:
            now = time.time()
        if self._gps_time is None:
            return False
        if now - self._gps_time > self.gps_timeout_s:
            return False
        if self._gps_fix_type is not None and self._gps_fix_type < self.min_fix_type:
            return False
        return True

    def drift_m(self):
        """Compute drift between GPS and odometry, if available."""
        if self._gps_local is None or self._odom is None:
            return None
        dx = self._gps_local[0] - self._odom[0]
        dy = self._gps_local[1] - self._odom[1]
        dz = self._gps_local[2] - self._odom[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def current_source(self, now=None):
        """Pick the position source based on drift and freshness."""
        if not self.gps_available(now):
            return "odometry"
        drift = self.drift_m()
        # Prefer odometry when GPS drift exceeds the configured threshold.
        if drift is not None and drift > self.drift_threshold_m:
            return "odometry"
        return "gps"

    def get_position(self, now=None):
        """Return the current best position estimate."""
        source = self.current_source(now)
        if source == "gps" and self._gps_local is not None:
            return self._gps_local
        return self._odom

    def gps_origin(self):
        """Return the current GPS origin, if set."""
        return self._gps_origin

    def gps_local(self):
        """Return the latest GPS local position, if available."""
        return self._gps_local

    def gps_time(self):
        """Return the timestamp of the latest GPS fix, if available."""
        return self._gps_time

    def gps_fix_type(self):
        """Return the latest GPS fix type, if available."""
        return self._gps_fix_type

    def set_gps_origin(self, lat, lon, alt_m=None):
        """Set the origin for local ENU conversion."""
        if lat is None or lon is None:
            return
        self._gps_origin = (lat, lon, alt_m)

    @staticmethod
    def local_to_ll(x_m, y_m, origin):
        """Convert local ENU meters to latitude/longitude."""
        lat0, lon0, _alt0 = origin
        radius_m = 6378137.0
        lat0_rad = math.radians(lat0)
        dlat = y_m / radius_m
        dlon = x_m / (radius_m * math.cos(lat0_rad))
        lat = lat0 + math.degrees(dlat)
        lon = lon0 + math.degrees(dlon)
        return lat, lon

    @staticmethod
    def _ll_to_local(lat, lon, origin):
        """Convert latitude/longitude to local ENU meters."""
        lat0, lon0, _alt0 = origin
        radius_m = 6378137.0
        lat_rad = math.radians(lat)
        lat0_rad = math.radians(lat0)
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        x_m = dlon * radius_m * math.cos(lat0_rad)
        y_m = dlat * radius_m
        return x_m, y_m
