"""Shared helpers for navigation output modes."""

from navisar.navigation.state_estimator import PositionSourceSelector


def enu_to_gps(x_m, y_m, z_m, origin):
    """Convert ENU meters into latitude/longitude/altitude."""
    lat, lon = PositionSourceSelector.local_to_ll(x_m, y_m, origin)
    alt_base = 0.0 if origin[2] is None else origin[2]
    alt_m = alt_base + (0.0 if z_m is None else z_m)
    return lat, lon, alt_m


class EnuVelocityTracker:
    """Track ENU position to estimate velocity."""
    def __init__(self):
        self._last = {"time": None, "x": None, "y": None, "z": None}

    def velocity_and_update(self, now, x_m, y_m, z_m):
        """Return ENU velocity and update the tracked state."""
        last_time = self._last["time"]
        if (
            last_time is None
            or now <= last_time
            or self._last["x"] is None
            or self._last["y"] is None
            or self._last["z"] is None
        ):
            vx, vy, vz = 0.0, 0.0, 0.0
        else:
            dt = now - last_time
            vx = (x_m - self._last["x"]) / dt
            vy = (y_m - self._last["y"]) / dt
            vz = (z_m - self._last["z"]) / dt
        self._last["time"] = now
        self._last["x"] = x_m
        self._last["y"] = y_m
        self._last["z"] = z_m
        return vx, vy, vz
