"""Height Estimator module. Provides height estimator utilities for NAVISAR."""

class HeightEstimator:
    """Expose height from barometer or fallback constant."""
    def __init__(self, use_barometer=True, fallback_m=1.0, barometer_driver=None):
        """Configure barometer usage and fallback height."""
        self.use_barometer = use_barometer
        self.fallback_m = fallback_m
        self.barometer_driver = barometer_driver

    def update(self):
        """Poll the barometer driver when enabled."""
        if self.use_barometer and self.barometer_driver is not None:
            self.barometer_driver.update()

    def get_height_m(self):
        """Return the current height estimate."""
        if self.use_barometer and self.barometer_driver is not None:
            return self.barometer_driver.get_height_m()
        return self.fallback_m
