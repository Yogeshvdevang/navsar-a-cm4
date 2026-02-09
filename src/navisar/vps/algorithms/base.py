"""Base module. Provides base utilities for NAVISAR."""

class MotionEstimator:
    """Interface for motion estimation algorithms."""
    def estimate(self, good_old, good_new, height_m, fx, fy, ransac_thresh):
        """Estimate motion given tracked feature points."""
        raise NotImplementedError
