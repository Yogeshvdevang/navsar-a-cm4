"""Median Flow module. Provides median flow utilities for NAVISAR."""

import numpy as np

from navisar.vps.algorithms.base import MotionEstimator


class MedianFlowEstimator(MotionEstimator):
    """Estimate motion using median optical flow."""
    def estimate(self, good_old, good_new, height_m, fx, fy, ransac_thresh):
        """Compute motion from median flow between point sets."""
        if len(good_old) == 0:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0

        flow = (good_new - good_old).reshape(-1, 2)
        dx_pixels = float(np.median(flow[:, 0]))
        dy_pixels = float(np.median(flow[:, 1]))

        flow_mag = np.hypot(flow[:, 0], flow[:, 1])
        if flow_mag.size:
            median_mag = float(np.median(flow_mag))
            flow_mad_px = float(np.median(np.abs(flow_mag - median_mag)))
        else:
            flow_mad_px = 0.0

        # Invert pixel flow to represent camera motion instead of image motion.
        dx_m = -(dx_pixels / fx) * height_m
        dy_m = -(dy_pixels / fy) * height_m
        dz_m = 0.0

        inlier_count = len(flow)
        inlier_ratio = 1.0

        return (
            dx_m,
            dy_m,
            dz_m,
            dx_pixels,
            dy_pixels,
            inlier_count,
            inlier_ratio,
            flow_mad_px,
        )
