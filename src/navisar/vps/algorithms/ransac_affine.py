"""Ransac Affine module. Provides ransac affine utilities for NAVISAR."""

import cv2
import numpy as np

from navisar.vps.algorithms.base import MotionEstimator


class RansacAffineEstimator(MotionEstimator):
    """Estimate translation using RANSAC affine fits."""
    def __init__(self, confidence=0.999, refine_iters=10):
        """Configure RANSAC confidence and refinement."""
        self.confidence = confidence
        self.refine_iters = refine_iters

    def _ransac_translation(self, good_old, good_new, ransac_thresh):
        """Estimate affine translation and return inlier mask."""
        model, inliers = cv2.estimateAffinePartial2D(
            good_old,
            good_new,
            method=cv2.RANSAC,
            ransacReprojThreshold=ransac_thresh,
            confidence=self.confidence,
            refineIters=self.refine_iters,
        )
        if inliers is None:
            return None, None
        inlier_mask = inliers.ravel().astype(bool)
        if model is None or not np.any(inlier_mask):
            return None, None
        return model, inlier_mask

    def estimate(self, good_old, good_new, height_m, fx, fy, ransac_thresh):
        """Estimate motion using a RANSAC translation model."""
        total = len(good_old)
        model, inlier_mask = self._ransac_translation(good_old, good_new, ransac_thresh)
        if inlier_mask is not None:
            good_old_use = good_old[inlier_mask]
            good_new_use = good_new[inlier_mask]
            inlier_count = len(good_old_use)
        else:
            good_old_use = good_old
            good_new_use = good_new
            inlier_count = 0

        if len(good_old_use) == 0:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0

        flow = (good_new_use - good_old_use).reshape(-1, 2)
        dx_pixels = float(np.median(flow[:, 0]))
        dy_pixels = float(np.median(flow[:, 1]))

        if model is not None:
            dx_pixels = float(model[0, 2])
            dy_pixels = float(model[1, 2])

        flow_mag = np.hypot(flow[:, 0], flow[:, 1])
        if flow_mag.size:
            median_mag = float(np.median(flow_mag))
            flow_mad_px = float(np.median(np.abs(flow_mag - median_mag)))
        else:
            flow_mad_px = 0.0

        inlier_ratio = float(inlier_count) / float(max(1, total))

        # Invert pixel flow to represent camera motion instead of image motion.
        dx_m = -(dx_pixels / fx) * height_m
        dy_m = -(dy_pixels / fy) * height_m
        dz_m = 0.0

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
