"""Adapter that delegates pose estimation to the selected algorithm."""

from navisar.vps.algorithms.ransac_affine import RansacAffineEstimator


class PoseEstimator:
    """Route feature tracks through a motion estimator."""
    def __init__(self, fx, fy, K, ransac_thresh=1.0, algorithm=None):
        """Store camera intrinsics and selected algorithm."""
        self.fx = fx
        self.fy = fy
        self.K = K
        self.ransac_thresh = ransac_thresh
        self.algorithm = algorithm or RansacAffineEstimator()

    def estimate(self, good_old, good_new, height_m):
        """Estimate motion given tracked points and height."""
        return self.algorithm.estimate(
            good_old,
            good_new,
            height_m,
            self.fx,
            self.fy,
            self.ransac_thresh,
        )
