"""Feature detection and tracking with grid-based coverage control."""

import cv2
import numpy as np


class FeatureTracker:
    """Track visual features with grid-based coverage control."""
    def __init__(
        self,
        min_features=40,
        max_features=300,
        redetect_interval=10,
        ransac_reproj_thresh=3.0,
        grid_rows=6,
        grid_cols=8,
        per_cell_max_features=30,
        texture_threshold=12.0,
        quality_level=0.2,
    ):
        """Initialize tracking parameters and buffers."""
        self.min_features = min_features
        self.max_features = max_features
        self.redetect_interval = redetect_interval
        self.ransac_reproj_thresh = ransac_reproj_thresh
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.per_cell_max_features = per_cell_max_features
        self.texture_threshold = texture_threshold
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        self.feature_params = dict(
            maxCorners=per_cell_max_features,
            qualityLevel=quality_level,
            minDistance=7,
            blockSize=7,
        )
        self.prev_gray = None
        self.p0 = None
        self.frame_idx = 0
        self._colors = None
        self._grid_colors = None

    def _init_grid_colors(self):
        """Build per-cell colors for debug visualization."""
        total = self.grid_rows * self.grid_cols
        if self._grid_colors is not None and len(self._grid_colors) == total:
            return
        hues = np.linspace(0, 179, total, endpoint=False).astype(np.uint8)
        hsv = np.zeros((total, 1, 3), dtype=np.uint8)
        hsv[:, 0, 0] = hues
        hsv[:, 0, 1] = 200
        hsv[:, 0, 2] = 255
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR).reshape(total, 3)
        self._grid_colors = [tuple(int(c) for c in color) for color in bgr]

    @staticmethod
    def _cell_texture(roi):
        """Return a simple texture metric for a region."""
        grad_x = cv2.Sobel(roi, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(roi, cv2.CV_32F, 0, 1, ksize=3)
        magnitude = cv2.magnitude(grad_x, grad_y)
        return float(np.mean(magnitude))

    def _detect_features_grid(self, gray):
        """Detect features per grid cell for balanced coverage."""
        self._init_grid_colors()
        height, width = gray.shape[:2]
        cell_h = max(1, height // self.grid_rows)
        cell_w = max(1, width // self.grid_cols)

        points = []
        colors = []
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                x0 = col * cell_w
                y0 = row * cell_h
                x1 = width if col == self.grid_cols - 1 else x0 + cell_w
                y1 = height if row == self.grid_rows - 1 else y0 + cell_h
                roi = gray[y0:y1, x0:x1]
                if roi.size == 0:
                    continue
                if self.texture_threshold is not None:
                    if self._cell_texture(roi) < self.texture_threshold:
                        continue
                # Detect features per cell to keep spatial distribution balanced.
                features = cv2.goodFeaturesToTrack(roi, mask=None, **self.feature_params)
                if features is None:
                    continue
                features[:, 0, 0] += x0
                features[:, 0, 1] += y0
                points.append(features)
                cell_color = self._grid_colors[row * self.grid_cols + col]
                colors.extend([cell_color] * len(features))

        if not points:
            return None, None

        points = np.vstack(points)
        colors = np.array(colors, dtype=np.uint8)

        if len(points) > self.max_features:
            # Retain highest-response corners to respect the global feature cap.
            responses = cv2.cornerMinEigenVal(gray, blockSize=self.feature_params["blockSize"])
            coords = points.reshape(-1, 2)
            xs = np.clip(coords[:, 0].astype(int), 0, width - 1)
            ys = np.clip(coords[:, 1].astype(int), 0, height - 1)
            scores = responses[ys, xs]
            top_idx = np.argsort(scores)[-self.max_features :]
            points = coords[top_idx].reshape(-1, 1, 2)
            colors = colors[top_idx]

        return points, colors

    def initialize(self, gray):
        """Initialize tracking on the first frame."""
        self.prev_gray = gray
        self.p0, self._colors = self._detect_features_grid(self.prev_gray)

    def track(self, gray):
        """Track features into the next frame."""
        if self.prev_gray is None:
            self.initialize(gray)
            self.frame_idx += 1
            return None, None, False

        reset_mask = False
        if self.p0 is None or len(self.p0) < self.min_features or (
            self.frame_idx % self.redetect_interval == 0
        ):
            # Refresh features when count drops or on periodic re-detect.
            self.p0, self._colors = self._detect_features_grid(self.prev_gray)
            reset_mask = True
            if self.p0 is None:
                self.prev_gray = gray
                self.frame_idx += 1
                return None, None, reset_mask

        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.p0, None, **self.lk_params)
        if p1 is None:
            self.prev_gray = gray
            self.frame_idx += 1
            return None, None, reset_mask

        status = st.reshape(-1).astype(bool)
        good_old = self.p0[status]
        good_new = p1[status]
        colors = None
        if self._colors is not None:
            colors = self._colors[status]
        if len(good_old) < 8:
            self.prev_gray = gray
            self.frame_idx += 1
            return None, None, reset_mask

        if len(good_old) >= 6:
            # Cull outliers using a partial affine RANSAC fit.
            _, inliers = cv2.estimateAffinePartial2D(
                good_old,
                good_new,
                method=cv2.RANSAC,
                ransacReprojThreshold=self.ransac_reproj_thresh,
            )
            if inliers is not None:
                inlier_mask = inliers.ravel().astype(bool)
                good_old = good_old[inlier_mask]
                good_new = good_new[inlier_mask]
                if colors is not None:
                    colors = colors[inlier_mask]

        if len(good_old) < 8:
            self.prev_gray = gray
            self.frame_idx += 1
            return None, None, reset_mask

        self.prev_gray = gray
        self.p0 = good_new.reshape(-1, 1, 2)
        self._colors = colors
        self.frame_idx += 1
        return good_old, good_new, reset_mask

    def current_colors(self):
        """Return a copy of per-feature colors for display."""
        if self._colors is None:
            return None
        return self._colors.copy()
