"""Median-flow VO loop matching the standalone OV9281 script."""

import time

import cv2
import numpy as np


class MedianFlowVO:
    """Visual odometry with median flow + forward-backward check."""

    def __init__(
        self,
        camera_driver,
        height_estimator,
        frame_size,
        focal_length_px,
        height_m,
        grid_rows=6,
        grid_cols=8,
        max_corners=300,
        quality_level=0.01,
        min_distance=7,
        fb_err_thresh=1.0,
        min_features=30,
        use_undistort=True,
        K=None,
        D=None,
        frame_delay_s=0.02,
        show_window=False,
        window_name="Median Flow VO",
    ):
        self.camera_driver = camera_driver
        self.height_estimator = height_estimator
        self.frame_size = frame_size
        self.focal_length_px = float(focal_length_px)
        self.height_m = float(height_m)
        self.grid_rows = int(grid_rows)
        self.grid_cols = int(grid_cols)
        self.max_corners = int(max_corners)
        self.quality_level = float(quality_level)
        self.min_distance = int(min_distance)
        self.fb_err_thresh = float(fb_err_thresh)
        self.min_features = int(min_features)
        self.use_undistort = bool(use_undistort)
        self.K = K
        self.D = D
        self.frame_delay_s = float(frame_delay_s)
        self.show_window = bool(show_window)
        self.window_name = str(window_name)

        self.x = 0.0
        self.y = 0.0
        self.z = self.height_m

        self._prev_gray = None
        self._prev_pts = None
        self._prev_time = None
        self._map1 = None
        self._map2 = None
        self._mask = None

    def _detect_features(self, gray):
        return cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
        )

    def _median_flow(self, prev_gray, gray, prev_pts):
        next_pts, st, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray, gray, prev_pts, None, winSize=(21, 21), maxLevel=3
        )
        if next_pts is None:
            return None, None, None, None

        back_pts, st_back, _ = cv2.calcOpticalFlowPyrLK(
            gray, prev_gray, next_pts, None, winSize=(21, 21), maxLevel=3
        )
        if back_pts is None:
            return None, None, None, None

        prev_good = prev_pts[st == 1]
        next_good = next_pts[st == 1]
        back_good = back_pts[st == 1]

        fb_err = np.linalg.norm(prev_good - back_good, axis=1)
        keep = fb_err < self.fb_err_thresh
        prev_good = prev_good[keep]
        next_good = next_good[keep]

        if len(prev_good) < self.min_features:
            return None, None, None, None

        flow = next_good - prev_good
        dx = float(np.median(flow[:, 0]))
        dy = float(np.median(flow[:, 1]))
        return dx, dy, prev_good, next_good

    def _draw_grid(self, frame):
        if self.grid_rows <= 1 and self.grid_cols <= 1:
            return
        height, width = frame.shape[:2]
        for row in range(1, self.grid_rows):
            y = int(row * height / self.grid_rows)
            cv2.line(frame, (0, y), (width, y), (60, 60, 60), 1)
        for col in range(1, self.grid_cols):
            x = int(col * width / self.grid_cols)
            cv2.line(frame, (x, 0), (x, height), (60, 60, 60), 1)

    def _prepare_gray(self, frame):
        if frame.ndim == 2:
            gray = frame
        elif frame.ndim == 3 and frame.shape[2] == 1:
            gray = frame[:, :, 0]
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if not self.use_undistort or self.K is None or self.D is None:
            return gray

        if self._map1 is None or self._map2 is None:
            new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self.K, self.D, self.frame_size, np.eye(3), balance=0.0
            )
            self._map1, self._map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, np.eye(3), new_k, self.frame_size, cv2.CV_16SC2
            )
        return cv2.remap(gray, self._map1, self._map2, interpolation=cv2.INTER_LINEAR)

    def run(self, on_update=None):
        ret, frame = self.camera_driver.read()
        if not ret:
            raise RuntimeError("Camera error: failed to read initial frame")

        gray = self._prepare_gray(frame)
        self._prev_gray = gray
        self._prev_pts = self._detect_features(gray)
        self._prev_time = time.time()

        while True:
            self.height_estimator.update()

            ret, frame = self.camera_driver.read()
            if not ret:
                time.sleep(self.frame_delay_s)
                continue

            gray = self._prepare_gray(frame)
            now = time.time()

            if self._prev_pts is None or len(self._prev_pts) < self.min_features:
                self._prev_pts = self._detect_features(self._prev_gray)
                self._prev_time = now
                self._prev_gray = gray
                self._mask = None
                continue

            dx_px, dy_px, prev_pts, next_pts = self._median_flow(
                self._prev_gray, gray, self._prev_pts
            )
            if dx_px is None:
                self._prev_gray = gray
                self._prev_pts = self._detect_features(self._prev_gray)
                self._prev_time = now
                self._mask = None
                continue

            dt = max(now - (self._prev_time or now), 1e-6)

            height = self.height_estimator.get_height_m()
            if height is None:
                height = self.height_m

            dx_m = (dx_px / self.focal_length_px) * height
            dy_m = (dy_px / self.focal_length_px) * height
            dz_m = 0.0

            self.x += dx_m
            self.y += dy_m
            self.z = height

            if on_update is not None:
                on_update(
                    self.x,
                    self.y,
                    self.z,
                    dx_m,
                    dy_m,
                    dz_m,
                    dx_px,
                    dy_px,
                    len(next_pts),
                    1.0,
                    0.0,
                )

            if self.show_window:
                vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                self._draw_grid(vis)
                self._mask = np.zeros_like(vis)
                for new, old in zip(next_pts[:50], prev_pts[:50]):
                    a, b = new.ravel()
                    c, d = old.ravel()
                    self._mask = cv2.line(
                        self._mask,
                        (int(a), int(b)),
                        (int(c), int(d)),
                        (0, 255, 255),
                        2,
                    )
                    vis = cv2.circle(vis, (int(a), int(b)), 3, (0, 0, 255), -1)
                vis = cv2.add(vis, self._mask)

                center = (self.frame_size[0] // 2, self.frame_size[1] // 2)
                arrow_end = (
                    int(center[0] + dy_px * 8),
                    int(center[1] + dx_px * 8),
                )
                cv2.arrowedLine(vis, center, arrow_end, (0, 255, 0), 2)
                cv2.putText(
                    vis,
                    f"x={self.x:.3f} y={self.y:.3f} z={self.z:.3f}",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    vis,
                    f"dx={dx_m:.4f} dy={dy_m:.4f} dz={dz_m:.4f}",
                    (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    vis,
                    f"vx={dx_m / dt:.3f} vy={dy_m / dt:.3f} vz={dz_m / dt:.3f}",
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2,
                )
                cv2.imshow(self.window_name, vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            self._prev_gray = gray
            self._prev_pts = next_pts.reshape(-1, 1, 2)
            self._prev_time = now

            time.sleep(self.frame_delay_s)

        self.camera_driver.release()
        if self.show_window:
            cv2.destroyAllWindows()
