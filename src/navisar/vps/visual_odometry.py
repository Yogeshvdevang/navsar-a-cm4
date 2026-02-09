"""Visual odometry loop with motion gating and optional yaw compensation."""

import cv2
import numpy as np
import time
from collections import deque


class VisualOdometry:
    """Run a monocular VO pipeline with optional gating and yaw."""
    def __init__(
        self,
        camera_driver,
        feature_tracker,
        pose_estimator,
        height_estimator,
        dist_coeffs=None,
        metric_threshold=0.02,
        frame_delay_s=0.02,
        img_width=640,
        img_height=480,
        yaw_provider=None,
        min_flow_px=0.4,
        min_height_m=0.1,
        exposure_min_mean=10.0,
        exposure_max_mean=245.0,
        zero_motion_window=8,
        zero_motion_mean_m=0.004,
        zero_motion_std_m=0.002,
        motion_deadband_m=0.003,
        motion_gate_enabled=True,
        min_inlier_ratio=0.5,
        max_flow_mad_px=1.2,
    ):
        """Configure VO components, thresholds, and smoothing."""
        self.camera_driver = camera_driver
        self.feature_tracker = feature_tracker
        self.pose_estimator = pose_estimator
        self.height_estimator = height_estimator
        self.metric_threshold = metric_threshold
        self.frame_delay_s = frame_delay_s
        self.img_width = img_width
        self.img_height = img_height
        self.yaw_provider = yaw_provider
        self.min_flow_px = min_flow_px
        self.min_height_m = min_height_m
        self.exposure_min_mean = exposure_min_mean
        self.exposure_max_mean = exposure_max_mean
        self.min_inlier_ratio = min_inlier_ratio
        self.max_flow_mad_px = max_flow_mad_px
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.zero_motion_window = zero_motion_window
        self.zero_motion_mean_m = zero_motion_mean_m
        self.zero_motion_std_m = zero_motion_std_m
        self.motion_deadband_m = motion_deadband_m
        self.motion_gate_enabled = motion_gate_enabled
        self.min_inliers = 30
        self.motion_confirm_frames = 3
        self.motion_window = 5
        self._motion_streak = 0
        self._dx_hist = deque(maxlen=self.motion_window)
        self._dy_hist = deque(maxlen=self.motion_window)
        self._zero_motion_hist = deque(maxlen=self.zero_motion_window)
        self._last_yaw = None
        self._last_yaw_time = None
        self.debug_enabled = False
        self.debug_interval_s = 0.5
        self._last_debug_time = 0.0
        self.dist_coeffs = None
        self._undistort_map = None
        if dist_coeffs is not None:
            dist = np.array(dist_coeffs, dtype=np.float64).ravel()
            if dist.size > 0 and not np.allclose(dist, 0.0):
                map1, map2 = cv2.initUndistortRectifyMap(
                    self.pose_estimator.K,
                    dist,
                    None,
                    self.pose_estimator.K,
                    (int(self.img_width), int(self.img_height)),
                    cv2.CV_16SC2,
                )
                self.dist_coeffs = dist
                self._undistort_map = (map1, map2)

    def _undistort(self, frame):
        """Undistort a frame using cached remap when available."""
        if self._undistort_map is None:
            return frame
        if frame.shape[1] != self.img_width or frame.shape[0] != self.img_height:
            return cv2.undistort(frame, self.pose_estimator.K, self.dist_coeffs)
        map1, map2 = self._undistort_map
        return cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

    @staticmethod
    def _ensure_bgr(frame):
        """Ensure a 3-channel BGR frame for display."""
        if frame.ndim == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.ndim == 3 and frame.shape[2] == 1:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame

    def _prepare_gray(self, frame):
        """Convert a frame to grayscale and undistort if needed."""
        if frame.ndim == 2:
            gray = frame
        elif frame.ndim == 3 and frame.shape[2] == 1:
            gray = frame[:, :, 0]
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.dist_coeffs is not None:
            # Remove lens distortion to improve tracking consistency.
            gray = self._undistort(gray)
        return gray

    def _prepare_display(self, frame):
        """Prepare a display frame with undistortion and BGR."""
        if self.dist_coeffs is not None:
            frame = self._undistort(frame)
        return self._ensure_bgr(frame)

    @staticmethod
    def _wrap_angle(angle_rad):
        """Wrap angle to [-pi, pi)."""
        return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi

    def _compensate_yaw(self, points, yaw_delta):
        """Rotate feature points to subtract yaw-induced motion."""
        if points is None or len(points) == 0:
            return points
        # Rotate features around image center to subtract yaw-induced flow.
        center = np.array([self.img_width / 2.0, self.img_height / 2.0], dtype=np.float32)
        cos_yaw = float(np.cos(-yaw_delta))
        sin_yaw = float(np.sin(-yaw_delta))
        pts = points.reshape(-1, 2).astype(np.float32) - center
        rot = np.empty_like(pts)
        rot[:, 0] = pts[:, 0] * cos_yaw - pts[:, 1] * sin_yaw
        rot[:, 1] = pts[:, 0] * sin_yaw + pts[:, 1] * cos_yaw
        rot += center
        return rot.reshape(-1, 1, 2)

    def _direction_from_motion(self, dx_m, dy_m):
        """Convert motion deltas into a coarse direction label."""
        direction = ""
        if abs(dx_m) > self.metric_threshold or abs(dy_m) > self.metric_threshold:
            if abs(dx_m) > abs(dy_m):
                direction = "RIGHT" if dx_m > 0 else "LEFT"
            else:
                direction = "UP" if dy_m > 0 else "DOWN"
        return direction

    def _draw_grid(self, frame):
        """Draw the tracking grid overlay."""
        rows = getattr(self.feature_tracker, "grid_rows", None)
        cols = getattr(self.feature_tracker, "grid_cols", None)
        if rows is None or cols is None or rows <= 1 and cols <= 1:
            return
        height, width = frame.shape[:2]
        for row in range(1, rows):
            y = int(row * height / rows)
            cv2.line(frame, (0, y), (width, y), (60, 60, 60), 1)
        for col in range(1, cols):
            x = int(col * width / cols)
            cv2.line(frame, (x, 0), (x, height), (60, 60, 60), 1)

    def run(self, window_name="VO + Barometer", on_update=None):
        """Run the VO loop, optionally emitting updates via callback."""
        ret, prev_frame = self.camera_driver.read()
        if not ret:
            raise RuntimeError("Camera error: failed to read initial frame")

        prev_gray = self._prepare_gray(prev_frame)
        self.feature_tracker.initialize(prev_gray)
        display_frame = self._prepare_display(prev_frame)
        mask = np.zeros_like(display_frame)

        while True:
            self.height_estimator.update()

            ret, frame = self.camera_driver.read()
            if not ret:
                continue

            gray = self._prepare_gray(frame)
            display_frame = self._prepare_display(frame)
            yaw_delta = 0.0
            if self.yaw_provider is not None:
                yaw_data = self.yaw_provider()
                if yaw_data is not None:
                    now_s = yaw_data.get("time_s", time.time())
                    yaw = yaw_data.get("yaw")
                    if yaw is not None:
                        if self._last_yaw is not None:
                            yaw_delta = self._wrap_angle(yaw - self._last_yaw)
                        self._last_yaw = yaw
                        self._last_yaw_time = now_s
                    else:
                        yaw_rate = yaw_data.get("yaw_rate")
                        if yaw_rate is not None and self._last_yaw_time is not None:
                            dt = max(0.0, now_s - self._last_yaw_time)
                            yaw_delta = float(yaw_rate) * dt
                            self._last_yaw_time = now_s

            good_old, good_new, reset_mask = self.feature_tracker.track(gray)
            if reset_mask:
                mask = np.zeros_like(display_frame)

            if good_old is None or good_new is None:
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                time.sleep(self.frame_delay_s)
                continue

            height = self.height_estimator.get_height_m()
            if height is None:
                height = self.z if self.z > 0.0 else self.min_height_m
            if yaw_delta != 0.0:
                good_new = self._compensate_yaw(good_new, yaw_delta)
            (
                dx_m,
                dy_m,
                dz_m,
                dx_pixels,
                dy_pixels,
                inlier_count,
                inlier_ratio,
                flow_mad_px,
            ) = self.pose_estimator.estimate(good_old, good_new, height)
            if self.motion_gate_enabled:
                # Gate integration to reduce drift when the camera is static.
                flow_mag_px = float(np.hypot(dx_pixels, dy_pixels))
                height_valid = height is not None and height >= self.min_height_m
                mean_intensity = float(np.mean(gray))
                exposure_ok = self.exposure_min_mean <= mean_intensity <= self.exposure_max_mean
                motion_detected = (
                    abs(dx_m) >= self.metric_threshold or abs(dy_m) >= self.metric_threshold
                )
                low_inliers = inlier_count < self.min_inliers
                low_ratio = inlier_ratio < self.min_inlier_ratio
                zero_motion_reject = False
                if (
                    (low_inliers and low_ratio)
                    or flow_mad_px > self.max_flow_mad_px
                    or flow_mag_px < self.min_flow_px
                    or not height_valid
                    or not exposure_ok
                ):
                    motion_detected = False

                step_m = float(np.hypot(dx_m, dy_m))
                if self.zero_motion_window != self._zero_motion_hist.maxlen:
                    self._zero_motion_hist = deque(
                        self._zero_motion_hist, maxlen=self.zero_motion_window
                    )
                self._zero_motion_hist.append(step_m)
                if len(self._zero_motion_hist) == self._zero_motion_hist.maxlen:
                    mean_step = float(np.mean(self._zero_motion_hist))
                    std_step = float(np.std(self._zero_motion_hist))
                    if mean_step < self.zero_motion_mean_m and std_step < self.zero_motion_std_m:
                        motion_detected = False
                        zero_motion_reject = True

                if self.debug_enabled:
                    now_s = time.time()
                    if now_s - self._last_debug_time >= self.debug_interval_s:
                        print(
                            "VO GATE: "
                            f"motion={motion_detected} inliers={inlier_count} ratio={inlier_ratio:.2f} "
                            f"flow_px={flow_mag_px:.2f} flow_mad={flow_mad_px:.2f} "
                            f"height={height:.2f} exposure={mean_intensity:.1f} "
                            f"low_inliers={low_inliers} low_ratio={low_ratio} "
                            f"height_ok={height_valid} exposure_ok={exposure_ok} "
                            f"zero_motion={zero_motion_reject}"
                        )
                        self._last_debug_time = now_s

                if motion_detected:
                    self._motion_streak += 1
                else:
                    self._motion_streak = 0
                    self._dx_hist.clear()
                    self._dy_hist.clear()
                    mask = np.zeros_like(display_frame)

                if (
                    self.motion_window != self._dx_hist.maxlen
                    or self.motion_window != self._dy_hist.maxlen
                ):
                    self._dx_hist = deque(self._dx_hist, maxlen=self.motion_window)
                    self._dy_hist = deque(self._dy_hist, maxlen=self.motion_window)

                if self._motion_streak >= self.motion_confirm_frames:
                    # Smooth motion estimates once we've confirmed real movement.
                    self._dx_hist.append(dx_m)
                    self._dy_hist.append(dy_m)
                    dx_m = float(np.mean(self._dx_hist))
                    dy_m = float(np.mean(self._dy_hist))
                else:
                    dx_m = 0.0
                    dy_m = 0.0
            else:
                if (
                    self.motion_window != self._dx_hist.maxlen
                    or self.motion_window != self._dy_hist.maxlen
                ):
                    self._dx_hist = deque(self._dx_hist, maxlen=self.motion_window)
                    self._dy_hist = deque(self._dy_hist, maxlen=self.motion_window)
                self._dx_hist.append(dx_m)
                self._dy_hist.append(dy_m)
                dx_m = float(np.mean(self._dx_hist))
                dy_m = float(np.mean(self._dy_hist))

            if self.motion_deadband_m > 0.0:
                step_m = float(np.hypot(dx_m, dy_m))
                if step_m < self.motion_deadband_m:
                    dx_m = 0.0
                    dy_m = 0.0

            self.z = height
            self.x += dx_m
            self.y += dy_m
            if on_update is not None:
                on_update(
                    self.x,
                    self.y,
                    self.z,
                    dx_m,
                    dy_m,
                    dz_m,
                    dx_pixels,
                    dy_pixels,
                    inlier_count,
                    inlier_ratio,
                    flow_mad_px,
                )

            self._draw_grid(display_frame)
            for idx, (new, old) in enumerate(zip(good_new[:50], good_old[:50])):
                a, b = new.ravel()
                c, d = old.ravel()
                line_color = (0, 0, 255)
                dot_color = (0, 255, 255)
                mask = cv2.line(
                    mask,
                    (int(a), int(b)),
                    (int(c), int(d)),
                    line_color,
                    2,
                )
                display_frame = cv2.circle(
                    display_frame, (int(a), int(b)), 3, dot_color, -1
                )

            direction = self._direction_from_motion(dx_m, dy_m)
            if direction == "":
                direction = self._direction_from_motion(dx_pixels, dy_pixels)
            step_m = float(np.hypot(dx_m, dy_m))
            move_label = "HOLD" if direction == "" else f"{direction} {step_m:.2f}m"
            center = (int(self.img_width / 2), int(self.img_height / 2))
            # Invert pixel flow so the arrow reflects camera motion direction.
            arrow_end = (int(center[0] - dx_pixels * 8), int(center[1] - dy_pixels * 8))
            cv2.arrowedLine(display_frame, center, arrow_end, (0, 255, 0), 2)

            cv2.putText(
                display_frame,
                f"MOVE: {move_label}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                display_frame,
                f"dx_pix: {dx_pixels:.2f} dy_pix: {dy_pixels:.2f}",
                (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                display_frame,
                f"X: {self.x:.2f} Y: {self.y:.2f} Z: {self.z:.2f}",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
            )
            cv2.putText(
                display_frame,
                f"dX: {dx_m:.3f} dY: {dy_m:.3f} dZ: {dz_m:.3f}",
                (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
            )

            img = cv2.add(display_frame, mask)
            cv2.imshow(window_name, img)

            if cv2.waitKey(1) & 0xFF == 27:
                break

            time.sleep(self.frame_delay_s)

        self.camera_driver.release()
        cv2.destroyAllWindows()
