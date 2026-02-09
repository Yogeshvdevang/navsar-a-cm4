"""Visual SLAM module using a secondary camera feed."""

from __future__ import annotations

import time
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class SlamConfig:
    """Configuration for the Visual SLAM runner."""

    max_features: int = 800
    draw_matches: int = 120
    motion_scale: float = 1.0
    trajectory_size: int = 600
    trajectory_scale: float = 50.0
    frame_delay_s: float = 0.01


class VisualSlam:
    """Simple monocular visual SLAM visualization for a secondary camera."""

    def __init__(
        self,
        camera_driver,
        intrinsics,
        dist_coeffs=None,
        config: SlamConfig | None = None,
    ) -> None:
        self.camera_driver = camera_driver
        self.k = intrinsics
        self.dist_coeffs = dist_coeffs
        self.config = config or SlamConfig()
        self._rotation = np.eye(3, dtype=np.float64)
        self._translation = np.zeros((3, 1), dtype=np.float64)
        self._traj = np.zeros(
            (self.config.trajectory_size, self.config.trajectory_size, 3),
            dtype=np.uint8,
        )
        self._orb = cv2.ORB_create(nfeatures=self.config.max_features)
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self._stop_requested = False

    def stop(self) -> None:
        """Signal the SLAM loop to stop."""
        self._stop_requested = True

    def _prepare_gray(self, frame):
        if frame.ndim == 2:
            return frame
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _draw_trajectory(self, window_name):
        center = self.config.trajectory_size // 2
        x = int(self._translation[0, 0] * self.config.trajectory_scale) + center
        y = int(self._translation[2, 0] * self.config.trajectory_scale) + center
        if 0 <= x < self._traj.shape[1] and 0 <= y < self._traj.shape[0]:
            cv2.circle(self._traj, (x, y), 1, (0, 0, 255), 2)
        cv2.imshow(window_name, self._traj)

    def run(self, window_name="Visual SLAM", trajectory_window="SLAM Trajectory"):
        """Run the SLAM visualization loop."""
        ret, prev_frame = self.camera_driver.read()
        if not ret:
            print("SLAM camera error: failed to read initial frame")
            return

        prev_gray = self._prepare_gray(prev_frame)
        prev_kp, prev_des = self._orb.detectAndCompute(prev_gray, None)

        while not self._stop_requested:
            ret, frame = self.camera_driver.read()
            if not ret:
                time.sleep(self.config.frame_delay_s)
                continue

            gray = self._prepare_gray(frame)
            kp, des = self._orb.detectAndCompute(gray, None)
            display = frame.copy()

            match_count = 0
            if prev_des is not None and des is not None:
                matches = self._bf.match(prev_des, des)
                matches = sorted(matches, key=lambda m: m.distance)
                matches = matches[: self.config.draw_matches]
                match_count = len(matches)
                if match_count >= 8:
                    pts_prev = np.float32(
                        [prev_kp[m.queryIdx].pt for m in matches]
                    ).reshape(-1, 1, 2)
                    pts_curr = np.float32(
                        [kp[m.trainIdx].pt for m in matches]
                    ).reshape(-1, 1, 2)
                    essential, _mask = cv2.findEssentialMat(
                        pts_curr,
                        pts_prev,
                        self.k,
                        method=cv2.RANSAC,
                        prob=0.999,
                        threshold=1.0,
                    )
                    if essential is not None:
                        _inliers, rotation, translation, _pose_mask = cv2.recoverPose(
                            essential, pts_curr, pts_prev, self.k
                        )
                        self._rotation = self._rotation @ rotation
                        self._translation += self._rotation @ (
                            translation * self.config.motion_scale
                        )
                        self._draw_trajectory(trajectory_window)

            if kp:
                display = cv2.drawKeypoints(
                    display,
                    kp,
                    None,
                    color=(0, 255, 255),
                    flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT,
                )
            cv2.putText(
                display,
                f"Matches: {match_count}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.imshow(window_name, display)

            if cv2.waitKey(1) & 0xFF == 27:
                break

            time.sleep(self.config.frame_delay_s)

        self.camera_driver.release()
        cv2.destroyWindow(window_name)
        cv2.destroyWindow(trajectory_window)
