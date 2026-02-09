#!/usr/bin/env python3
"""Estimate yaw/rotation offset so straight motion aligns with the image axes."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Estimate yaw offset from optical flow (move in a straight line)."
    )
    parser.add_argument(
        "--backend",
        type=str,
        default="opencv",
        choices=("opencv", "picamera2"),
        help="Capture backend: opencv or picamera2.",
    )
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument("--width", type=int, default=640, help="Capture width.")
    parser.add_argument("--height", type=int, default=480, help="Capture height.")
    parser.add_argument(
        "--format",
        type=str,
        default="YUV420",
        help="Picamera2 pixel format (e.g., YUV420, RGB888).",
    )
    parser.add_argument(
        "--direction",
        type=str,
        default="right",
        choices=("right", "left", "up", "down"),
        help="Direction you will move the camera.",
    )
    parser.add_argument("--samples", type=int, default=40, help="Valid motion samples.")
    parser.add_argument("--min-motion", type=float, default=0.4, help="Min px motion.")
    return parser.parse_args()


class _CvCamera:
    def __init__(self, index, width, height):
        self._cap = cv2.VideoCapture(index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self._cap.isOpened():
            raise RuntimeError("Failed to open OpenCV camera.")

    def read(self):
        return self._cap.read()

    def release(self):
        self._cap.release()


class _PiCamera2Wrapper:
    def __init__(self, width, height, format_name):
        repo_root = Path(__file__).resolve().parents[1]
        src_path = repo_root / "src"
        if str(src_path) not in __import__("sys").path:
            __import__("sys").path.insert(0, str(src_path))
        from navisar.sensors.cameras.ov9281 import OV9281Camera

        self._cam = OV9281Camera(width=width, height=height, format_name=format_name)

    def read(self):
        ok, frame = self._cam.read()
        return ok, frame

    def release(self):
        self._cam.release()


def _open_camera(args):
    if args.backend == "picamera2":
        return _PiCamera2Wrapper(args.width, args.height, args.format)
    return _CvCamera(args.camera_index, args.width, args.height)


def _direction_angle(direction):
    mapping = {
        "right": 0.0,
        "up": -90.0,
        "left": 180.0,
        "down": 90.0,
    }
    return mapping[direction]


def main():
    args = _parse_args()
    cap = _open_camera(args)

    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Failed to read from camera.")
    if frame.ndim == 2:
        prev_gray = frame.copy()
    else:
        prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    collected = 0
    motion_vectors = []
    desired_deg = _direction_angle(args.direction)

    print(
        f"Move the camera straight {args.direction.upper()} for {args.samples} samples."
    )
    print("Press Q to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if frame.ndim == 2:
            gray = frame
            preview = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            preview = frame.copy()

        features = cv2.goodFeaturesToTrack(
            prev_gray, maxCorners=300, qualityLevel=0.01, minDistance=7
        )
        if features is not None:
            next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                prev_gray, gray, features, None, winSize=(21, 21), maxLevel=3
            )
            if next_pts is not None:
                good_old = features[status == 1]
                good_new = next_pts[status == 1]
                if len(good_old) > 10:
                    dx = np.mean(good_new[:, 0] - good_old[:, 0])
                    dy = np.mean(good_new[:, 1] - good_old[:, 1])
                    mag = float(np.hypot(dx, dy))
                    if mag >= args.min_motion:
                        motion_vectors.append((dx, dy))
                        collected += 1

                    center = (preview.shape[1] // 2, preview.shape[0] // 2)
                    arrow_end = (int(center[0] + dx * 10), int(center[1] + dy * 10))
                    cv2.arrowedLine(preview, center, arrow_end, (0, 255, 0), 2)

        cv2.putText(
            preview,
            f"Samples: {collected}/{args.samples}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.imshow("Yaw Offset Calibration", preview)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if collected >= args.samples:
            break

        prev_gray = gray

    cap.release()
    cv2.destroyAllWindows()

    if not motion_vectors:
        raise RuntimeError("No motion captured; try again with larger movement.")

    mean_dx = float(np.mean([v[0] for v in motion_vectors]))
    mean_dy = float(np.mean([v[1] for v in motion_vectors]))
    angle_deg = math.degrees(math.atan2(mean_dy, mean_dx))
    yaw_offset = desired_deg - angle_deg

    print("\nCalibration result")
    print(f"Mean flow: dx={mean_dx:.3f} dy={mean_dy:.3f}")
    print(f"Measured angle: {angle_deg:.2f} deg")
    print(f"Desired angle: {desired_deg:.2f} deg")
    print(f"Yaw offset to apply: {yaw_offset:.2f} deg")
    print("Apply as: x' = x*cos(t) - y*sin(t), y' = x*sin(t) + y*cos(t)")
    print(f"Where t = {yaw_offset:.2f} deg")


if __name__ == "__main__":
    main()
