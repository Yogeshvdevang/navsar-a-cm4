#!/usr/bin/env python3
"""Interactive chessboard camera calibration helper."""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np


def _parse_args():
    """Parse CLI arguments for camera calibration."""
    parser = argparse.ArgumentParser(
        description="Calibrate camera intrinsics using a chessboard pattern."
    )
    parser.add_argument(
        "--backend",
        type=str,
        default="opencv",
        choices=("opencv", "picamera2"),
        help="Capture backend: opencv (USB/UVC) or picamera2 (CSI/OV9281).",
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
        "--board-cols", type=int, default=9, help="Chessboard inner corners (columns)."
    )
    parser.add_argument(
        "--board-rows", type=int, default=6, help="Chessboard inner corners (rows)."
    )
    parser.add_argument(
        "--square-size",
        type=float,
        default=0.025,
        help="Square size in meters (used for scaling).",
    )
    parser.add_argument("--samples", type=int, default=20, help="Valid samples to collect.")
    parser.add_argument(
        "--save-dir",
        type=str,
        default="",
        help="Optional directory to save captured images.",
    )
    return parser.parse_args()


def _yaml_snippet(fx, fy, cx, cy, dist_coeffs):
    """Render intrinsics as a YAML snippet."""
    coeffs = ", ".join(f"{c:.6f}" for c in dist_coeffs[:5])
    return (
        "intrinsics:\n"
        f"  fx: {fx:.6f}\n"
        f"  fy: {fy:.6f}\n"
        f"  cx: {cx:.6f}\n"
        f"  cy: {cy:.6f}\n"
        f"  dist_coeffs: [{coeffs}]\n"
    )


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


def _to_gray_and_preview(frame):
    if frame.ndim == 2:
        gray = frame
        preview = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        preview = frame.copy()
    return gray, preview


def main():
    """Run interactive chessboard calibration."""
    args = _parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    src_path = repo_root / "src"
    if str(src_path) not in sys.path:
        sys.path.insert(0, str(src_path))
    board_size = (args.board_cols, args.board_rows)
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : board_size[0], 0 : board_size[1]].T.reshape(-1, 2)
    objp *= float(args.square_size)

    save_dir = None
    if args.save_dir:
        save_dir = Path(args.save_dir)
        save_dir.mkdir(parents=True, exist_ok=True)

    cap = _open_camera(args)

    obj_points = []
    img_points = []
    collected = 0
    img_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print("Press SPACE to capture when corners are visible. Press Q to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        if img_size is None:
            img_size = (frame.shape[1], frame.shape[0])

        gray, preview = _to_gray_and_preview(frame)
        found, corners = cv2.findChessboardCorners(gray, board_size, None)
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(preview, board_size, corners, found)

        cv2.putText(
            preview,
            f"Samples: {collected}/{args.samples}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.imshow("Camera Calibration", preview)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        if key == ord(" ") and found:
            obj_points.append(objp)
            img_points.append(corners)
            collected += 1
            if save_dir is not None:
                filename = save_dir / f"calib_{collected:03d}.png"
                cv2.imwrite(str(filename), frame)
            if collected >= args.samples:
                break

    cap.release()
    cv2.destroyAllWindows()

    if collected < 5:
        raise RuntimeError("Not enough samples collected to calibrate.")

    rms, camera_mtx, dist, _, _ = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )

    fx = float(camera_mtx[0, 0])
    fy = float(camera_mtx[1, 1])
    cx = float(camera_mtx[0, 2])
    cy = float(camera_mtx[1, 2])
    dist_coeffs = dist.ravel().tolist()

    print("\nCalibration complete")
    print(f"RMS reprojection error: {rms:.6f}")
    print(_yaml_snippet(fx, fy, cx, cy, dist_coeffs))


if __name__ == "__main__":
    main()
