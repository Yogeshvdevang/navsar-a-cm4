"""OpenCV-backed camera capture driver."""

import cv2
import re
import time
from pathlib import Path

import numpy as np

from navisar.sensors.cameras.base import BaseCamera


class OpenCVCamera(BaseCamera):
    """OpenCV VideoCapture-based camera driver."""
    def __init__(self, index=0, width=640, height=480, fourcc=None, fps=None):
        """Open the camera device and apply capture size."""
        self.width = width
        self.height = height
        self.fourcc = fourcc
        self.index = self._resolve_index(index, width=width, height=height, fourcc=fourcc)
        # Try default backend first, then fallback to V4L2 (common on Linux/RPi).
        self.cap = cv2.VideoCapture(self.index)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
        if fps:
            try:
                self.cap.set(cv2.CAP_PROP_FPS, float(fps))
            except Exception:
                pass
        if self.fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        if width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self):
        """Read a frame from the camera."""
        return self.cap.read()

    def release(self):
        """Release the OpenCV capture handle."""
        self.cap.release()

    @staticmethod
    def _resolve_index(index, width=None, height=None, fourcc=None):
        """Resolve 'auto' to a working /dev/video* device."""
        if isinstance(index, str) and index.strip().lower() == "auto":
            opened_candidate = None
            candidates = []
            for device in sorted(Path("/dev").glob("video*")):
                match = re.match(r"video(\d+)$", device.name)
                if match:
                    candidates.append(int(match.group(1)))
            for cand in candidates:
                cap = cv2.VideoCapture(cand)
                backends = [None]
                if not cap.isOpened():
                    cap.release()
                    backends = [cv2.CAP_V4L2]
                for backend in backends:
                    if backend is not None:
                        cap = cv2.VideoCapture(cand, backend)
                    if not cap.isOpened():
                        cap.release()
                        continue
                    if fourcc:
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                    if width:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    if height:
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    ok = False
                    for _ in range(10):
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            mean_val = float(np.mean(frame))
                            std_val = float(np.std(frame))
                            if mean_val > 1.0 or std_val > 1.0:
                                ok = True
                                break
                        time.sleep(0.15)
                    cap.release()
                    if ok:
                        print(f"OpenCV camera auto-selected index {cand}.")
                        return cand
                    if opened_candidate is None:
                        opened_candidate = cand
            if opened_candidate is not None:
                print(
                    "OpenCV camera auto-select could not read frames yet; "
                    f"using index {opened_candidate}."
                )
                return opened_candidate
            raise RuntimeError("OpenCV camera auto-select failed to open any /dev/video*.")
        return int(index)
