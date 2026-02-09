"""Camera driver factory and compatibility helpers."""

import threading
import time

from navisar.sensors.cameras.opencv import OpenCVCamera


CameraDriver = OpenCVCamera


class SharedCamera:
    """Thread-safe wrapper to share a single camera instance."""
    def __init__(self, camera_driver):
        self._camera_driver = camera_driver
        self._lock = threading.Lock()
        self._released = False

    def read(self):
        """Serialize access to the underlying camera read."""
        with self._lock:
            return self._camera_driver.read()

    def release(self):
        """Release the camera only once when shared."""
        with self._lock:
            if self._released:
                return
            self._released = True
            self._camera_driver.release()


class RateLimitedCamera:
    """Wrap a camera driver to enforce a max frame rate."""
    def __init__(self, camera_driver, rate_hz):
        self._camera_driver = camera_driver
        self._interval_s = 0.0
        if rate_hz is not None:
            try:
                rate_hz = float(rate_hz)
            except (TypeError, ValueError):
                rate_hz = None
        if rate_hz and rate_hz > 0:
            self._interval_s = 1.0 / rate_hz
        self._last_read = None

    def read(self):
        """Read a frame while throttling to the configured rate."""
        if self._interval_s > 0.0:
            now = time.monotonic()
            if self._last_read is not None:
                elapsed = now - self._last_read
                if elapsed < self._interval_s:
                    time.sleep(self._interval_s - elapsed)
            self._last_read = time.monotonic()
        return self._camera_driver.read()

    def release(self):
        """Release the underlying camera."""
        return self._camera_driver.release()


def create_camera_driver(camera_cfg):
    """Instantiate a camera driver from config."""
    model = str(camera_cfg.get("model", "opencv")).strip().lower()
    width = camera_cfg.get("width", 640)
    height = camera_cfg.get("height", 480)
    rate_hz = camera_cfg.get("rate_hz")

    if model in {"opencv", "usb", "generic"}:
        index = camera_cfg.get("index", 0)
        fourcc = camera_cfg.get("fourcc")
        driver = OpenCVCamera(
            index=index,
            width=width,
            height=height,
            fourcc=fourcc,
            fps=rate_hz,
        )
        return RateLimitedCamera(driver, rate_hz) if rate_hz else driver

    if model in {"ov9281", "ov9821"}:
        from navisar.sensors.cameras.ov9281 import OV9281Camera

        format_name = camera_cfg.get("format", "YUV420")
        driver = OV9281Camera(width=width, height=height, format_name=format_name)
        return RateLimitedCamera(driver, rate_hz) if rate_hz else driver

    raise ValueError(f"Unknown camera model '{model}'")
