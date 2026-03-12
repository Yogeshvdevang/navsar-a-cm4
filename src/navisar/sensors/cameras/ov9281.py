"""CSI camera driver using Picamera2."""

from navisar.sensors.cameras.base import BaseCamera


class OV9281Camera(BaseCamera):
    """Picamera2-backed driver for supported CSI sensors."""
    def __init__(self, width=640, height=400, format_name="YUV420"):
        """Configure Picamera2 with the requested format/size."""
        self.width = width
        self.height = height
        self.format_name = format_name
        try:
            from picamera2 import Picamera2
        except Exception as exc:  # pragma: no cover - hardware dependency
            raise ImportError("Picamera2 is required for CSI camera models.") from exc

        camera_info = []
        try:
            camera_info = Picamera2.global_camera_info() or []
        except Exception:
            camera_info = []
        if not camera_info:
            raise RuntimeError(
                "No camera detected by Picamera2 (global_camera_info returned 0 cameras). "
                "Check /boot/firmware/config.txt overlays, CSI cable orientation, and reboot. "
                "If you are using a USB webcam, set model: opencv in config/camera.yaml."
            )
        try:
            self._picam2 = Picamera2()
        except Exception as exc:
            raise RuntimeError(
                f"Picamera2 failed to initialize CSI camera (detected {len(camera_info)} "
                f"camera(s) in global_camera_info): {exc}"
            ) from exc
        config = self._picam2.create_video_configuration(
            main={"format": format_name, "size": (width, height)}
        )
        self._picam2.configure(config)
        self._picam2.start()

    def read(self):
        """Capture a frame and return a grayscale image."""
        frame = self._picam2.capture_array()
        if frame is None:
            return False, None

        if frame.ndim == 2:
            # YUV420 luma plane is the top portion of the frame.
            return True, frame[: self.height, : self.width]

        if frame.ndim == 3 and frame.shape[2] >= 1:
            return True, frame[:, :, 0]

        return False, None

    def release(self):
        """Stop the Picamera2 stream."""
        self._picam2.stop()
