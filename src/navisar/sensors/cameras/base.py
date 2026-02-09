"""Shared camera interface."""


class BaseCamera:
    """Abstract camera interface used by the pipeline."""
    def read(self):
        """Return (ret, frame) like cv2.VideoCapture.read()."""
        raise NotImplementedError

    def release(self):
        """Release camera resources (optional)."""
        pass
