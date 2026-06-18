import cv2

from .abstract_camera import AbstractCamera
from .frame import CameraFrame


class ArduCam(AbstractCamera):
    def __init__(self, device_index: int = 0, width: int = 1280, height: int = 720) -> None:
        super().__init__()
        self.device_index = device_index
        self.width = width
        self.height = height
        self._capture: cv2.VideoCapture | None = None

    def initialize_camera(self) -> bool:
        capture = cv2.VideoCapture(self.device_index)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not capture.isOpened():
            capture.release()
            self._capture = None
            return False

        self._capture = capture
        return True

    def capture_frame(self) -> CameraFrame | None:
        """Grab one frame and wrap it in a CameraFrame; None if the read failed."""
        if self._capture is None:
            return None

        ret, frame = self._capture.read()
        if not ret or frame is None:
            return None
        return CameraFrame(rgb=frame, depth=None, rgb_down=None)

    def stop(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
