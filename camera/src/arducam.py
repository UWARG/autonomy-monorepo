from .abstract_camera import AbstractCamera
from .frame import CameraFrame
import cv2
import cv_bridge


class Arducam(AbstractCamera):
    """Arducam camera"""

    def __init__(self):
        super().__init__()


    def initialize_camera(self) -> bool:
        self.cap=cv2.VideoCapture(0)
        if not self.cap.isOpened():
            return False
        return True

    def capture_frame(self) -> CameraFrame | None:

        ret, frame = self.cap.read()
        if not ret:
            return None
        return CameraFrame(rgb=frame,depth=None,rgb_down=None)
