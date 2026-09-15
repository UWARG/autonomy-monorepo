import numpy as np

from .abstract_camera import AbstractCamera
from .constants import CAMERA_HEIGHT, CAMERA_WIDTH
from .frame import CameraFrame


class SimCamera(AbstractCamera):
    """Simulated camera stub that produces blank frames."""

    WIDTH = CAMERA_WIDTH
    HEIGHT = CAMERA_HEIGHT

    def initialize_camera(self) -> bool:
        return True

    def capture_frame(self) -> CameraFrame | None:
        rgb = np.zeros((self.HEIGHT, self.WIDTH, 3), dtype=np.uint8)
        return CameraFrame(rgb=rgb, depth=None, rgb_down=None)

    def close_camera(self) -> None:
        pass
