import numpy as np

from .abstract_camera import AbstractCamera
from .constants import SIM_DEFAULT_HEIGHT, SIM_DEFAULT_WIDTH
from .frame import CameraFrame


class SimCamera(AbstractCamera):
    """Simulated camera stub"""

    def __init__(
        self,
        width: int = SIM_DEFAULT_WIDTH,
        height: int = SIM_DEFAULT_HEIGHT,
    ) -> None:
        super().__init__(width=width, height=height)

    def initialize_camera(self) -> bool:
        return True

    def capture_frame(self) -> CameraFrame | None:
        rgb = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        return CameraFrame(rgb=rgb)

    def close_camera(self) -> None:
        pass
