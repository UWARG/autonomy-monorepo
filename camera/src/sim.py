from .abstract_camera import AbstractCamera
from .frame import CameraFrame


class SimCamera(AbstractCamera):
    """Simulated camera stub"""

    def initialize_camera(self) -> bool:
        return True

    def capture_frame(self) -> CameraFrame | None:
        return None

    def close_camera(self) -> None:
        return None
