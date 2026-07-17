"""
OakD camera implementation.

---

"""

from __future__ import annotations

from abstract_camera import AbstractCamera
from frame import CameraFrame


class OakD(AbstractCamera):
    """Concrete OAK-D camera adapter.

    hardware-specific pipeline should be built here
    """

    def initialize_camera(self) -> bool:
        raise NotImplementedError("not implemented yet.")

    def capture_frame(self) -> CameraFrame | None:
        raise NotImplementedError("not implemented yet.")

    def close_camera(self) -> None:
        raise NotImplementedError("not implemented yet.")

    def get_point_cloud(self) -> object | None:
        raise NotImplementedError("not implemented yet.")

    def get_visual_odometry(self) -> object | None:
        raise NotImplementedError("not implemented yet.")

    def get_imu_data(self) -> object | None:
        raise NotImplementedError("not implemented yet.")
