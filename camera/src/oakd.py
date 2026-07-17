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
        raise NotImplementedError("OAK-D pipeline setup is not implemented yet.")

    def capture_frame(self) -> CameraFrame | None:
        raise NotImplementedError("OAK-D frame capture is not implemented yet.")

    def close_camera(self) -> None:
        raise NotImplementedError("OAK-D shutdown is not implemented yet.")

    def get_point_cloud(self) -> object | None:
        """Return the latest point cloud if the active pipeline produces one."""
        raise NotImplementedError("OAK-D point cloud support is not implemented yet.")

    def get_visual_odometry(self) -> object | None:
        """Return the latest VO estimate if the active pipeline produces one."""
        raise NotImplementedError("OAK-D visual odometry support is not implemented yet.")

    def get_imu_data(self) -> object | None:
        """Return the latest IMU sample if the active pipeline produces one."""
        raise NotImplementedError("OAK-D IMU support is not implemented yet.")
