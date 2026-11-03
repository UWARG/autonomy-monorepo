"""
The drone camera's calibration: undistorts photos and gives how much ground
each pixel covers.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml

# Calibration of the 640 x 480 Arducam, in the ROS camera_info format written
# by airside_prec/camera_calibration.py (precision_landing branch).
DEFAULT_CAMERA_INFO = Path(__file__).with_name("camera_info.yaml")


class Camera:
    """
    A calibrated camera with lens distortion.

    Photos are undistorted into an ideal pinhole camera with square pixels and
    focal length `focal_px`, zoomed just enough that every output pixel has
    image data (no black corners). All stitcher geometry uses that ideal
    camera's pixels.
    """

    def __init__(
        self,
        matrix: np.ndarray,
        distortion: np.ndarray,
        width: int,
        height: int,
    ) -> None:
        self.width = width
        self.height = height
        self._matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        self._distortion = np.asarray(distortion, dtype=np.float64).ravel()

        ideal, _ = cv2.getOptimalNewCameraMatrix(
            self._matrix, self._distortion, (width, height), alpha=0
        )
        # The stitcher only models rotation, scale and shift between photos,
        # so pixels must be square. Zooming in on the longer focal length
        # keeps every pixel inside the photo.
        self.focal_px = float(max(ideal[0, 0], ideal[1, 1]))
        self.principal_point = ideal[:2, 2].copy()
        self._ideal_matrix = np.array(
            [
                [self.focal_px, 0.0, self.principal_point[0]],
                [0.0, self.focal_px, self.principal_point[1]],
                [0.0, 0.0, 1.0],
            ]
        )
        self._map_x, self._map_y = cv2.initUndistortRectifyMap(
            self._matrix,
            self._distortion,
            None,
            self._ideal_matrix,
            (width, height),
            cv2.CV_32FC1,
        )

    @classmethod
    def from_camera_info(cls, info: dict) -> Camera:
        """
        Builds a camera from a parsed ROS camera_info YAML.
        """
        return cls(
            np.array(info["camera_matrix"]["data"]),
            np.array(info["distortion_coefficients"]["data"]),
            int(info["image_width"]),
            int(info["image_height"]),
        )

    @classmethod
    def load(cls, path: Path = DEFAULT_CAMERA_INFO) -> Camera:
        with path.open() as file:
            return cls.from_camera_info(yaml.safe_load(file))

    def matches(self, image: np.ndarray) -> bool:
        """
        Whether an image has the size this camera was calibrated at.
        """
        return image.shape[:2] == (self.height, self.width)

    def undistort(self, image: np.ndarray) -> np.ndarray:
        if not self.matches(image):
            raise ValueError(
                f"image is {image.shape[1]} x {image.shape[0]}, camera is "
                f"calibrated for {self.width} x {self.height}"
            )
        return cv2.remap(image, self._map_x, self._map_y, cv2.INTER_LINEAR)

    def undistort_points(self, points: np.ndarray) -> np.ndarray:
        """
        Where raw photo pixels (N x 2) end up in the undistorted photo.
        """
        points = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
        undistorted = cv2.undistortPoints(
            points, self._matrix, self._distortion, P=self._ideal_matrix
        )
        return undistorted.reshape(-1, 2)

    def metres_per_px(self, altitude_m: float) -> float:
        """
        Ground metres covered by one undistorted pixel, looking straight down.
        """
        return altitude_m / self.focal_px
