"""
The message the stitcher receives for every drone photo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ImageMessage:
    """
    One drone photo tagged with where it was taken.

    lat, lon: drone GPS position in degrees
    alt: metres above the launch point
    img: H x W x 3 RGB array
    """

    lat: float
    lon: float
    alt: float
    img: np.ndarray
    drone_id: str = ""
    image_id: str = ""

    def has_valid_position(self) -> bool:
        """
        False when the drone had no GPS fix (the camera node writes NaN) or
        reports a non-positive altitude.
        """
        values = (self.lat, self.lon, self.alt)
        return all(math.isfinite(value) for value in values) and self.alt > 0

    def has_valid_image(self) -> bool:
        return (
            isinstance(self.img, np.ndarray)
            and self.img.ndim == 3
            and self.img.shape[2] == 3
            and self.img.shape[0] > 0
            and self.img.shape[1] > 0
        )
