"""
The message the stitcher receives for every drone photo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def is_finite(value: object) -> bool:
    """
    True for a finite number. A missing value (None, JSON null) or anything
    else must reject the photo, not crash the stitcher's worker thread.
    """
    try:
        return math.isfinite(value)
    except TypeError:
        return False


@dataclass
class ImageMessage:
    """
    One drone photo tagged with where it was taken.

    lat, lon: drone GPS position in degrees
    alt: metres above the launch point
    img: H x W x 3 RGB array
    roll_deg, pitch_deg: drone attitude when the photo was taken, 0 = level.
        NaN if unknown, which gets the photo rejected.
    yaw_deg: degrees clockwise from north that the top of the image faces.
        NaN if unknown, which gets the photo rejected.
    """

    lat: float
    lon: float
    alt: float
    img: np.ndarray
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    drone_id: str = ""
    image_id: str = ""

    def has_valid_position(self) -> bool:
        """
        False when the drone had no GPS fix (the camera node writes NaN, a
        glitching receiver reports 0, 0) or reports a non-positive altitude.
        """
        values = (self.lat, self.lon, self.alt)
        if not all(is_finite(value) for value in values):
            return False
        no_fix = abs(self.lat) < 1e-6 and abs(self.lon) < 1e-6
        return not no_fix and self.alt > 0

    def has_valid_image(self) -> bool:
        """
        True for an 8-bit H x W x 3 image, the format the camera node sends.
        """
        return (
            isinstance(self.img, np.ndarray)
            and self.img.dtype == np.uint8
            and self.img.ndim == 3
            and self.img.shape[2] == 3
            and self.img.shape[0] > 0
            and self.img.shape[1] > 0
        )
