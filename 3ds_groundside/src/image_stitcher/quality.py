"""
Rejects photos that would spoil the map even if they match: blurry or
washed out ones blend mush over good ones, and tilted ones aren't the
straight-down view the stitcher assumes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from src.image_stitcher.message import ImageMessage, is_finite

# Grey levels counted as clipped: crushed to black or blown out to white.
DARK_LEVEL = 10
BRIGHT_LEVEL = 245

# Second difference along x, y and both diagonals.
_DIRECTIONS = [
    np.array([[1.0, -2.0, 1.0]]),
    np.array([[1.0], [-2.0], [1.0]]),
    np.array([[1.0, 0.0, 0.0], [0.0, -2.0, 0.0], [0.0, 0.0, 1.0]]),
    np.array([[0.0, 0.0, 1.0], [0.0, -2.0, 0.0], [1.0, 0.0, 0.0]]),
]


@dataclass
class QualityLimits:
    """
    Rejection thresholds. The image thresholds were picked on synthetic
    photos and should be checked against real flight photos.
    """

    # Largest angle (degrees) between the camera and straight down, from roll
    # and pitch together. A tilted photo's scale changes across the image, so
    # rotation, scale and shift can't line it up.
    max_tilt_deg: float = 5.0
    # Lowest sharpness (see sharpness()). Sits between sharp synthetic photos
    # and slightly blurred ones (Gaussian sigma 0.7 px, 5 px motion), leaning
    # towards keeping photos; test_stitcher_quality.py checks a whole mission.
    min_sharpness: float = 0.32
    # Largest fraction of pixels allowed to be clipped dark or bright.
    max_clipped_fraction: float = 0.2


def sharpness(img: np.ndarray) -> float:
    """
    Pixel-scale detail relative to detail twice as coarse: similar in a sharp
    photo, while blur wipes out the fine detail first. Comparing the photo
    with itself makes the score independent of the ground and exposure.

    The worst of x, y and both diagonals counts, since motion blur only
    smears one way. A 3 x 3 median first removes most sensor noise, which
    would look like fine detail. Returns 0 for a featureless image.
    """
    grey = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    grey = cv2.medianBlur(grey, 3).astype(np.float64)
    half = cv2.pyrDown(grey)
    scores = []
    for kernel in _DIRECTIONS:
        coarse = cv2.filter2D(half, cv2.CV_64F, kernel).var()
        if coarse < 1e-6:
            return 0.0
        scores.append(cv2.filter2D(grey, cv2.CV_64F, kernel).var() / coarse)
    return min(scores)


def clipped_fractions(img: np.ndarray) -> tuple[float, float]:
    """
    Fractions of pixels crushed to black and blown out to white.
    """
    grey = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    return float((grey <= DARK_LEVEL).mean()), float((grey >= BRIGHT_LEVEL).mean())


def tilt_deg(roll_deg: float, pitch_deg: float) -> float:
    """
    Angle between the camera axis and straight down for a camera rolled and
    pitched by these angles. Both together tilt it more than either alone.
    """
    cos_tilt = math.cos(math.radians(roll_deg)) * math.cos(math.radians(pitch_deg))
    return math.degrees(math.acos(min(1.0, cos_tilt)))


def rejection_reason(message: ImageMessage, limits: QualityLimits) -> str | None:
    """
    Why the photo should not be used, or None if it is fine. Expects a valid
    H x W x 3 image.
    """
    if not (is_finite(message.roll_deg) and is_finite(message.pitch_deg)):
        return "no roll/pitch"
    if not is_finite(message.yaw_deg):
        return "no yaw"
    tilt = tilt_deg(message.roll_deg, message.pitch_deg)
    if tilt > limits.max_tilt_deg:
        return f"tilted {tilt:.1f} deg (limit {limits.max_tilt_deg:.1f})"

    dark, bright = clipped_fractions(message.img)
    if bright > limits.max_clipped_fraction:
        return f"overexposed ({bright:.0%} of pixels blown out)"
    if dark > limits.max_clipped_fraction:
        return f"underexposed ({dark:.0%} of pixels black)"

    score = sharpness(message.img)
    if score < limits.min_sharpness:
        return f"blurry (sharpness {score:.3f}, limit {limits.min_sharpness:.3f})"
    return None
