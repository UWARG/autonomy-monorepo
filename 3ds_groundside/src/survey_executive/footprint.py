"""
Ground area one photo covers, from the drone's position and attitude.
"""

from __future__ import annotations

import math

from gnc.src.orthomosaic import CameraSpec
from gnc.src.orthomosaic.footprint import validate_camera
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

from src.image_stitcher.message import is_finite

# Image corners as (right, forward) signs: top-left, top-right, bottom-right,
# bottom-left, with the image top facing the drone's nose.
_CORNERS = ((-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0))
# A ray with no downward part never meets the ground (and would divide by 0).
_MIN_DOWN = 1e-9
# Relative slack on the GSD depth limit, so float rounding can't clip a level
# photo taken exactly at the limit.
_DEPTH_TOLERANCE = 1e-9


def photo_footprint(
    east_m: float,
    north_m: float,
    alt_m: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    camera: CameraSpec,
    max_gsd_m: float | None = None,
) -> Polygon | None:
    """
    Ground polygon in the survey ENU frame, or None if it can't be placed.

    The camera points straight down from the airframe with the image top
    toward the nose. Attitude follows the aerospace convention, applied yaw,
    then pitch, then roll: yaw is degrees clockwise from north, pitch is
    positive nose up, roll is positive right wing down. The ground is flat at
    altitude 0, so ``alt_m`` is height above it.

    With ``max_gsd_m``, the polygon is cut down to the ground seen at that
    GSD or finer, so a tilted photo can't claim far-off ground it only sees
    as a smear. GSD here is the square root of ground area per pixel.

    Returns None when the drone is not above the ground, an input is not
    finite, any corner ray points at or above the horizon, or no ground is
    seen finely enough.
    """

    validate_camera(camera)
    if max_gsd_m is not None and not (math.isfinite(max_gsd_m) and max_gsd_m > 0.0):
        raise ValueError("max_gsd_m must be positive and finite")
    values = (east_m, north_m, alt_m, roll_deg, pitch_deg, yaw_deg)
    # is_finite, not math.isfinite: a JSON null attitude is "no footprint".
    if not all(is_finite(value) for value in values) or alt_m <= 0.0:
        return None

    rotation = _body_to_ned(
        math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    )
    half_width = math.tan(camera.horizontal_fov_rad / 2.0)
    half_length = math.tan(camera.vertical_fov_rad / 2.0)

    corners: list[tuple[float, float]] = []
    for right_sign, forward_sign in _CORNERS:
        # Ray in the body frame (forward, right, down); the camera looks down.
        ray = (forward_sign * half_length, right_sign * half_width, 1.0)
        north, east, down = (
            sum(row[i] * ray[i] for i in range(3)) for row in rotation
        )
        if down <= _MIN_DOWN:
            return None
        scale = alt_m / down
        corners.append((east_m + east * scale, north_m + north * scale))

    if max_gsd_m is not None:
        # Optical axis in NED: the body down axis after rotation.
        axis_north, axis_east, axis_down = (row[2] for row in rotation)
        corners = _clip_to_gsd(
            corners, east_m, north_m, alt_m, (axis_east, axis_north, axis_down),
            _focal_px(camera), max_gsd_m,
        )
        if len(corners) < 3:
            return None
    polygon = Polygon(corners)
    if polygon.area <= 0.0:
        return None
    return orient(polygon)


def _focal_px(camera: CameraSpec) -> float:
    """
    Pinhole focal length in pixels, geometric mean of the two axes.
    """

    focal_x = camera.image_width_px / (2.0 * math.tan(camera.horizontal_fov_rad / 2.0))
    focal_y = camera.image_height_px / (2.0 * math.tan(camera.vertical_fov_rad / 2.0))
    return math.sqrt(focal_x * focal_y)


def _clip_to_gsd(
    corners: list[tuple[float, float]],
    east_m: float,
    north_m: float,
    alt_m: float,
    axis_enu_down: tuple[float, float, float],
    focal_px: float,
    max_gsd_m: float,
) -> list[tuple[float, float]]:
    """
    Keeps the part of the footprint whose GSD is at most ``max_gsd_m``.

    For a pinhole camera over flat ground, a point at depth Z along the
    optical axis has ground area per pixel Z**3 / (alt * f**2). So the limit
    is a maximum depth, and depth is linear on the ground: one straight cut.
    """

    max_depth_m = ((max_gsd_m * focal_px) ** 2 * alt_m) ** (1.0 / 3.0)
    max_depth_m *= 1.0 + _DEPTH_TOLERANCE
    axis_east, axis_north, axis_down = axis_enu_down

    def excess(point: tuple[float, float]) -> float:
        depth = (
            axis_east * (point[0] - east_m)
            + axis_north * (point[1] - north_m)
            + axis_down * alt_m
        )
        return depth - max_depth_m

    # One step of Sutherland-Hodgman against the half-plane excess <= 0.
    kept: list[tuple[float, float]] = []
    for index, current in enumerate(corners):
        following = corners[(index + 1) % len(corners)]
        here, there = excess(current), excess(following)
        if here <= 0.0:
            kept.append(current)
        if (here < 0.0 < there) or (there < 0.0 < here):
            t = here / (here - there)
            kept.append(
                (
                    current[0] + t * (following[0] - current[0]),
                    current[1] + t * (following[1] - current[1]),
                )
            )
    return kept


def _body_to_ned(
    roll_rad: float, pitch_rad: float, yaw_rad: float
) -> tuple[tuple[float, float, float], ...]:
    """
    Rotation from body (forward, right, down) to (north, east, down).
    """

    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return (
        (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        (-sp, sr * cp, cr * cp),
    )
