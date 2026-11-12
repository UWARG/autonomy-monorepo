"""Camera footprint, altitude, and overlap math for scan planning."""

from __future__ import annotations

import math

from .models import CameraSpec


def validate_camera(camera: CameraSpec) -> None:
    """Raises ``ValueError`` if the camera spec is unusable."""

    if camera.image_width_px <= 0 or camera.image_height_px <= 0:
        raise ValueError("image dimensions must be positive")
    if not 0.0 < camera.horizontal_fov_rad < math.pi:
        raise ValueError("horizontal_fov_rad must be in (0, pi)")
    if not 0.0 < camera.vertical_fov_rad < math.pi:
        raise ValueError("vertical_fov_rad must be in (0, pi)")


def derive_altitude(camera: CameraSpec, target_gsd_m: float) -> float:
    """Altitude AGL that meets the target GSD on both image axes.

    Returns the smaller of the altitudes implied by the horizontal and
    vertical axes, so the coarser axis exactly meets the target and the
    other ends up finer.
    """

    if target_gsd_m <= 0.0:
        raise ValueError("target_gsd_m must be positive")
    validate_camera(camera)

    horizontal_altitude = target_gsd_m * camera.image_width_px / (
        2.0 * math.tan(camera.horizontal_fov_rad / 2.0)
    )
    vertical_altitude = target_gsd_m * camera.image_height_px / (
        2.0 * math.tan(camera.vertical_fov_rad / 2.0)
    )
    return min(horizontal_altitude, vertical_altitude)


def ground_footprint(altitude_m: float, camera: CameraSpec) -> tuple[float, float]:
    """Returns the ``(width, length)`` in meters one photo covers on the ground."""

    if altitude_m <= 0.0:
        raise ValueError("altitude_m must be positive")
    validate_camera(camera)

    width_m = 2.0 * altitude_m * math.tan(camera.horizontal_fov_rad / 2.0)
    length_m = 2.0 * altitude_m * math.tan(camera.vertical_fov_rad / 2.0)
    return width_m, length_m


def ground_sample_distance(
    altitude_m: float, camera: CameraSpec
) -> tuple[float, float]:
    """Returns the ``(gsd_x, gsd_y)`` meters-per-pixel at the given altitude."""

    width_m, length_m = ground_footprint(altitude_m, camera)
    return width_m / camera.image_width_px, length_m / camera.image_height_px


def spacings(
    footprint_width_m: float,
    footprint_length_m: float,
    forward_overlap: float,
    side_overlap: float,
) -> tuple[float, float]:
    """Returns the ``(capture_spacing, line_spacing)`` in meters.

    ``capture_spacing`` is the distance between photos along a flight line;
    ``line_spacing`` is the distance between adjacent lines. Both follow from
    the overlap fractions.
    """

    if not 0.0 <= forward_overlap < 1.0:
        raise ValueError("forward_overlap must be in [0, 1)")
    if not 0.0 <= side_overlap < 1.0:
        raise ValueError("side_overlap must be in [0, 1)")

    capture_spacing_m = footprint_length_m * (1.0 - forward_overlap)
    line_spacing_m = footprint_width_m * (1.0 - side_overlap)
    return capture_spacing_m, line_spacing_m