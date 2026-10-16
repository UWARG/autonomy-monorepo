"""
Geometry helpers for placing drone photos on a flat ground map.

Conventions used throughout the stitcher:
- "Local" coordinates are metres east and north of a reference GPS point.
- Image pixels are (x, y) with x to the right and y downward.
- Transforms are 3x3 matrices acting on (x, y, 1) column vectors.
- The camera points straight down (nadir) and the ground is flat.
"""

from __future__ import annotations

import math

import numpy as np

EARTH_RADIUS_M = 6_378_137.0


def gps_to_local(
    lat: float, lon: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    """
    Converts a GPS coordinate to (east, north) metres from a reference point.

    Uses an equirectangular approximation, which is accurate to well under a
    metre over the few kilometres a mapping mission covers.
    """
    north = math.radians(lat - ref_lat) * EARTH_RADIUS_M
    east = (
        math.radians(lon - ref_lon) * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    )
    return east, north


def local_to_gps(
    east: float, north: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    """
    Inverse of gps_to_local: converts (east, north) metres back to (lat, lon).
    """
    lat = ref_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = ref_lon + math.degrees(
        east / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat)))
    )
    return lat, lon


def ground_sample_distance(
    altitude_m: float, horizontal_fov_deg: float, image_width_px: int
) -> float:
    """
    Metres of ground covered by one image pixel for a downward-facing camera.

    The photo's ground width is 2 * altitude * tan(fov / 2), split evenly
    across the image's pixel columns.
    """
    if altitude_m <= 0:
        raise ValueError(f"altitude must be positive, got {altitude_m}")
    ground_width_m = 2.0 * altitude_m * math.tan(math.radians(horizontal_fov_deg) / 2)
    return ground_width_m / image_width_px


def nadir_image_to_local(
    image_width_px: int,
    image_height_px: int,
    gsd_m: float,
    heading_deg: float,
    centre_east_m: float,
    centre_north_m: float,
) -> np.ndarray:
    """
    Transform from image pixels to local metres for a nadir photo.

    heading_deg is degrees clockwise from north that the image top faces, so
    with heading 0 image right is east and image up is north.
    """
    cos_h = math.cos(math.radians(heading_deg))
    sin_h = math.sin(math.radians(heading_deg))
    # Offsets from the image centre along the image axes, in metres:
    #   right = (x - cx) * gsd,  up = -(y - cy) * gsd
    # Rotating (right, up) clockwise by the heading gives (east, north):
    #   east  =  right * cos + up * sin
    #   north = -right * sin + up * cos
    linear = gsd_m * np.array([[cos_h, -sin_h], [-sin_h, -cos_h]])
    centre_px = image_centre(image_width_px, image_height_px)
    translation = np.array([centre_east_m, centre_north_m]) - linear @ centre_px
    transform = np.eye(3)
    transform[:2, :2] = linear
    transform[:2, 2] = translation
    return transform


def image_centre(width_px: int, height_px: int) -> np.ndarray:
    """
    Centre of an image in pixel coordinates (pixel centres are integers).
    """
    return np.array([(width_px - 1) / 2, (height_px - 1) / 2])


def apply(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    """
    Applies a 3x3 transform to an (N, 2) array of points.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    return points @ transform[:2, :2].T + transform[:2, 2]


def to_3x3(affine: np.ndarray) -> np.ndarray:
    """
    Converts a 2x3 OpenCV affine matrix to a 3x3 transform.
    """
    return np.vstack([affine, [0.0, 0.0, 1.0]])


def scale_of(transform: np.ndarray) -> float:
    """
    Uniform scale factor of a similarity transform.
    """
    return math.sqrt(abs(np.linalg.det(transform[:2, :2])))


def footprint_corners(
    transform: np.ndarray, width_px: int, height_px: int
) -> np.ndarray:
    """
    The four image corners mapped through a transform, as a (4, 2) array.
    """
    corners = np.array(
        [[0, 0], [width_px - 1, 0], [width_px - 1, height_px - 1], [0, height_px - 1]]
    )
    return apply(transform, corners)


def fit_similarity(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """
    Least-squares rotation + uniform scale + translation mapping source
    points onto target points (Umeyama's method). No reflection.
    """
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centred = source - source_mean
    target_centred = target - target_mean

    covariance = target_centred.T @ source_centred / len(source)
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[1, 1] = -1
    rotation = u @ correction @ vt
    source_variance = (source_centred**2).sum() / len(source)
    scale = (singular_values * np.diag(correction)).sum() / source_variance

    transform = np.eye(3)
    transform[:2, :2] = scale * rotation
    transform[:2, 2] = target_mean - scale * rotation @ source_mean
    return transform


# Flips image y (down) so pixel axes match east/north (y up) before fitting
# a reflection-free similarity.
FLIP_Y = np.diag([1.0, -1.0, 1.0])
