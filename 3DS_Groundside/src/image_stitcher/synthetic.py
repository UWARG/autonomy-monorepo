"""
Fake drone photos for testing the stitcher without a real flight.

A large "ground truth" texture stands in for the area being mapped. Photos are
cut out of it along a lawn mower path, with a downward camera model, so the
true position of every photo is known and the stitcher can be scored.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from src.image_stitcher.geo import (
    ground_sample_distance,
    local_to_gps,
    nadir_image_to_local,
)
from src.image_stitcher.message import ImageMessage


@dataclass(frozen=True)
class SyntheticGround:
    """
    A texture covering the area being mapped, north up.

    Pixel (0, 0) is the north west corner, which sits at `origin` (lat, lon).
    Local metres used below are east and north of that corner, so the whole
    ground has east >= 0 and north <= 0.
    """

    texture: np.ndarray
    resolution_m: float
    origin: tuple[float, float]

    @property
    def width_m(self) -> float:
        return self.texture.shape[1] * self.resolution_m

    @property
    def height_m(self) -> float:
        return self.texture.shape[0] * self.resolution_m


@dataclass(frozen=True)
class SyntheticPhoto:
    """
    A fake photo plus the truth the stitcher has to recover.
    """

    message: ImageMessage
    # Truth in metres east/north of the ground's north west corner.
    true_east: float
    true_north: float
    true_heading_deg: float
    # True image pixels -> local metres (same frame as true_east/north).
    true_image_to_local: np.ndarray


def make_ground(
    width_px: int = 2400,
    height_px: int = 1800,
    resolution_m: float = 0.05,
    origin: tuple[float, float] = (43.4723, -80.5449),
    seed: int = 0,
) -> SyntheticGround:
    """
    Generates a random, feature-rich texture (blobs, lines and noise).
    """
    rng = np.random.default_rng(seed)
    noise = rng.random((height_px // 12, width_px // 12, 3)).astype(np.float32)
    texture = cv2.resize(noise, (width_px, height_px), interpolation=cv2.INTER_CUBIC)
    texture = np.clip(texture * 160 + 50, 0, 255).astype(np.uint8)

    area_scale = width_px * height_px / (1600 * 1200)
    for _ in range(int(250 * area_scale)):
        colour = tuple(int(c) for c in rng.integers(0, 256, 3))
        centre = (int(rng.integers(0, width_px)), int(rng.integers(0, height_px)))
        shape = rng.random()
        if shape < 0.4:
            cv2.circle(texture, centre, int(rng.integers(4, 30)), colour, -1)
        elif shape < 0.7:
            size = rng.integers(6, 50, 2)
            corner = (centre[0] + int(size[0]), centre[1] + int(size[1]))
            cv2.rectangle(texture, centre, corner, colour, -1)
        else:
            end = (
                centre[0] + int(rng.integers(-200, 200)),
                centre[1] + int(rng.integers(-200, 200)),
            )
            cv2.line(texture, centre, end, colour, int(rng.integers(1, 5)))

    return SyntheticGround(np.ascontiguousarray(texture), resolution_m, origin)


def take_photo(
    ground: SyntheticGround,
    fov_deg: float,
    east_m: float,
    north_m: float,
    altitude_m: float,
    heading_deg: float = 0.0,
    image_size: tuple[int, int] = (640, 480),
) -> tuple[np.ndarray, np.ndarray]:
    """
    Renders what a nadir camera at (east_m, north_m) would see.

    Returns (rgb, image_to_local).
    """
    width, height = image_size
    gsd_m = ground_sample_distance(altitude_m, fov_deg, width)
    image_to_local = nadir_image_to_local(
        width, height, gsd_m, heading_deg, east_m, north_m
    )
    res = ground.resolution_m
    local_to_texture = np.array(
        [
            [1 / res, 0, -0.5],
            [0, -1 / res, -0.5],
            [0, 0, 1],
        ]
    )
    image_to_texture = local_to_texture @ image_to_local
    rgb = cv2.warpAffine(
        ground.texture,
        image_to_texture[:2],
        (width, height),
        flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
        borderMode=cv2.BORDER_REFLECT,
    )
    return rgb, image_to_local


def lawn_mower_mission(
    ground: SyntheticGround,
    fov_deg: float = 70.0,
    altitude_m: float = 20.0,
    overlap: float = 0.6,
    image_size: tuple[int, int] = (640, 480),
    gps_noise_m: float = 0.0,
    altitude_noise_m: float = 0.0,
    heading_noise_deg: float = 0.0,
    drone_count: int = 1,
    seed: int = 0,
) -> list[SyntheticPhoto]:
    """
    Photos along back-and-forth sweeps covering the whole ground.

    Like a real lawn mower mission, the drone faces along its sweep, so the
    heading flips 180 degrees every row. Rows are split between drones.
    GPS and altitude errors only affect what is reported in the message;
    photos are always taken from the true position.
    """
    rng = np.random.default_rng(seed)
    width, height = image_size
    gsd_m = ground_sample_distance(altitude_m, fov_deg, width)
    # The drone flies east/west with the image top facing its direction of
    # travel, so the image's height spans east-west along each row.
    along_m = height * gsd_m
    across_m = width * gsd_m
    columns = _sweep(along_m / 2, ground.width_m - along_m / 2, along_m * (1 - overlap))
    rows = _sweep(
        across_m / 2, ground.height_m - across_m / 2, across_m * (1 - overlap)
    )

    photos: list[SyntheticPhoto] = []
    for row_index, row in enumerate(rows):
        drone = row_index * drone_count // len(rows)
        heading = 90.0 if row_index % 2 == 0 else 270.0
        ordered = columns if row_index % 2 == 0 else columns[::-1]
        for column in ordered:
            east, north = column, -row
            true_heading = heading + rng.normal(0, heading_noise_deg)
            true_altitude = altitude_m + rng.normal(0, altitude_noise_m)
            rgb, image_to_local = take_photo(
                ground, fov_deg, east, north, true_altitude, true_heading, image_size
            )
            lat, lon = local_to_gps(
                east + rng.normal(0, gps_noise_m),
                north + rng.normal(0, gps_noise_m),
                *ground.origin,
            )
            message = ImageMessage(
                lat=lat,
                lon=lon,
                alt=true_altitude + rng.normal(0, altitude_noise_m),
                img=rgb,
                drone_id=f"drone-{drone + 1:02d}",
                image_id=f"photo_{len(photos) + 1:04d}",
            )
            photos.append(
                SyntheticPhoto(message, east, north, true_heading, image_to_local)
            )
    return photos


def _sweep(start: float, end: float, step: float) -> list[float]:
    """
    Evenly spaced positions from start to end (inclusive) at most `step` apart.
    """
    if end <= start:
        return [(start + end) / 2]
    count = math.ceil((end - start) / step) + 1
    return list(np.linspace(start, end, count))
