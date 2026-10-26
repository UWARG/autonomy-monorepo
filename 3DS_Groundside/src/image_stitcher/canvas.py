"""
The output map: a north-up canvas that photos are warped onto and blended.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from src.image_stitcher.geo import footprint_corners, local_to_gps


@dataclass(frozen=True)
class MapBounds:
    """
    GPS (lat, lon) of the rendered map image's outer corners.
    """

    north_west: tuple[float, float]
    north_east: tuple[float, float]
    south_east: tuple[float, float]
    south_west: tuple[float, float]


class Canvas:
    """
    Accumulates warped photos into one top-down map.

    One canvas pixel is `resolution_m` metres of ground, with north up. The
    canvas grows as photos land outside it. Overlapping photos are blended
    with a weighted average favouring pixels near each photo's centre, which
    hides seams.
    """

    def __init__(
        self,
        resolution_m: float,
        reference: tuple[float, float],
        max_size_px: int = 20_000,
    ) -> None:
        """
        resolution_m: metres of ground per canvas pixel
        reference: (lat, lon) that local (0, 0) metres refers to
        max_size_px: largest allowed canvas side, guards against bad positions
        """
        if resolution_m <= 0:
            raise ValueError(f"resolution must be positive, got {resolution_m}")
        self._resolution_m = resolution_m
        self._reference = reference
        self._max_size_px = max_size_px

        # Canvas edges in whole pixels from the reference point.
        # Column c covers east [(left + c) * res, (left + c + 1) * res).
        # Row r covers north [(top - r - 1) * res, (top - r) * res).
        self._left_px = 0
        self._top_px = 0
        self._accum = np.zeros((0, 0, 3), dtype=np.float32)
        self._weight = np.zeros((0, 0), dtype=np.float32)

    @property
    def resolution_m(self) -> float:
        return self._resolution_m

    @property
    def width(self) -> int:
        return self._weight.shape[1]

    @property
    def height(self) -> int:
        return self._weight.shape[0]

    @property
    def is_empty(self) -> bool:
        return self._weight.size == 0

    def add(self, rgb: np.ndarray, image_to_local: np.ndarray) -> None:
        """
        Paints an H x W x 3 image onto the canvas.

        image_to_local: 3x3 transform from image pixels to local metres
        """
        height, width = rgb.shape[:2]
        corners = footprint_corners(image_to_local, width, height)
        self._grow_to_fit(corners)

        # Only warp into the window the photo covers, not the whole canvas.
        res = self._resolution_m
        x0 = max(math.floor(corners[:, 0].min() / res) - self._left_px - 1, 0)
        x1 = min(math.ceil(corners[:, 0].max() / res) - self._left_px + 1, self.width)
        y0 = max(self._top_px - math.ceil(corners[:, 1].max() / res) - 1, 0)
        y1 = min(self._top_px - math.floor(corners[:, 1].min() / res) + 1, self.height)

        image_to_window = self._local_to_canvas(x0, y0) @ image_to_local
        size = (x1 - x0, y1 - y0)
        # Replicate photo edges so bilinear sampling at the border doesn't
        # blend in black; the weight (zero outside the photo) sets coverage.
        warped = cv2.warpAffine(
            rgb.astype(np.float32),
            image_to_window[:2],
            size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )
        weight = cv2.warpAffine(
            _feather_weights(height, width),
            image_to_window[:2],
            size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        self._accum[y0:y1, x0:x1] += warped * weight[..., None]
        self._weight[y0:y1, x0:x1] += weight

    def render(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Returns (rgb, coverage): the blended H x W x 3 uint8 map, and an
        H x W bool mask of which pixels any photo covered.
        """
        coverage = self._weight > 1e-6
        rgb = np.zeros(self._accum.shape, dtype=np.uint8)
        blended = self._accum[coverage] / self._weight[coverage][:, None]
        rgb[coverage] = np.clip(np.rint(blended), 0, 255).astype(np.uint8)
        return rgb, coverage

    def bounds(self) -> MapBounds:
        """
        GPS coordinates of the rendered image's outer corners.
        """
        res = self._resolution_m
        west = self._left_px * res
        east = (self._left_px + self.width) * res
        north = self._top_px * res
        south = (self._top_px - self.height) * res
        ref_lat, ref_lon = self._reference
        return MapBounds(
            north_west=local_to_gps(west, north, ref_lat, ref_lon),
            north_east=local_to_gps(east, north, ref_lat, ref_lon),
            south_east=local_to_gps(east, south, ref_lat, ref_lon),
            south_west=local_to_gps(west, south, ref_lat, ref_lon),
        )

    def local_to_pixel(self, east: float, north: float) -> tuple[float, float]:
        """
        Canvas pixel (column, row) showing a local (east, north) point.
        """
        x, y, _ = self._local_to_canvas(0, 0) @ np.array([east, north, 1.0])
        return x, y

    def _local_to_canvas(self, x_offset: int, y_offset: int) -> np.ndarray:
        """
        3x3 transform from local metres to canvas pixel (x, y) inside a
        window starting at canvas pixel (x_offset, y_offset).
        """
        res = self._resolution_m
        # Pixel centres: x = east / res - left - 0.5, y = top - north / res - 0.5
        return np.array(
            [
                [1 / res, 0, -self._left_px - 0.5 - x_offset],
                [0, -1 / res, self._top_px - 0.5 - y_offset],
                [0, 0, 1],
            ]
        )

    def _grow_to_fit(self, corners: np.ndarray) -> None:
        """
        Expands the canvas so it covers the given local-metre corners.
        """
        res = self._resolution_m
        left = math.floor(corners[:, 0].min() / res)
        right = math.ceil(corners[:, 0].max() / res)
        top = math.ceil(corners[:, 1].max() / res)
        bottom = math.floor(corners[:, 1].min() / res)

        if not self.is_empty:
            left = min(left, self._left_px)
            right = max(right, self._left_px + self.width)
            top = max(top, self._top_px)
            bottom = min(bottom, self._top_px - self.height)

        new_width = right - left
        new_height = top - bottom
        if max(new_width, new_height) > self._max_size_px:
            raise ValueError(
                f"map would grow to {new_width} x {new_height} px, "
                f"over the {self._max_size_px} px limit"
            )
        if not self.is_empty and (new_width, new_height) == (self.width, self.height):
            return

        accum = np.zeros((new_height, new_width, 3), dtype=np.float32)
        weight = np.zeros((new_height, new_width), dtype=np.float32)
        if not self.is_empty:
            x = self._left_px - left
            y = top - self._top_px
            accum[y : y + self.height, x : x + self.width] = self._accum
            weight[y : y + self.height, x : x + self.width] = self._weight
        self._accum, self._weight = accum, weight
        self._left_px, self._top_px = left, top


def _feather_weights(height: int, width: int) -> np.ndarray:
    """
    Per-pixel blend weight: 1 at the photo's centre, falling to near 0 at its
    edges, so overlapping photos fade into each other instead of leaving seams.
    """
    ramp_x = np.minimum(np.arange(width) + 1, width - np.arange(width)) / (width / 2)
    ramp_y = np.minimum(np.arange(height) + 1, height - np.arange(height)) / (
        height / 2
    )
    return np.outer(ramp_y, ramp_x).astype(np.float32)
