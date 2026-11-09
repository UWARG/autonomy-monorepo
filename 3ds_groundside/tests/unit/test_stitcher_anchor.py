"""
Tests for placing a cluster on the map from GPS, with hand-made poses so no
feature matching is involved.
"""

import numpy as np
import pytest

from src.image_stitcher.camera import Camera
from src.image_stitcher.geo import scale_of
from src.image_stitcher.message import ImageMessage
from src.image_stitcher.stitcher import ImageStitcher, _Photo

ALTITUDE_M = 25.0


@pytest.fixture(scope="module")
def camera():
    return Camera.load()


def add_photo(stitcher, pose, east):
    image = np.zeros((480, 640, 3), np.uint8)
    message = ImageMessage(
        43.0,
        -80.0,
        ALTITUDE_M,
        image,
        0.0,
        0.0,
        image_id=f"photo_{len(stitcher._photos)}",
    )
    stitcher._photos.append(_Photo(message, None, east, 0.0, cluster=0, pose=pose))


def stitcher_with(camera, frame_x_px, gps_east_m):
    """
    A stitcher holding one cluster of photos at these frame x positions (in
    pixels) and these GPS east positions (in metres).
    """
    stitcher = ImageStitcher(matcher=object(), camera=camera)
    for x, east in zip(frame_x_px, gps_east_m):
        pose = np.eye(3)
        pose[0, 2] = x
        add_photo(stitcher, pose, east)
    return stitcher


def placed_scale(stitcher):
    return scale_of(stitcher.placements()[0][1])


def test_gps_noise_on_nearby_photos_does_not_change_the_scale(camera):
    # The images say these photos are within a metre or so of each other,
    # but noisy GPS spreads them over 10 m, enough to pass the GPS check.
    stitcher = stitcher_with(camera, [0, 5, 10, 15, 20], [-6, 4, -3, 6, -1])
    assert placed_scale(stitcher) == pytest.approx(camera.metres_per_px(ALTITUDE_M))


def test_photos_really_spread_out_are_fitted_to_gps(camera):
    # About 45 m of real separation from the mean, with GPS reading 5% further
    # than the calibration: the fit should follow the GPS.
    metres_per_px = camera.metres_per_px(ALTITUDE_M)
    frame_x_px = [0, 500, 1000, 1500, 2000]
    gps_east_m = [1.05 * x * metres_per_px for x in frame_x_px]
    stitcher = stitcher_with(camera, frame_x_px, gps_east_m)
    assert placed_scale(stitcher) == pytest.approx(1.05 * metres_per_px)
