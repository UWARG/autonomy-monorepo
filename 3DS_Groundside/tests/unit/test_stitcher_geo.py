import math

import numpy as np
import pytest

from src.image_stitcher.geo import (
    FLIP_Y,
    apply,
    fit_similarity,
    footprint_corners,
    gps_to_local,
    ground_sample_distance,
    local_to_gps,
    nadir_image_to_local,
    scale_of,
)


def test_gps_local_round_trip():
    ref = (43.4723, -80.5449)
    lat, lon = local_to_gps(123.4, -56.7, *ref)
    east, north = gps_to_local(lat, lon, *ref)
    assert east == pytest.approx(123.4, abs=1e-6)
    assert north == pytest.approx(-56.7, abs=1e-6)


def test_one_degree_of_latitude_is_about_111_km():
    _, north = gps_to_local(44.0, -80.0, 43.0, -80.0)
    assert north == pytest.approx(111_320, rel=0.01)


def test_ground_sample_distance():
    # 90 degree FOV from 10 m sees 20 m of ground across 1000 px.
    assert ground_sample_distance(10.0, 90.0, 1000) == pytest.approx(0.02)
    with pytest.raises(ValueError):
        ground_sample_distance(0.0, 90.0, 1000)


def test_nadir_heading_zero_keeps_north_up():
    transform = nadir_image_to_local(3, 3, 1.0, 0.0, 10.0, 20.0)
    corners = footprint_corners(transform, 3, 3)
    # Top left pixel is north west of the centre.
    np.testing.assert_allclose(corners[0], [9.0, 21.0])
    np.testing.assert_allclose(corners[2], [11.0, 19.0])


def test_nadir_heading_ninety_points_image_top_east():
    transform = nadir_image_to_local(3, 3, 1.0, 90.0, 0.0, 0.0)
    # Top centre pixel (1, 0) is one pixel "up" in the image, which is east.
    np.testing.assert_allclose(apply(transform, [[1.0, 0.0]])[0], [1.0, 0.0], atol=1e-9)


def test_fit_similarity_recovers_rotation_scale_shift():
    angle = math.radians(37)
    truth = np.array(
        [
            [2.5 * math.cos(angle), -2.5 * math.sin(angle), 4.0],
            [2.5 * math.sin(angle), 2.5 * math.cos(angle), -7.0],
            [0, 0, 1],
        ]
    )
    source = np.random.default_rng(0).random((20, 2)) * 100
    fitted = fit_similarity(source, apply(truth, source))
    np.testing.assert_allclose(fitted, truth, atol=1e-9)
    assert scale_of(fitted) == pytest.approx(2.5)


def test_fit_similarity_through_flip_handles_image_axes():
    # Image pixels -> local metres always includes a y flip (image y is down,
    # north is up). Fitting in flipped coordinates must recover it exactly.
    truth = nadir_image_to_local(640, 480, 0.05, 123.0, 30.0, -20.0)
    pixels = np.random.default_rng(1).random((10, 2)) * [640, 480]
    fitted = fit_similarity(apply(FLIP_Y, pixels), apply(truth, pixels)) @ FLIP_Y
    np.testing.assert_allclose(fitted, truth, atol=1e-9)


@pytest.mark.parametrize("turns", [0, 1, 2, 3])
def test_unrotate_points_inverts_rot90(turns):
    from src.image_stitcher.features import unrotate_points

    image = np.zeros((4, 6), dtype=np.uint8)
    image[1, 4] = 255  # the marked pixel is at x=4, y=1
    rotated = np.rot90(image, turns)
    y, x = np.argwhere(rotated == 255)[0]
    original = unrotate_points(np.array([[x, y]], dtype=np.float32), turns, 6, 4)
    np.testing.assert_array_equal(original, [[4, 1]])
