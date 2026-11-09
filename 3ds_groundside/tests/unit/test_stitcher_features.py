"""
Tests for turning photos north-up before feature extraction. No XFeat model
is loaded; extraction is replaced with made-up keypoints.
"""

import numpy as np
import pytest
import torch

from src.image_stitcher.features import (
    EDGE_MARGIN_PX,
    Features,
    XFeatMatcher,
    quarter_turn,
    rotate_north_up,
)
from src.image_stitcher.geo import apply

WIDTH, HEIGHT = 64, 48


def marked_image(x, y):
    image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    image[y - 1 : y + 2, x - 1 : x + 2] = 255
    return image


def brightest(image):
    ys, xs = np.nonzero(image[..., 0] > 127)
    return np.array([xs.mean(), ys.mean()])


@pytest.mark.parametrize("yaw_deg", [0.0, 37.0, 90.0, 180.0, 233.0, 359.0])
def test_points_map_back_to_the_original_pixels(yaw_deg):
    turned, to_original = rotate_north_up(marked_image(50, 12), yaw_deg)
    np.testing.assert_allclose(
        apply(to_original, [brightest(turned)])[0], [50, 12], atol=1.0
    )


def test_top_of_image_ends_up_on_the_side_it_faces():
    # With yaw 90 the top of the image faces east, and north-up puts east on
    # the right.
    turned, _ = rotate_north_up(marked_image(32, 2), 90.0)
    x, y = brightest(turned)
    assert x > 0.9 * HEIGHT
    assert y == pytest.approx((WIDTH - 1) / 2, abs=2)


@pytest.mark.parametrize("turns", [0, 1, 2, 3, -1, 4])
def test_quarter_turn_points_map_back_exactly(turns):
    turned, to_original = quarter_turn(marked_image(50, 12), turns)
    np.testing.assert_allclose(
        apply(to_original, [brightest(turned)])[0], [50, 12], atol=1e-9
    )


def test_quarter_turn_is_clockwise_and_loses_nothing():
    image = marked_image(32, 2)
    turned, _ = quarter_turn(image, 1)
    assert turned.shape[:2] == (WIDTH, HEIGHT)
    assert brightest(turned)[0] > 0.9 * HEIGHT
    assert np.sort(turned, axis=None).tolist() == np.sort(image, axis=None).tolist()


def test_near_quarter_turn_keeps_keypoints_at_the_photo_edge():
    # No empty corners, so edge keypoints are real. They also pin down
    # rotation and scale best.
    image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    features = matcher_finding([[1.0, 1.0], [30.0, 30.0]]).extract(image, 92.0)
    assert features.descriptors.tolist() == [0, 1]


@pytest.mark.parametrize("yaw_deg", [12.0, 45.0])
def test_yaw_far_from_a_quarter_turn_is_really_rotated(yaw_deg):
    # These need the full rotation, whose canvas is larger than the photo.
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    seen = []
    matcher = XFeatMatcher.__new__(XFeatMatcher)

    def record(rgb):
        seen.append(rgb.shape)
        return Features(np.zeros((0, 2), np.float32), torch.zeros(0))

    matcher._extract_one = record
    matcher.extract(image, yaw_deg)
    assert seen[0][0] > 480 and seen[0][1] > 480


def test_turned_canvas_holds_the_whole_photo():
    image = np.full((HEIGHT, WIDTH, 3), 200, dtype=np.uint8)
    turned, _ = rotate_north_up(image, 45.0)
    # All the photo's brightness is still there, apart from edge blending.
    assert turned[..., 0].sum() == pytest.approx(image[..., 0].sum(), rel=0.02)


def matcher_finding(points_in_turned_image):
    """
    An XFeatMatcher whose extraction "finds" these points, each with a
    descriptor holding its own index.
    """
    matcher = XFeatMatcher.__new__(XFeatMatcher)
    points = np.array(points_in_turned_image, dtype=np.float32)
    matcher._extract_one = lambda rgb: Features(
        keypoints=points, descriptors=torch.arange(len(points))
    )
    return matcher


def test_keypoints_in_the_empty_corners_are_dropped():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    turned, to_original = rotate_north_up(image, 45.0)
    height, width = turned.shape[:2]
    centre = [width / 2, height / 2]
    # Index 1 is the empty corner of the canvas, the rest are on the photo.
    matcher = matcher_finding([centre, [2.0, 2.0], [width / 2 + 40, height / 2]])

    features = matcher.extract(image, 45.0)

    assert features.descriptors.tolist() == [0, 2]
    np.testing.assert_allclose(
        features.keypoints[0], apply(to_original, [centre])[0], atol=1e-3
    )
    assert (features.keypoints >= EDGE_MARGIN_PX).all()


def test_keypoints_near_the_edge_of_a_rotated_photo_are_dropped():
    # At yaw 45 the photo's edges run diagonally across the canvas, so these
    # are found by mapping points from the photo's own pixels forwards.
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    _, to_original = rotate_north_up(image, 45.0)
    to_turned = np.linalg.inv(to_original)
    just_inside_margin = [EDGE_MARGIN_PX + 2.0, 240.0]
    within_margin = [EDGE_MARGIN_PX - 8.0, 240.0]
    matcher = matcher_finding(apply(to_turned, [just_inside_margin, within_margin]))

    features = matcher.extract(image, 45.0)

    assert features.descriptors.tolist() == [0]
