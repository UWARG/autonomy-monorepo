"""
Tests for placing photos and clusters from GPS, yaw and matches, with
hand-made poses and matches so no feature matching is involved.
"""

import math

import numpy as np
import pytest

from src.image_stitcher.camera import Camera
from src.image_stitcher.geo import apply, image_axes_to_local, scale_of
from src.image_stitcher.message import ImageMessage
from src.image_stitcher.stitcher import ImageStitcher, _Link, _Match, _Photo

ALTITUDE_M = 25.0


@pytest.fixture(scope="module")
def camera():
    return Camera.load()


def add_photo(stitcher, pose, east, yaw_deg):
    image = np.zeros((480, 640, 3), np.uint8)
    message = ImageMessage(
        43.0,
        -80.0,
        ALTITUDE_M,
        image,
        0.0,
        0.0,
        yaw_deg,
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
        add_photo(stitcher, pose, east, 0.0)
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


@pytest.mark.parametrize("yaw_deg", [0.0, 90.0, 200.0, 359.0, -90.0, -180.0])
def test_lone_photo_is_oriented_by_its_yaw(camera, yaw_deg):
    stitcher = ImageStitcher(matcher=object(), camera=camera)
    add_photo(stitcher, np.eye(3), 0.0, yaw_deg)
    [(_, placed)] = stitcher.placements()
    # % 360 so a negative yaw is checked against its 0 to 360 equivalent.
    expected = camera.metres_per_px(ALTITUDE_M) * image_axes_to_local(yaw_deg % 360)
    np.testing.assert_allclose(placed[:2, :2], expected, atol=1e-9)


def test_photo_rotated_in_its_cluster_still_gets_its_own_yaw(camera):
    # The second photo is turned 30 degrees within the cluster frame relative
    # to the first, and its yaw says the same thing.
    angle = math.radians(30)
    # Image y points down, so this matrix turns pixels clockwise on screen.
    pose = np.array(
        [
            [math.cos(angle), -math.sin(angle), 5.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    stitcher = ImageStitcher(matcher=object(), camera=camera)
    add_photo(stitcher, np.eye(3), 0.0, 10.0)
    add_photo(stitcher, pose, 0.0, 40.0)
    metres_per_px = camera.metres_per_px(ALTITUDE_M)
    for (message, placed), yaw_deg in zip(stitcher.placements(), [10.0, 40.0]):
        assert message.yaw_deg == yaw_deg
        expected = metres_per_px * image_axes_to_local(yaw_deg)
        np.testing.assert_allclose(placed[:2, :2], expected, atol=1e-9)


def test_scaled_pose_does_not_change_the_orientation(camera):
    # A refined pose can have a scale other than 1. These two photos say the
    # frame is at 10 and 20 degrees, and must count equally whatever the scale.
    angle = math.radians(30)
    scale = 2.0
    pose = np.array(
        [
            [scale * math.cos(angle), -scale * math.sin(angle), 5.0],
            [scale * math.sin(angle), scale * math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    stitcher = ImageStitcher(matcher=object(), camera=camera)
    add_photo(stitcher, np.eye(3), 0.0, 10.0)
    add_photo(stitcher, pose, 0.0, 50.0)
    placed = stitcher.placements()[0][1]
    rotation = placed[:2, :2] / scale_of(placed)
    np.testing.assert_allclose(rotation, image_axes_to_local(15.0), atol=1e-9)


def test_wrong_match_is_refused_when_gps_and_images_disagree(camera):
    # GPS spreads these photos 44 m but the images show them almost together,
    # so the cluster is not fitted to GPS. A match landing 200 m from the new
    # photo's GPS must still be refused.
    stitcher = stitcher_with(camera, [0, 5], [0, 44])
    far_away_px = 200 / camera.metres_per_px(ALTITUDE_M)
    pose = np.eye(3)
    pose[0, 2] = far_away_px
    photo = _Photo(stitcher._photos[0].message, None, 22.0, 0.0)
    assert not stitcher._agrees_with_gps(photo, 0, pose)
    assert stitcher._agrees_with_gps(photo, 0, np.eye(3))


def test_correct_match_is_kept_when_yaw_has_a_constant_offset(camera):
    # Yaw reads 180 degrees out, so this cluster is drawn upside down. A
    # correct match 10 m from its centre must not be refused for landing on
    # the wrong side of it.
    stitcher = ImageStitcher(matcher=object(), camera=camera)
    for x, east in zip([0, 5], [0, 44]):
        pose = np.eye(3)
        pose[0, 2] = x
        add_photo(stitcher, pose, east, 180.0)
    pose = np.eye(3)
    pose[0, 2] = 2.5 + 10 / camera.metres_per_px(ALTITUDE_M)
    photo = _Photo(stitcher._photos[0].message, None, 32.0, 0.0)
    assert stitcher._agrees_with_gps(photo, 0, pose)


def test_yaws_either_side_of_north_average_to_north(camera):
    stitcher = ImageStitcher(matcher=object(), camera=camera)
    add_photo(stitcher, np.eye(3), 0.0, 350.0)
    add_photo(stitcher, np.eye(3), 0.0, 10.0)
    placed = stitcher.placements()[0][1]
    expected = camera.metres_per_px(ALTITUDE_M) * image_axes_to_local(0.0)
    np.testing.assert_allclose(placed[:2, :2], expected, atol=1e-9)


def test_yaws_either_side_of_south_average_to_south(camera):
    # With yaw from -180 to 180, these are 20 degrees apart, not 340.
    stitcher = ImageStitcher(matcher=object(), camera=camera)
    add_photo(stitcher, np.eye(3), 0.0, 170.0)
    add_photo(stitcher, np.eye(3), 0.0, -170.0)
    placed = stitcher.placements()[0][1]
    expected = camera.metres_per_px(ALTITUDE_M) * image_axes_to_local(180.0)
    np.testing.assert_allclose(placed[:2, :2], expected, atol=1e-9)


def shifted(x_px):
    pose = np.eye(3)
    pose[0, 2] = x_px
    return pose


def place_against_a_and_b(camera, error_vs_b_px):
    """
    Photos a and b are placed 300 px apart and the new photo truly sits at
    150 px. Its match with a is right; its match with b is off by a constant
    shift, so it still passed RANSAC on its own. Places the new photo and
    refines.
    """
    stitcher = ImageStitcher(matcher=object(), camera=camera)
    add_photo(stitcher, shifted(0.0), 0.0, 0.0)
    add_photo(stitcher, shifted(300.0), 0.0, 0.0)
    a, b = stitcher._photos
    points = np.random.default_rng(0).uniform([0, 0], [640, 480], (100, 2))
    stitcher._links.append(_Link(a, b, points + [300, 0], points))

    def match(other, points_new, points_other):
        return _Match(other, points_new, points_other, apply(other.pose, points_other))

    right = match(a, points[:60], points[:60] + [150, 0])
    off = match(b, points[60:], points[60:] + [-150, 0] + error_vs_b_px)
    photo = _Photo(a.message, None, 0.0, 0.0)
    stitcher._place(photo, {0: [right, off]})
    stitcher._photos.append(photo)
    stitcher._unrefined.add(0)
    stitcher.refine()
    return stitcher, photo


def test_conflicting_match_is_left_out_of_refinement(camera):
    # About one repeat of repeating ground (9 m) off.
    stitcher, photo = place_against_a_and_b(camera, [200, 0])
    a = stitcher._photos[0]
    assert [link.b for link in stitcher._links if link.a is photo] == [a]
    np.testing.assert_allclose(photo.pose, shifted(150.0), atol=1e-6)


def test_match_off_by_drift_is_still_refined(camera):
    # Before refinement, correct matches can disagree by tens of pixels.
    stitcher, photo = place_against_a_and_b(camera, [20, 15])
    assert len([link for link in stitcher._links if link.a is photo]) == 2


def test_fit_that_no_match_agrees_with_is_refused(camera):
    # A safety net that real matches rarely reach, since each one already
    # fits a single transform. Here each is mostly scattered points: the
    # joint fit still finds the pose from both together, but neither match
    # agrees with it, so the photo would join with nothing to refine it.
    stitcher = ImageStitcher(matcher=object(), camera=camera)
    add_photo(stitcher, shifted(0.0), 0.0, 0.0)
    a = stitcher._photos[0]
    rng = np.random.default_rng(1)
    matches = []
    for _ in range(2):
        points = rng.uniform([0, 0], [640, 480], (100, 2))
        others = points + [150, 0]
        scatter = rng.uniform(100, 400, (60, 2)) * rng.choice([-1, 1], (60, 2))
        others[40:] += scatter
        matches.append(_Match(a, points, others, others))
    assert stitcher._estimate_pose(matches) is None
