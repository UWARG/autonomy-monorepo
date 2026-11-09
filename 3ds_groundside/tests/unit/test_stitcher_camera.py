import cv2
import numpy as np
import pytest

from src.image_stitcher.camera import Camera
from src.image_stitcher.geo import apply
from src.image_stitcher.synthetic import make_ground, take_photo

# Strong barrel distortion, so errors in undistortion are easy to see.
BARREL = Camera(
    np.array([[500.0, 0.0, 330.0], [0.0, 500.0, 230.0], [0.0, 0.0, 1.0]]),
    np.array([-0.3, 0.1, 0.0, 0.0, 0.0]),
    640,
    480,
)


def test_loads_bundled_calibration():
    camera = Camera.load()
    assert (camera.width, camera.height) == (640, 480)
    # Undistorting zooms in slightly past the raw 542.6 px focal length.
    assert 540 < camera.focal_px < 560
    # Principal point is off the image centre for this camera.
    assert camera.principal_point[1] < 239.5 - 30


def test_undistort_points_matches_ideal_pinhole():
    # Ground points seen from 1 m up, projected through the distorted lens.
    rng = np.random.default_rng(0)
    ground = np.column_stack([rng.uniform(-0.4, 0.4, (50, 2)), np.ones(50)])
    raw, _ = cv2.projectPoints(
        ground, np.zeros(3), np.zeros(3), BARREL._matrix, BARREL._distortion
    )
    expected = ground[:, :2] * BARREL.focal_px + BARREL.principal_point
    np.testing.assert_allclose(
        BARREL.undistort_points(raw.reshape(-1, 2)), expected, atol=0.05
    )


def test_undistorted_photo_is_a_pinhole_photo():
    # The same view through an ideal pinhole camera with the undistorted
    # camera's focal length and principal point.
    pinhole = Camera(
        np.array(
            [
                [BARREL.focal_px, 0.0, BARREL.principal_point[0]],
                [0.0, BARREL.focal_px, BARREL.principal_point[1]],
                [0.0, 0.0, 1.0],
            ]
        ),
        np.zeros(5),
        640,
        480,
    )
    ground = make_ground(width_px=1600, height_px=1200)
    distorted, transform = take_photo(ground, BARREL, 40.0, -30.0, 20.0, 35.0)
    ideal, ideal_transform = take_photo(ground, pinhole, 40.0, -30.0, 20.0, 35.0)
    np.testing.assert_allclose(transform, ideal_transform)

    undistorted = BARREL.undistort(distorted)
    error = np.abs(undistorted.astype(int) - ideal.astype(int)).mean()
    # Resampling twice blurs a little; the raw distorted photo is far off.
    assert error < 6.0
    assert np.abs(distorted.astype(int) - ideal.astype(int)).mean() > 4 * error


def test_undistorted_photo_has_no_empty_corners():
    white = np.full((480, 640, 3), 255, np.uint8)
    assert BARREL.undistort(white).min() > 200


def test_ground_resolution_scales_with_altitude():
    assert BARREL.metres_per_px(25.0) == pytest.approx(25.0 / BARREL.focal_px)


def test_wrong_image_size_is_refused():
    assert not BARREL.matches(np.zeros((240, 320, 3), np.uint8))
    with pytest.raises(ValueError):
        BARREL.undistort(np.zeros((240, 320, 3), np.uint8))


def test_principal_point_is_straight_below_the_drone():
    ground = make_ground(width_px=1600, height_px=1200)
    _, image_to_local = take_photo(ground, BARREL, 40.0, -30.0, 20.0, 0.0)
    # The principal point is straight below the camera.
    np.testing.assert_allclose(
        apply(image_to_local, [BARREL.principal_point])[0], [40.0, -30.0]
    )
