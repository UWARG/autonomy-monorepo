import math

import pytest

from orthomosaic.footprint import (
    derive_altitude,
    ground_footprint,
    ground_sample_distance,
    spacings,
)
from orthomosaic.models import CameraSpec

ARDUCAM = CameraSpec(
    image_width_px=1280,
    image_height_px=720,
    horizontal_fov_rad=math.radians(80),
    vertical_fov_rad=math.radians(55),
)


def test_derive_altitude_matches_hand_calculation():
    altitude = derive_altitude(ARDUCAM, target_gsd_m=0.02)
    expected = min(
        0.02 * 1280 / (2.0 * math.tan(math.radians(40))),
        0.02 * 720 / (2.0 * math.tan(math.radians(27.5))),
    )
    assert altitude == pytest.approx(expected, rel=1e-9)
    assert altitude == pytest.approx(13.83, abs=0.01)


def test_derive_altitude_is_limiting_axis():
    altitude = derive_altitude(ARDUCAM, target_gsd_m=0.02)
    gsd_x, gsd_y = ground_sample_distance(altitude, ARDUCAM)
    assert gsd_x <= 0.02
    assert gsd_y <= 0.02
    assert max(gsd_x, gsd_y) == pytest.approx(0.02, rel=1e-9)


def test_ground_footprint_scales_with_altitude():
    width_a, length_a = ground_footprint(10.0, ARDUCAM)
    width_b, length_b = ground_footprint(20.0, ARDUCAM)
    assert width_b == pytest.approx(2.0 * width_a)
    assert length_b == pytest.approx(2.0 * length_a)


def test_spacings():
    capture, line = spacings(
        footprint_width_m=23.21,
        footprint_length_m=14.40,
        forward_overlap=0.75,
        side_overlap=0.65,
    )
    assert capture == pytest.approx(3.60, abs=0.01)
    assert line == pytest.approx(8.12, abs=0.01)


def test_square_camera_symmetric_altitude():
    camera = CameraSpec(
        image_width_px=640,
        image_height_px=640,
        horizontal_fov_rad=math.radians(60),
        vertical_fov_rad=math.radians(60),
    )
    assert derive_altitude(camera, 0.02) == pytest.approx(
        0.02 * 640 / (2.0 * math.tan(math.radians(30)))
    )


def test_derive_altitude_rejects_nonpositive_gsd():
    with pytest.raises(ValueError):
        derive_altitude(ARDUCAM, 0.0)
    with pytest.raises(ValueError):
        derive_altitude(ARDUCAM, -1.0)


@pytest.mark.parametrize(
    "camera",
    [
        CameraSpec(0, 720, math.radians(80), math.radians(55)),
        CameraSpec(-1, 720, math.radians(80), math.radians(55)),
        CameraSpec(1280, 0, math.radians(80), math.radians(55)),
        CameraSpec(1280, 720, 0.0, math.radians(55)),
        CameraSpec(1280, 720, math.pi, math.radians(55)),
        CameraSpec(1280, 720, math.radians(80), 0.0),
        CameraSpec(1280, 720, math.radians(80), math.pi),
    ],
)
def test_ground_footprint_rejects_invalid_camera(camera):
    with pytest.raises(ValueError):
        ground_footprint(10.0, camera)


@pytest.mark.parametrize("overlap", [-0.1, 1.0, 1.5])
def test_spacings_rejects_invalid_overlaps(overlap):
    with pytest.raises(ValueError):
        spacings(23.0, 14.0, forward_overlap=overlap, side_overlap=0.65)
    with pytest.raises(ValueError):
        spacings(23.0, 14.0, forward_overlap=0.75, side_overlap=overlap)