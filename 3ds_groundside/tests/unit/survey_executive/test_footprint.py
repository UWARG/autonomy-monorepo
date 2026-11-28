"""
Unit tests for the attitude-aware photo footprint.
"""

from __future__ import annotations

import math

import pytest
from gnc.src.orthomosaic import CameraSpec, derive_altitude, ground_footprint
from shapely.affinity import rotate
from shapely.geometry import Point

from src.survey_executive.footprint import photo_footprint
from tests.unit.survey_executive.factories import CAMERA, make_request

ALT_M = 30.0
HALF_V_RAD = CAMERA.vertical_fov_rad / 2.0
HALF_H_RAD = CAMERA.horizontal_fov_rad / 2.0


def footprint(**attitude: float):
    args = {"east_m": 0.0, "north_m": 0.0, "alt_m": ALT_M}
    args |= {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0} | attitude
    return photo_footprint(camera=CAMERA, **args)


def test_nadir_matches_gnc_ground_footprint() -> None:
    width_m, length_m = ground_footprint(ALT_M, CAMERA)
    polygon = footprint()
    assert polygon is not None
    min_e, min_n, max_e, max_n = polygon.bounds
    # Yaw 0: image width runs east-west, image length north-south.
    assert max_e - min_e == pytest.approx(width_m)
    assert max_n - min_n == pytest.approx(length_m)
    assert polygon.area == pytest.approx(width_m * length_m)
    assert polygon.centroid.x == pytest.approx(0.0, abs=1e-9)
    assert polygon.centroid.y == pytest.approx(0.0, abs=1e-9)


def test_nadir_footprint_follows_position() -> None:
    polygon = photo_footprint(120.0, -45.0, ALT_M, 0.0, 0.0, 0.0, CAMERA)
    assert polygon is not None
    assert polygon.centroid.x == pytest.approx(120.0)
    assert polygon.centroid.y == pytest.approx(-45.0)


def test_yaw_90_rotates_the_footprint() -> None:
    width_m, length_m = ground_footprint(ALT_M, CAMERA)
    polygon = footprint(yaw_deg=90.0)
    assert polygon is not None
    min_e, min_n, max_e, max_n = polygon.bounds
    assert max_e - min_e == pytest.approx(length_m)
    assert max_n - min_n == pytest.approx(width_m)


def test_image_top_faces_the_yaw_direction() -> None:
    # Pitch nose up shifts the view forward, so the shift shows which way
    # the image top faces: north at yaw 0, east at yaw 90 (clockwise).
    north = footprint(pitch_deg=10.0)
    east = footprint(yaw_deg=90.0, pitch_deg=10.0)
    assert north is not None and east is not None
    assert north.centroid.y > 1.0 and north.centroid.x == pytest.approx(0.0, abs=1e-9)
    assert east.centroid.x > 1.0 and east.centroid.y == pytest.approx(0.0, abs=1e-9)


def test_pitch_shifts_the_footprint_forward_exactly() -> None:
    pitch_rad = math.radians(15.0)
    polygon = footprint(pitch_deg=15.0)
    assert polygon is not None
    _, min_n, _, max_n = polygon.bounds
    # Pure pitch keeps the near and far edges in the pitch plane.
    assert max_n == pytest.approx(ALT_M * math.tan(pitch_rad + HALF_V_RAD))
    assert min_n == pytest.approx(ALT_M * math.tan(pitch_rad - HALF_V_RAD))
    # Looking ahead makes the far edge wider than the near edge.
    assert polygon.area > footprint().area


def test_roll_right_wing_down_looks_left() -> None:
    roll_rad = math.radians(12.0)
    polygon = footprint(roll_deg=12.0)
    assert polygon is not None
    min_e, _, max_e, _ = polygon.bounds
    assert max_e == pytest.approx(ALT_M * math.tan(HALF_H_RAD - roll_rad))
    assert min_e == pytest.approx(-ALT_M * math.tan(HALF_H_RAD + roll_rad))


def test_ray_above_the_horizon_gives_none() -> None:
    # The far corners leave the ground once pitch + half the vertical FOV
    # reaches 90 degrees.
    limit_deg = 90.0 - math.degrees(HALF_V_RAD)
    assert footprint(pitch_deg=limit_deg - 1.0) is not None
    assert footprint(pitch_deg=limit_deg + 1.0) is None
    assert footprint(roll_deg=-(90.0 - math.degrees(HALF_H_RAD) + 1.0)) is None
    assert footprint(pitch_deg=180.0) is None


def test_yaw_is_applied_last_as_a_clockwise_turn() -> None:
    # Yaw-then-pitch-then-roll means yaw turns the whole tilted footprint
    # about the drone, clockwise seen from above.
    tilted = footprint(roll_deg=8.0, pitch_deg=-6.0)
    yawed = footprint(roll_deg=8.0, pitch_deg=-6.0, yaw_deg=37.0)
    assert tilted is not None and yawed is not None
    expected = rotate(tilted, -37.0, origin=Point(0.0, 0.0))
    assert yawed.symmetric_difference(expected).area == pytest.approx(0.0, abs=1e-6)
    assert yawed.is_valid and len(yawed.exterior.coords) == 5
    assert yawed.exterior.is_ccw


def test_yaw_full_turn_and_negative_yaw_agree() -> None:
    base = footprint(yaw_deg=30.0, pitch_deg=5.0)
    turned = footprint(yaw_deg=390.0, pitch_deg=5.0)
    negative = footprint(yaw_deg=-330.0, pitch_deg=5.0)
    assert base is not None and turned is not None and negative is not None
    assert base.symmetric_difference(turned).area == pytest.approx(0.0, abs=1e-6)
    assert base.symmetric_difference(negative).area == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize(
    "overrides",
    [
        {"alt_m": 0.0},
        {"alt_m": -5.0},
        {"alt_m": math.nan},
        {"east_m": math.inf},
        {"roll_deg": math.nan},
        {"yaw_deg": math.inf},
        {"pitch_deg": None},  # JSON null from the photo source
    ],
)
def test_unplaceable_photo_gives_none(overrides: dict[str, float]) -> None:
    args = {
        "east_m": 0.0,
        "north_m": 0.0,
        "alt_m": ALT_M,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": 0.0,
    } | overrides
    assert photo_footprint(camera=CAMERA, **args) is None


def focal_px() -> float:
    focal_x = CAMERA.image_width_px / (2.0 * math.tan(HALF_H_RAD))
    focal_y = CAMERA.image_height_px / (2.0 * math.tan(HALF_V_RAD))
    return math.sqrt(focal_x * focal_y)


def pitched_gsd_m(north_m: float, east_m: float, pitch_rad: float) -> float:
    """
    GSD at a ground point, measured from a separate pinhole projection.

    Camera at (0, 0, ALT_M) pitched nose up; numeric Jacobian of ground to
    pixel, GSD = sqrt(ground area per pixel).
    """

    focal_x = CAMERA.image_width_px / (2.0 * math.tan(HALF_H_RAD))
    focal_y = CAMERA.image_height_px / (2.0 * math.tan(HALF_V_RAD))
    sp, cp = math.sin(pitch_rad), math.cos(pitch_rad)

    def to_pixel(n: float, e: float) -> tuple[float, float]:
        # Vector camera -> ground in NED; ground is ALT_M below.
        v = (n, e, ALT_M)
        depth = v[0] * sp + v[2] * cp  # along the optical axis
        forward = v[0] * cp - v[2] * sp  # along the image top
        return focal_x * v[1] / depth, focal_y * forward / depth

    def derivative(dn: float, de: float) -> tuple[float, float]:
        ahead = to_pixel(north_m + dn, east_m + de)
        behind = to_pixel(north_m - dn, east_m - de)
        return ((ahead[0] - behind[0]) / 2e-4, (ahead[1] - behind[1]) / 2e-4)

    x_n = derivative(1e-4, 0.0)
    x_e = derivative(0.0, 1e-4)
    pixels_per_m2 = abs(x_n[0] * x_e[1] - x_n[1] * x_e[0])
    return math.sqrt(1.0 / pixels_per_m2)


def test_level_photo_is_kept_whole_or_dropped_at_its_gsd() -> None:
    nadir_gsd_m = ALT_M / focal_px()
    full = footprint()
    kept = footprint(max_gsd_m=nadir_gsd_m * 1.0001)
    assert full is not None and kept is not None
    assert kept.symmetric_difference(full).area == pytest.approx(0.0, abs=1e-9)
    assert footprint(max_gsd_m=nadir_gsd_m * 0.9999) is None


def test_level_photo_exactly_at_the_gsd_limit_is_kept() -> None:
    # The planner flies at exactly the target GSD, so with max_gsd_ratio=1
    # float rounding in the depth limit must not drop the whole photo.
    camera = CameraSpec(
        image_width_px=4000,
        image_height_px=4000,
        horizontal_fov_rad=math.radians(90.0),
        vertical_fov_rad=math.radians(90.0),
    )
    alt_m = 0.01 * 4000 / (2.0 * math.tan(math.radians(45.0)))

    kept = photo_footprint(0.0, 0.0, alt_m, 0.0, 0.0, 0.0, camera, max_gsd_m=0.01)

    assert kept is not None
    assert kept.area == pytest.approx((2.0 * alt_m) ** 2)


def test_level_photo_gsd_matches_the_measured_gsd() -> None:
    assert pitched_gsd_m(5.0, 3.0, 0.0) == pytest.approx(ALT_M / focal_px(), rel=1e-6)


def test_tilted_photo_is_cut_where_gsd_reaches_the_limit() -> None:
    pitch_rad = math.radians(50.0)
    max_gsd_m = 1.5 * ALT_M / focal_px()
    full = footprint(pitch_deg=50.0)
    cut = footprint(pitch_deg=50.0, max_gsd_m=max_gsd_m)
    assert full is not None and cut is not None
    assert cut.within(full.buffer(1e-9))
    assert cut.area < full.area
    # Near edge is untouched; the far side is replaced by the cut line.
    assert cut.bounds[1] == pytest.approx(full.bounds[1])
    far_n = cut.bounds[3]
    assert far_n < full.bounds[3]
    far_points = [(e, n) for e, n in cut.exterior.coords if n == pytest.approx(far_n)]
    assert len(far_points) >= 2
    for east, north in far_points:
        assert pitched_gsd_m(north, east, pitch_rad) == pytest.approx(max_gsd_m, rel=1e-4)
    # A point just inside the cut is finer than the limit.
    assert pitched_gsd_m(far_n - 1.0, 0.0, pitch_rad) < max_gsd_m


def test_gsd_cut_bounds_the_area_of_any_tilt() -> None:
    max_gsd_m = 1.5 * ALT_M / focal_px()
    areas = []
    for pitch in (0.0, 30.0, 50.0, 60.0, 62.0):
        polygon = footprint(pitch_deg=pitch, max_gsd_m=max_gsd_m)
        areas.append(0.0 if polygon is None else polygon.area)
    # Without the cut, 60 degrees covers over 200 times the level area.
    assert max(areas) < 5.0 * areas[0]


def test_gsd_cut_follows_yaw_and_roll() -> None:
    max_gsd_m = 1.5 * ALT_M / focal_px()
    tilted = footprint(roll_deg=10.0, pitch_deg=40.0, max_gsd_m=max_gsd_m)
    yawed = footprint(roll_deg=10.0, pitch_deg=40.0, yaw_deg=120.0, max_gsd_m=max_gsd_m)
    assert tilted is not None and yawed is not None
    expected = rotate(tilted, -120.0, origin=Point(0.0, 0.0))
    assert yawed.symmetric_difference(expected).area == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("max_gsd_m", [0.0, -0.01, math.nan, math.inf])
def test_invalid_max_gsd_is_rejected(max_gsd_m: float) -> None:
    with pytest.raises(ValueError, match="max_gsd_m"):
        footprint(max_gsd_m=max_gsd_m)


def test_default_gsd_ratio_keeps_normal_tilt_and_cuts_steep_tilt() -> None:
    # Backs the max_gsd_ratio = 1.5 default in SurveyRequest.
    request = make_request()
    altitude_m = derive_altitude(request.camera, request.target_gsd_m)

    def kept_fraction(pitch_deg: float) -> float:
        full = photo_footprint(0.0, 0.0, altitude_m, 0.0, pitch_deg, 0.0, CAMERA)
        cut = photo_footprint(
            0.0, 0.0, altitude_m, 0.0, pitch_deg, 0.0, CAMERA,
            max_gsd_m=request.max_gsd_m,
        )
        assert full is not None
        return 0.0 if cut is None else cut.area / full.area

    for pitch_deg in (0.0, 10.0, 20.0):
        assert kept_fraction(pitch_deg) == pytest.approx(1.0)
    assert kept_fraction(30.0) < 0.6
    # At 60 degrees only the near strip, seen finely enough, is kept.
    assert kept_fraction(60.0) < 0.01
