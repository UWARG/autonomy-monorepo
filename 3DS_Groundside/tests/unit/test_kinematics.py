"""Unit tests for src.mock_drone_fleet.kinematics."""

import math

import pytest

from src.mock_drone_fleet.geometry import GeoPosition, from_local, to_local
from src.mock_drone_fleet.kinematics import step_toward, step_toward_geo


def test_happy_case_moves_max_step_along_line_to_target() -> None:
    # 3-4-5 triangle: target is 50 m away, drone covers 2 m this step.
    new_position = step_toward((0.0, 0.0, 0.0), (30.0, 40.0, 0.0), speed=2.0, dt=1.0)

    assert new_position == pytest.approx((1.2, 1.6, 0.0))
    assert math.dist((0.0, 0.0, 0.0), new_position) == pytest.approx(2.0)


def test_distance_less_than_max_step_snaps_to_target() -> None:
    # Target is 1 m away but the drone could cover 2 m, so it must not overshoot.
    target = (1.0, 0.0, 0.0)

    assert step_toward((0.0, 0.0, 0.0), target, speed=2.0, dt=1.0) == target


def test_already_at_target_stays_put() -> None:
    target = (5.0, 5.0, 5.0)

    assert step_toward(target, target, speed=2.0, dt=1.0) == target


@pytest.mark.parametrize(
    ("speed", "dt"),
    [
        (-1.0, 1.0),
        (1.0, -1.0),
        (-1.0, -1.0),
    ],
)
def test_negative_speed_or_dt_raises(speed: float, dt: float) -> None:
    with pytest.raises(ValueError):
        step_toward((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), speed=speed, dt=dt)


HOME = GeoPosition(43.4723, -80.5449, 0.0)


def test_one_degree_of_latitude_is_about_111_km() -> None:
    north = to_local(HOME, GeoPosition(44.4723, -80.5449, 0.0))[1]

    assert north == pytest.approx(111_195, rel=1e-4)


def test_longitude_degrees_shrink_with_latitude() -> None:
    # At ~43.5° N a degree of longitude is only cos(43.5°) ≈ 0.725 of one at the equator.
    east = to_local(HOME, GeoPosition(43.4723, -79.5449, 0.0))[0]

    assert east == pytest.approx(111_195 * math.cos(math.radians(43.4723)), rel=1e-4)


def test_local_and_geo_conversions_round_trip() -> None:
    offset = (123.4, -56.7, 15.0)

    assert to_local(HOME, from_local(HOME, offset)) == pytest.approx(offset, abs=1e-6)


def test_step_toward_geo_moves_speed_times_dt_metres_toward_target() -> None:
    target = from_local(HOME, (30.0, 40.0, 0.0))  # 50 m away

    position = step_toward_geo(HOME, target, speed=2.0, dt=1.0)

    assert to_local(HOME, position) == pytest.approx((1.2, 1.6, 0.0), abs=1e-6)


def test_step_toward_geo_returns_target_exactly_on_arrival() -> None:
    target = from_local(HOME, (1.0, 0.0, 0.0))  # 1 m away, within one 2 m step

    assert step_toward_geo(HOME, target, speed=2.0, dt=1.0) is target
