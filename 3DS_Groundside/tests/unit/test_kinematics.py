"""Unit tests for src.kinematics."""

import math

import pytest

from src.kinematics import step_toward


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
