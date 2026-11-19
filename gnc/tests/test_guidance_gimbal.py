import math

import pytest

from guidance import GimbalModel, GimbalState
from guidance.gimbal import (
    body_angles,
    is_settled,
    look_angles,
    rate_command,
    wrap_angle,
    yaw_for_limits,
)


def test_look_angles_cardinal_directions():
    yaw, pitch = look_angles((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    assert yaw == pytest.approx(math.pi / 2.0)
    assert pitch == pytest.approx(0.0)

    yaw, _ = look_angles((0.0, 0.0, 0.0), (0.0, 10.0, 0.0))
    assert yaw == pytest.approx(0.0)

    _, pitch = look_angles((0.0, 0.0, 0.0), (10.0, 0.0, 10.0))
    assert pitch == pytest.approx(math.pi / 4.0)


def test_wrap_angle_handles_branch_cut():
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_body_angles_subtract_aircraft_yaw():
    model = GimbalModel()
    yaw, pitch, roll = body_angles(0.0, 0.0, math.pi / 2.0, model)
    assert yaw == pytest.approx(-math.pi / 2.0)
    assert pitch == pytest.approx(0.0)
    assert roll == pytest.approx(0.0)


def test_yaw_for_limits_turns_aircraft_only_when_needed():
    model = GimbalModel(yaw_min_rad=-math.pi / 2.0, yaw_max_rad=math.pi / 2.0)
    assert yaw_for_limits(0.4, 0.0, model) == pytest.approx(0.0)
    yaw = yaw_for_limits(math.pi, 0.0, model)
    assert -math.pi / 2.0 <= wrap_angle(math.pi - yaw) <= math.pi / 2.0


def test_settle_checks_angle_and_rate():
    model = GimbalModel(
        settle_tolerance_rad=0.02, settle_rate_tolerance_radps=0.02
    )
    settled = GimbalState(yaw_rad=0.01, pitch_rad=0.01, roll_rad=0.01)
    assert is_settled(settled, 0.0, 0.0, 0.0, model)
    moving = GimbalState(
        yaw_rad=0.01, pitch_rad=0.01, roll_rad=0.01, yaw_rate_radps=0.1
    )
    assert not is_settled(moving, 0.0, 0.0, 0.0, model)
    assert not is_settled(
        GimbalState(yaw_rad=0.1, pitch_rad=0.01, roll_rad=0.01),
        0.0,
        0.0,
        0.0,
        model,
    )
    assert not is_settled(
        GimbalState(yaw_rad=0.01, pitch_rad=0.01, roll_rad=0.1),
        0.0,
        0.0,
        0.0,
        model,
    )
    assert not is_settled(
        GimbalState(yaw_rad=0.01, pitch_rad=0.01, roll_rad=0.01, roll_rate_radps=0.1),
        0.0,
        0.0,
        0.0,
        model,
    )


def test_rate_command_limits_rate_and_acceleration():
    rate = rate_command(1.0, 0.0, 1.0, 2.0, 0.1, 10.0)
    assert rate == pytest.approx(0.2)
    rate = rate_command(0.01, 0.0, 1.0, 2.0, 0.1, 10.0)
    assert rate == pytest.approx(0.1)


def test_invalid_gimbal_limits_raise():
    with pytest.raises(ValueError, match="yaw limits"):
        GimbalModel(yaw_min_rad=1.0, yaw_max_rad=0.0)
    with pytest.raises(ValueError, match="roll limits"):
        GimbalModel(roll_min_rad=1.0, roll_max_rad=0.0)


def test_aircraft_yaw_rate_prevents_settle():
    model = GimbalModel()
    measured = GimbalState()
    assert is_settled(measured, 0.0, 0.0, 0.0, model)
    assert not is_settled(
        measured, 0.0, 0.0, 0.0, model, aircraft_yaw_rate_radps=0.1
    )
