"""Gate 0 -- body FRD velocity -> MAVROS FLU TwistStamped sign mapping."""
import pytest

from mavros_setpoint import body_velocity_to_flu


def test_forward_is_unchanged():
    t = body_velocity_to_flu(1.5, 0.0, 0.0, 0.0)
    assert t.linear_x == 1.5
    assert t.linear_y == 0.0
    assert t.linear_z == 0.0
    assert t.angular_z == 0.0


def test_right_flips_to_negative_left():
    t = body_velocity_to_flu(0.0, 1.0, 0.0, 0.0)
    assert t.linear_y == -1.0  # FRD right -> FLU left


def test_down_flips_to_negative_up():
    t = body_velocity_to_flu(0.0, 0.0, 1.0, 0.0)
    assert t.linear_z == -1.0  # FRD down -> FLU up


def test_yaw_rate_flips_sign():
    t = body_velocity_to_flu(0.0, 0.0, 0.0, 0.8)
    assert t.angular_z == -0.8  # NED clockwise -> ENU counter-clockwise


def test_slew_limits_step_commands():
    """Speeding up ramps at dv_max per tick; slowing down and reversals are
    INSTANT. Step-up commands make the FC overshoot the commanded speed (a
    spurious closing-rate BRAKE on a nominal approach); but a lagged slow-down
    makes the vehicle brake late and penetrate the ring -- both observed in
    SITL, hence the asymmetry.
    """
    from mavros_setpoint import slew

    # speeding up: ramped
    assert slew(0.0, 1.5, 0.05) == 0.05
    assert slew(0.05, 1.5, 0.05) == pytest.approx(0.10)
    assert slew(-0.2, -1.5, 0.05) == pytest.approx(-0.25)  # away from zero, negative
    assert slew(0.3, 0.32, 0.05) == 0.32  # within the limit: exact
    # slowing down: instant (braking must never lag the demand)
    assert slew(1.5, 0.8, 0.05) == 0.8
    assert slew(1.5, 0.0, 0.05) == 0.0
    assert slew(-1.0, -0.2, 0.05) == -0.2
    # sign reversal (recede): instant -- overshooting a recede is the safe direction
    assert slew(0.4, -1.0, 0.05) == -1.0
