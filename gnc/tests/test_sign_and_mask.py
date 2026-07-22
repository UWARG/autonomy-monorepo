"""Gate 3a (offline half) -- the end-to-end sign table.

Asserts that a camera-frame target in each direction produces the correct MAVROS
FLU TwistStamped signs through the full pipeline (compute_setpoint -> FLU mapping).
The LIVE half (props-off in SITL) re-checks these against a real published
TwistStamped before any arm -- see airside/docs/sitl_gates.md. A single wrong sign
here would fly the drone the wrong way, possibly into the person.
"""

from follow_controller import FollowConfig, compute_setpoint
from mavros_setpoint import body_velocity_to_flu

CFG = FollowConfig()


def _flu(x_mm, y_mm, z_mm):
    sp = compute_setpoint(x_mm, y_mm, z_mm, CFG)
    return sp, body_velocity_to_flu(sp.v_forward, sp.v_right, sp.v_down, sp.yaw_rate)


def test_target_ahead_commands_forward():
    sp, flu = _flu(0, 0, 6000)  # far ahead
    assert sp.v_forward > 0 and flu.linear_x > 0


def test_target_too_close_commands_backward():
    sp, flu = _flu(0, 0, 2000)  # inside the 2.5 m standoff
    assert sp.v_forward < 0 and flu.linear_x < 0


def test_target_right_yaws_clockwise_flu_negative():
    sp, flu = _flu(1000, 0, 3000)  # to the right
    assert sp.yaw_rate > 0  # NED clockwise
    assert flu.angular_z < 0  # -> ENU counter-clockwise (the sign flip)


def test_target_left_yaws_counterclockwise_flu_positive():
    sp, flu = _flu(-1000, 0, 3000)
    assert sp.yaw_rate < 0
    assert flu.angular_z > 0


def test_target_below_descends_flu_negative_z():
    sp, flu = _flu(0, 500, 3000)  # below centre (cam y +)
    assert sp.v_down > 0  # descend
    assert flu.linear_z < 0  # FRD down -> FLU up sign flip
