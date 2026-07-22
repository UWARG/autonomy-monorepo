"""Gate 0 -- the follow control law (deterministic, example-based)."""

import math

from follow_controller import FollowConfig, braking_cap, compute_setpoint

CFG = FollowConfig()
STANDOFF_MM = int(CFG.standoff_m * 1000)


def _ahead(distance_m, x_mm=0, y_mm=0):
    """Camera reading for a target `distance_m` straight ahead (centred)."""
    return compute_setpoint(x_mm, y_mm, distance_m * 1000, CFG)


def test_far_target_moves_forward_within_caps():
    sp = _ahead(6.0)
    assert sp.v_forward > 0.0
    assert sp.v_forward <= CFG.v_max + 1e-9
    assert sp.v_forward <= sp.v_brake_cap + 1e-9
    assert abs(sp.yaw_rate) < 1e-9  # centred -> no yaw


def test_at_standoff_holds():
    sp = _ahead(CFG.standoff_m)
    assert abs(sp.v_forward) < 1e-9
    assert abs(sp.range_error_m) < 1e-9


def test_inside_standoff_recedes():
    sp = _ahead(2.0)  # standoff is 2.5
    assert sp.v_forward < 0.0  # backs away


def test_approach_speed_never_exceeds_braking_cap():
    # A range where the predictive cap binds below v_max.
    sp = _ahead(CFG.standoff_m + 0.55)
    assert sp.v_brake_cap < CFG.v_max
    assert sp.v_forward > 0.0
    assert abs(sp.v_forward - sp.v_brake_cap) < 1e-9  # clamped to the cap


def test_yaw_centres_target():
    right = compute_setpoint(1000, 0, 3000, CFG)
    left = compute_setpoint(-1000, 0, 3000, CFG)
    assert right.yaw_rate > 0.0  # target to the right -> turn right (+CW)
    assert left.yaw_rate < 0.0
    assert abs(right.yaw_rate) <= CFG.yaw_rate_max + 1e-9


def test_vertical_framing_sign():
    below = compute_setpoint(0, 500, 3000, CFG)  # target below centre (cam y +)
    above = compute_setpoint(0, -500, 3000, CFG)
    assert below.v_down > 0.0  # descend to re-centre
    assert above.v_down < 0.0
    assert abs(below.v_down) <= CFG.v_vertical_max + 1e-9


def test_forward_speed_reduced_when_target_off_axis():
    centred = compute_setpoint(0, 0, 5000, CFG)
    off_axis = compute_setpoint(3000, 0, 5000, CFG)  # ~31 deg to the right
    assert off_axis.v_forward < centred.v_forward  # mostly turn before advancing


def test_braking_cap_monotonic_and_zero_inside_margin():
    assert braking_cap(0.0, 1.0, 0.5) == 0.0
    assert braking_cap(0.5, 1.0, 0.5) == 0.0  # at the margin
    assert braking_cap(3.0, 1.0, 0.5) > braking_cap(1.0, 1.0, 0.5) > 0.0
    # exact value: sqrt(2*a*(err-margin))
    assert abs(braking_cap(2.5, 1.0, 0.5) - math.sqrt(2 * 1.0 * 2.0)) < 1e-12
