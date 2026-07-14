"""Gate 1 -- property-based + fuzz tests.

Asserts safety invariants over thousands of random inputs (the kind of thing that
catches a sign slip or a phantom-setpoint NaN that example tests miss).
"""

import math

from hypothesis import given, settings
from hypothesis import strategies as st

from follow_controller import FollowConfig, compute_setpoint
from safety import EStopAction, SafetyConfig, SafetyMonitor

CFG = FollowConfig()

# Camera readings in mm. Includes the raw_z==0 sentinel and negatives to fuzz.
mm = st.floats(min_value=-4000, max_value=4000, allow_nan=False, allow_infinity=False)
pitch = st.floats(min_value=-0.8, max_value=0.8, allow_nan=False, allow_infinity=False)


def _finite(*values) -> bool:
    return all(math.isfinite(v) for v in values)


@settings(max_examples=600)
@given(x=mm, y=mm, z=mm, p=pitch)
def test_setpoint_always_finite_and_within_clamps(x, y, z, p):
    cfg = FollowConfig(mount_pitch_rad=p)
    sp = compute_setpoint(x, y, z, cfg)
    assert _finite(sp.v_forward, sp.v_right, sp.v_down, sp.yaw_rate)
    assert abs(sp.v_forward) <= cfg.v_max + 1e-9
    assert abs(sp.yaw_rate) <= cfg.yaw_rate_max + 1e-9
    assert abs(sp.v_down) <= cfg.v_vertical_max + 1e-9


@settings(max_examples=600)
@given(x=mm, y=mm, z=mm)
def test_approach_speed_never_exceeds_braking_cap(x, y, z):
    sp = compute_setpoint(x, y, z, CFG)
    if sp.v_forward > 0.0:
        assert sp.v_forward <= sp.v_brake_cap + 1e-9


@settings(max_examples=200)
@given(
    x=st.sampled_from([-1e7, -1e3, 0.0, 1e3, 1e7]),
    y=st.sampled_from([-1e7, 0.0, 1e7]),
    z=st.sampled_from([-1e7, 0.0, 1e7]),
)
def test_extreme_inputs_stay_finite_and_bounded(x, y, z):
    sp = compute_setpoint(x, y, z, CFG)
    assert _finite(sp.v_forward, sp.v_right, sp.v_down, sp.yaw_rate)
    assert abs(sp.v_forward) <= CFG.v_max + 1e-9
    assert abs(sp.yaw_rate) <= CFG.yaw_rate_max + 1e-9


@settings(max_examples=300)
@given(
    ranges=st.lists(
        st.one_of(st.none(), st.floats(min_value=0.05, max_value=10.0)),
        min_size=1,
        max_size=60,
    )
)
def test_hard_min_always_trips_emergency(ranges):
    scfg = SafetyConfig()
    mon = SafetyMonitor(scfg)
    for r in ranges:
        v = mon.evaluate(r, 0.05)
        if r is not None and r < scfg.hard_min_m:
            assert v.is_emergency
            assert v.action is not EStopAction.NONE


@settings(max_examples=100)
@given(hold_ticks=st.integers(min_value=25, max_value=80))
def test_target_lost_always_eventually_latches(hold_ticks):
    scfg = SafetyConfig(lost_timeout_s=1.0)
    mon = SafetyMonitor(scfg)
    mon.evaluate(3.0, 0.05)
    last = None
    for _ in range(hold_ticks):
        last = mon.evaluate(None, 0.05)
    assert last.is_emergency
    assert last.action is EStopAction.ZERO_VELOCITY
