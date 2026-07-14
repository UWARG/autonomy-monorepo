"""Gate 0 -- the autonomous safety monitor triggers."""

from safety import EStopAction, SafetyConfig, SafetyMonitor

CFG = SafetyConfig(hard_min_m=1.5, lost_timeout_s=1.0, a_brake=1.0, reaction_time_s=0.3)
DT = 0.05


def _settle(mon, range_m, ticks=1):
    v = None
    for _ in range(ticks):
        v = mon.evaluate(range_m, DT)
    return v


def test_normal_follow_is_not_emergency():
    mon = SafetyMonitor(CFG)
    v = _settle(mon, 3.0, ticks=3)
    assert v.action is EStopAction.NONE
    assert not v.is_emergency


def test_hard_min_breach_trips_zero_velocity_and_recede():
    mon = SafetyMonitor(CFG)
    _settle(mon, 3.0)
    v = mon.evaluate(1.2, DT)  # inside the 1.5 m ring
    assert v.is_emergency
    assert v.action is EStopAction.ZERO_VELOCITY
    assert v.recede


def test_target_lost_after_timeout():
    mon = SafetyMonitor(CFG)
    _settle(mon, 3.0)
    # 1.0 s timeout / 0.05 s ticks -> 20 ticks; brief dropout before that holds.
    brief = mon.evaluate(None, DT)
    assert not brief.is_emergency
    assert brief.action is EStopAction.ZERO_VELOCITY
    last = None
    for _ in range(25):
        last = mon.evaluate(None, DT)
    assert last.is_emergency
    assert "lost" in last.reason


def test_closing_too_fast_trips():
    mon = SafetyMonitor(CFG)
    # Approach from 5 m at ~4 m/s: stopping dist = 16/2 = 8 m >> remaining range.
    mon.evaluate(5.0, DT)
    v = mon.evaluate(5.0 - 4.0 * DT, DT)  # closed 0.2 m in one 0.05 s tick -> 4 m/s
    assert v.is_emergency
    assert v.recede
    assert "closing" in v.reason


def test_slow_approach_does_not_trip():
    mon = SafetyMonitor(CFG)
    mon.evaluate(5.0, DT)
    v = mon.evaluate(5.0 - 0.5 * DT, DT)  # 0.5 m/s, easily stoppable
    assert not v.is_emergency


def test_latch_is_sticky_until_stable_reacquire():
    mon = SafetyMonitor(CFG)
    _settle(mon, 3.0)
    mon.evaluate(1.2, DT)  # trip hard-min
    # Target reappears just outside hard_min but inside the clear margin -> stays latched.
    v = mon.evaluate(1.6, DT)
    assert v.is_emergency
    # Stably outside hard_min + clear_margin for clear_ticks -> clears.
    last = None
    for _ in range(CFG.clear_ticks + 1):
        last = mon.evaluate(2.5, DT)
    assert not last.is_emergency
    assert last.action is EStopAction.NONE
