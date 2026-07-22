"""Gate 0 -- the steady-state position-hold policy (pure state machine)."""

from hypothesis import given, settings
from hypothesis import strategies as st

from hold_policy import HoldAction, HoldConfig, HoldPolicy

CFG = HoldConfig(
    enter_speed_mps=0.08,
    exit_speed_mps=0.25,
    enter_yaw_rate=0.10,
    exit_yaw_rate=0.40,
    min_hold_s=2.0,
    relatch_cooldown_s=1.0,
    pose_max_age_s=0.5,
)

SETTLED = dict(speed_mps=0.0, yaw_rate=0.0, pose_age_s=0.05, ekf_ok=True)
MOVING = dict(speed_mps=0.6, yaw_rate=0.0, pose_age_s=0.05, ekf_ok=True)


def test_latches_when_settled_with_fresh_pose_and_ekf():
    policy = HoldPolicy(CFG)
    status = policy.step(now_s=0.0, **SETTLED)
    assert status.action is HoldAction.LATCH
    assert policy.is_holding
    assert status.relatch_count == 1


def test_follows_while_moving():
    policy = HoldPolicy(CFG)
    status = policy.step(now_s=0.0, **MOVING)
    assert status.action is HoldAction.FOLLOW
    assert not policy.is_holding


def test_never_latches_on_stale_pose():
    policy = HoldPolicy(CFG)
    status = policy.step(speed_mps=0.0, yaw_rate=0.0, pose_age_s=0.9, ekf_ok=True, now_s=0.0)
    assert status.action is HoldAction.FOLLOW
    assert "stale" in status.reason


def test_never_latches_without_ekf():
    policy = HoldPolicy(CFG)
    status = policy.step(speed_mps=0.0, yaw_rate=0.0, pose_age_s=0.05, ekf_ok=False, now_s=0.0)
    assert status.action is HoldAction.FOLLOW
    assert not policy.is_holding


def test_holds_then_releases_when_target_moves_after_dwell():
    policy = HoldPolicy(CFG)
    assert policy.step(now_s=0.0, **SETTLED).action is HoldAction.LATCH
    assert policy.step(now_s=0.5, **SETTLED).action is HoldAction.HOLD
    # target moves after the dwell -> release to follow
    status = policy.step(now_s=3.0, **MOVING)
    assert status.action is HoldAction.FOLLOW
    assert "moved" in status.reason
    assert not policy.is_holding


def test_dwell_ignores_exit_jitter():
    policy = HoldPolicy(CFG)
    policy.step(now_s=0.0, **SETTLED)
    # inside min_hold_s, even a clear "moved" signal does not release
    status = policy.step(now_s=1.0, **MOVING)
    assert status.action is HoldAction.HOLD
    assert "dwell" in status.reason


def test_small_jitter_never_releases():
    policy = HoldPolicy(CFG)
    policy.step(now_s=0.0, **SETTLED)
    status = policy.step(
        speed_mps=0.15, yaw_rate=0.2, pose_age_s=0.05, ekf_ok=True, now_s=5.0
    )  # between enter and exit thresholds -> hysteresis holds
    assert status.action is HoldAction.HOLD


def test_ekf_degradation_releases_immediately_even_in_dwell():
    policy = HoldPolicy(CFG)
    policy.step(now_s=0.0, **SETTLED)
    status = policy.step(speed_mps=0.0, yaw_rate=0.0, pose_age_s=0.05, ekf_ok=False, now_s=0.5)
    assert status.action is HoldAction.FOLLOW
    assert "ekf" in status.reason
    assert not policy.is_holding


def test_relatch_cooldown_blocks_immediate_ratchet():
    policy = HoldPolicy(CFG)
    policy.step(now_s=0.0, **SETTLED)  # latch #1
    policy.step(now_s=3.0, **MOVING)  # release
    status = policy.step(now_s=3.5, **SETTLED)  # settled again, inside cooldown
    assert status.action is HoldAction.FOLLOW
    assert "cooldown" in status.reason
    status = policy.step(now_s=4.5, **SETTLED)  # cooldown over
    assert status.action is HoldAction.LATCH
    assert status.relatch_count == 2


def test_reset_clears_everything():
    policy = HoldPolicy(CFG)
    policy.step(now_s=0.0, **SETTLED)
    policy.reset()
    assert not policy.is_holding
    assert policy.relatch_count == 0
    assert policy.step(now_s=10.0, **SETTLED).action is HoldAction.LATCH


@settings(max_examples=300)
@given(
    speed=st.floats(min_value=0.0, max_value=3.0, allow_nan=False),
    yaw=st.floats(min_value=-2.0, max_value=2.0, allow_nan=False),
    pose_age=st.floats(min_value=0.0, max_value=3.0, allow_nan=False),
    ekf_ok=st.booleans(),
)
def test_property_never_latch_blind_or_stale(speed, yaw, pose_age, ekf_ok):
    """From idle, a latch requires fresh pose AND healthy EKF, no exceptions."""
    policy = HoldPolicy(CFG)
    status = policy.step(speed_mps=speed, yaw_rate=yaw, pose_age_s=pose_age, ekf_ok=ekf_ok, now_s=0.0)
    if status.action is HoldAction.LATCH:
        assert ekf_ok
        assert pose_age < CFG.pose_max_age_s
        assert speed < CFG.enter_speed_mps and abs(yaw) < CFG.enter_yaw_rate


@settings(max_examples=100)
@given(st.lists(st.floats(min_value=0.0, max_value=0.5, allow_nan=False), min_size=20, max_size=200))
def test_property_relatch_count_bounded_under_jitter(speeds):
    """Random near-threshold jitter cannot ratchet: cooldown + dwell bound latches.

    Over a T-second episode there can be at most ~T / (min_hold_s +
    relatch_cooldown_s) latch events, whatever the jitter does.
    """
    policy = HoldPolicy(CFG)
    dt = 0.05
    for i, speed in enumerate(speeds):
        policy.step(speed_mps=speed, yaw_rate=0.0, pose_age_s=0.01, ekf_ok=True, now_s=i * dt)
    duration = len(speeds) * dt
    max_latches = int(duration / (CFG.min_hold_s + CFG.relatch_cooldown_s)) + 1
    assert policy.relatch_count <= max_latches
