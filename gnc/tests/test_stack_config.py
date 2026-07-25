"""Gate 0 -- cross-consistency invariants of the deployed stack configuration.

These tests exist to make config de-sync a test failure instead of a flight
incident: every quantity that appears in more than one sub-config must agree,
and the timing windows must be mutually consistent.
"""

from flight_modes import should_request
from stack_config import DEPLOYED, StackConfig


def test_command_staleness_exceeds_tree_period():
    """The gap-9 invariant: a fresh command must not go stale between tree ticks."""
    assert DEPLOYED.command_stale_s > 1.0 / DEPLOYED.tree_hz


def test_hard_min_identical_everywhere():
    assert DEPLOYED.follow.hard_min_m == DEPLOYED.safety.hard_min_m
    assert DEPLOYED.follow.hard_min_m == DEPLOYED.reflex.hard_min_m


def test_controller_brakes_more_conservatively_than_the_monitors_predict():
    """The controller PLANS on a lower decel than the monitors PREDICT with:
    the gap between the two v^2/2a curves absorbs the FC's velocity-tracking
    lag. With identical values the demand curve sits a razor-thin fixed gap
    under the trip line at every a (spurious terminal BRAKEs on nominal
    arrivals, observed in SITL). The monitors agree with each other and stay
    at honest threat physics."""
    assert DEPLOYED.follow.a_brake <= DEPLOYED.safety.a_brake
    assert DEPLOYED.safety.a_brake == DEPLOYED.reflex.a_brake


def test_hold_hysteresis_ordered():
    assert DEPLOYED.hold.exit_speed_mps > DEPLOYED.hold.enter_speed_mps
    assert DEPLOYED.hold.exit_yaw_rate > DEPLOYED.hold.enter_yaw_rate


def test_standoff_geometry_ordered():
    f = DEPLOYED.follow
    assert f.standoff_m > f.hard_min_m + f.margin_m
    assert f.standoff_m + f.margin_m < 3.0


def test_stream_faster_than_tree():
    assert DEPLOYED.stream_hz > DEPLOYED.tree_hz


def test_target_freshness_covers_stream_period():
    """A 20 Hz target feed must still look fresh to the 20 Hz reflex."""
    assert DEPLOYED.target_freshness_s > 1.0 / DEPLOYED.stream_hz


def test_deployed_tune_is_the_flying_tune():
    """Pin the deployed FollowConfig so a silent re-tune fails a test, not a flight."""
    assert DEPLOYED.follow.v_max == 1.5
    assert DEPLOYED.follow.v_vertical_max == 1.0
    assert DEPLOYED.follow.kp_vertical == 0.7
    assert DEPLOYED.follow.a_brake == 0.5  # conservative until Gate 6 measures it
    assert DEPLOYED.follow.margin_m == 0.4


def test_defaults_are_consistent_too():
    cfg = StackConfig()
    assert cfg.command_stale_s > 1.0 / cfg.tree_hz


def test_should_request_throttles():
    # already in the wanted mode -> never re-request
    assert not should_request("BRAKE", "BRAKE", "", 0.0, 10.0)
    # first request goes out
    assert should_request("GUIDED", "BRAKE", "", 0.0, 10.0)
    # same request inside the throttle window -> suppressed
    assert not should_request("GUIDED", "BRAKE", "BRAKE", 10.0, 10.2)
    # after the window -> retry allowed
    assert should_request("GUIDED", "BRAKE", "BRAKE", 10.0, 10.6)
    # a different wanted mode is never throttled by the previous request
    assert should_request("GUIDED", "LOITER", "BRAKE", 10.0, 10.2)


def test_cmd_slew_is_a_gentle_ramp():
    """The slew shapes ACCELERATION only (slow-downs are instant -- a lagged
    deceleration penetrates the ring). The ramp just has to stay well inside
    the plant's real acceleration authority so the FC tracks it without the
    overshoot the slew exists to remove."""
    assert 0.0 < DEPLOYED.cmd_slew_mps2 <= 2.0
