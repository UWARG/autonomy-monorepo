"""Gate 2 -- closed-loop simulation assertions (the proof-of-concept).

These run headless (no matplotlib) and prove, with a multi-rate momentum model
running the REAL ladder/hold/reflex stack, that the deployed logic produces safe
motion -- and that the harness REVEALS the known failure modes (sign error, the
stale-window re-latch ratchet) rather than hiding them.

Honest-guarantee note: when the PERSON charges the drone, the deployed ladder
BRAKEs (position hold); it does not flee. A person can therefore still close the
gap -- the verified guarantee is that the drone never commands motion toward a
target inside the ring, and that the emergency latches BEFORE the ring is
breached. The lunge tests below assert exactly that, not "min range >= hard-min",
which no stop-in-place system can promise against a determined walker.
"""

import pytest

from follow_controller import FollowConfig
from sim.follow_sim import DRONE_Z, SCENARIOS, SimConfig, run_sim, sim_config_for
from stack_config import StackConfig

FC = FollowConfig()
HARD_MIN = FC.hard_min_m


# --- drone-caused safety: scenarios where the person never charges ------------

@pytest.mark.parametrize("scenario", ["walk_straight", "weave", "vary_height", "gusty_follow"])
def test_safe_scenarios_never_breach_hard_min(scenario):
    result = run_sim(SCENARIOS[scenario], FC, sim_config_for(scenario))
    assert result.min_true_range >= HARD_MIN, (
        f"{scenario} breached hard-min: {result.min_true_range:.2f} < {HARD_MIN}"
    )


def test_walk_straight_settles_near_standoff():
    result = run_sim(SCENARIOS["walk_straight"], FC, sim_config_for("walk_straight"))
    # over the second half of the run, mean range should sit within ~1 m of standoff
    tail = result.true_range[len(result.true_range) // 2:]
    mean_range = sum(tail) / len(tail)
    assert abs(mean_range - FC.standoff_m) < 1.0


def test_nominal_scenarios_never_brake_or_loiter():
    for scenario in ("walk_straight", "weave", "vary_height"):
        result = run_sim(SCENARIOS[scenario], FC, sim_config_for(scenario))
        actions = {a for _, a in result.stream_actions}
        assert "set_brake" not in actions and "set_loiter" not in actions, scenario


# --- person-caused proximity: the honest guarantee ----------------------------

@pytest.mark.parametrize("scenario", ["lunge", "fast_lunge"])
def test_lunge_latches_emergency_before_the_ring_and_never_advances(scenario):
    result = run_sim(SCENARIOS[scenario], FC, sim_config_for(scenario))
    assert any(result.emergency), f"{scenario}: emergency never engaged"

    # 1) the emergency must latch BEFORE the ring is breached (predictive, not reactive)
    first_emergency = result.emergency.index(True)
    breaches = [i for i, r in enumerate(result.true_range) if r < HARD_MIN]
    if breaches:
        assert first_emergency < breaches[0], (
            f"{scenario}: emergency at tick {first_emergency} but breach at {breaches[0]}"
        )

    # 2) once the emergency is latched the drone never commands motion toward the person
    after = range(first_emergency, len(result.t))
    assert all(result.v_forward_cmd[i] <= 1e-9 for i in after), (
        f"{scenario}: commanded forward motion during an emergency"
    )

    # 3) the ladder escalated to the hard BRAKE latch (EKF is healthy in this sim)
    assert "set_brake" in {a for _, a in result.stream_actions}


def test_fast_lunge_breach_is_person_caused_not_drone_caused():
    """After BRAKE the drone is position-held: any further closure is the walker."""
    result = run_sim(SCENARIOS["fast_lunge"], FC, sim_config_for("fast_lunge"))
    first_emergency = result.emergency.index(True)
    dx0 = result.drone_x[first_emergency]
    # drone displacement toward the person (+x) after the latch stays ~zero
    assert max(result.drone_x[first_emergency:]) - dx0 < 0.25


# --- target loss ---------------------------------------------------------------

def test_target_lost_latches_emergency_and_loiters():
    result = run_sim(SCENARIOS["disappear"], FC, sim_config_for("disappear"))
    assert result.emergency[-1] is True
    assert "set_loiter" in {a for _, a in result.stream_actions}


def test_dropout_storm_never_false_latches_lost():
    """25% random detection dropout is far from a 1 s continuous gap: no lost latch,
    no emergency, and the drone still behaves (hold engages on the standing person)."""
    result = run_sim(SCENARIOS["dropout_storm"], FC, sim_config_for("dropout_storm"))
    assert not any(result.emergency)
    assert result.min_true_range >= HARD_MIN


# --- noise / estimation --------------------------------------------------------

def test_noisy_detection_with_ema_stays_safe():
    result = run_sim(
        SCENARIOS["walk_straight"],
        FC,
        sim_config_for("walk_straight", noise_mm=40.0, ema_alpha=0.3, seed=1),
    )
    assert result.min_true_range >= HARD_MIN


def test_injected_sign_error_is_revealed_by_the_harness():
    # Correct signs: converges to the standoff. Inverted forward axis: the drone
    # runs away from every command and the range diverges -- the sim catches the
    # frame bug before any flight.
    safe = run_sim(SCENARIOS["walk_straight"], FC, sim_config_for("walk_straight"))
    bad = run_sim(
        SCENARIOS["walk_straight"], FC, sim_config_for("walk_straight", sign_error=True)
    )
    assert abs(safe.final_range - FC.standoff_m) < 1.0
    assert bad.final_range > 2.0 * FC.standoff_m


# --- steady-state position hold ------------------------------------------------

def test_stand_still_hold_engages_and_sticks():
    result = run_sim(SCENARIOS["stand_still"], FC, sim_config_for("stand_still"))
    assert 1 <= result.relatch_count <= 2, f"latches={result.relatch_count}"
    assert result.hold_point_drift < 0.05  # the latched point itself never ratchets
    assert result.hold_active[-1], "hold not engaged at the end of a 60 s stand-still"
    # held position rejects the wind/sag disturbance the velocity mode would drift on
    t_hold = result.hold_points[0][0]
    idx = next(i for i, t in enumerate(result.t) if t >= t_hold)
    xs, ys, zs = result.drone_x[idx:], result.drone_y[idx:], result.drone_z[idx:]
    assert max(xs) - min(xs) < 0.4 and max(ys) - min(ys) < 0.4 and max(zs) - min(zs) < 0.4


def test_hold_releases_when_the_person_walks_away():
    result = run_sim(SCENARIOS["hold_then_move"], FC, sim_config_for("hold_then_move"))
    assert result.relatch_count >= 1
    assert not result.hold_active[-1], "hold never released after the person moved"
    assert not any(result.emergency)
    # follow re-engaged: by the end the drone is closing back toward the standoff
    assert result.final_range < 5.0


def test_hold_disabled_leaves_a_standing_altitude_sag():
    """Without the hold the sag bias parks the drone below the person (the
    steady-state correction never zeroes the offset) -- the hold's whole point."""
    no_hold = StackConfig()
    no_hold = StackConfig(hold=type(no_hold.hold)(enter_speed_mps=-1.0))  # never latch
    result = run_sim(
        SCENARIOS["stand_still"], sim_cfg=sim_config_for("stand_still"), stack=no_hold
    )
    assert result.relatch_count == 0
    tail = result.drone_z[len(result.drone_z) // 2:]
    assert sum(tail) / len(tail) < DRONE_Z - 0.02  # sits measurably low forever


# --- the gap-9 tripwire ---------------------------------------------------------

def test_stale_window_shorter_than_command_period_ratchets_the_hold():
    """REGRESSION TRIPWIRE for the SITL position-hold failure: with the legacy
    streamer semantics (any non-follow action drops the latch) and a staleness
    window (0.25 s) shorter than the 2 Hz command period, the ladder toggles
    velocity/zero every period and the hold re-latches at an ever-sagging pose.
    The deployed config (0.75 s window + HOLD_POSITION rung) must be clean."""
    calm_sag = SimConfig(
        duration_s=60.0, wind=(0.0, 0.0, -0.02), gust_sigma=0.0, legacy_stale_semantics=True
    )
    legacy = run_sim(
        SCENARIOS["stand_still"], sim_cfg=calm_sag, stack=StackConfig(command_stale_s=0.25)
    )
    assert legacy.action_toggles() > 50, "legacy flapping not reproduced"
    assert legacy.relatch_count > 20, "legacy re-latch ratchet not reproduced"
    hold_zs = [p[3] for p in legacy.hold_points]
    assert hold_zs[0] - hold_zs[-1] > 0.05, "hold altitude did not ratchet downward"

    fixed = run_sim(
        SCENARIOS["stand_still"],
        sim_cfg=SimConfig(duration_s=60.0, wind=(0.0, 0.0, -0.02), gust_sigma=0.0),
        stack=StackConfig(),  # deployed staleness semantics + window
    )
    assert fixed.action_toggles() <= 2
    assert fixed.relatch_count <= 2
    assert fixed.hold_point_drift < 0.05


# --- the descent study (D1 evidence) --------------------------------------------

def test_pitch_coupling_descent_is_bounded_and_transient():
    """Quasi-static braking pitch-back corrupts the camera vertical channel, but
    the resulting excursion is centimetres and self-recovering -- evidence that
    the SITL 'descended and crashed' event was NOT explained by pitch-back alone
    (see the --sweep CLI for the full grid)."""
    for v_max in (0.8, 1.5):
        fc = FollowConfig(v_max=v_max)
        result = run_sim(
            SCENARIOS["approach_brake"],
            fc,
            sim_config_for("approach_brake", pitch_coupling=True),
        )
        dip = DRONE_Z - result.min_altitude
        assert dip < 0.15, f"v_max={v_max}: unexpected dip {dip:.3f} m"
        assert result.min_true_range >= HARD_MIN
        # and the framing recovers: final altitude back near the person's height
        assert abs(result.drone_z[-1] - DRONE_Z) < 0.3


def test_streamed_command_never_steps_faster_than_the_slew():
    """Engage-transient regression: the streamed follow command must never step
    UP faster than cmd_slew_mps2 (the step -> FC overshoot -> spurious
    closing-rate BRAKE chain observed in SITL at v_max=1.5). Slow-downs are
    deliberately instant -- a lagged deceleration penetrates the ring."""
    stack = StackConfig()
    result = run_sim(
        SCENARIOS["approach_brake"], sim_cfg=sim_config_for("approach_brake"), stack=stack
    )
    dv_tick = stack.cmd_slew_mps2 / stack.stream_hz
    worst_up = 0.0
    for (a, b, ha, hb) in zip(
        result.v_forward_cmd,
        result.v_forward_cmd[1:],
        result.hold_active,
        result.hold_active[1:],
    ):
        # only velocity-mode samples; at a hold latch the RECORDED diagnostic
        # switches to the hold's zero (the wire carries a position setpoint)
        if ha or hb:
            continue
        if abs(b) > abs(a) and (a == 0.0 or (a >= 0) == (b >= 0)):
            worst_up = max(worst_up, abs(b) - abs(a))
    assert worst_up <= dv_tick + 1e-6, f"command stepped up {worst_up:.3f} m/s in one tick"
    assert max(result.v_forward_cmd) == pytest.approx(stack.follow.v_max, abs=0.05)


def test_full_speed_arrival_never_trips_the_reflex_brake():
    """Arrival-deceleration regression: settling on the ring from a v_max=1.5
    approach must NOT read as a lunge (the LS rate estimator's midpoint lag
    overstates the closing rate during hard self-braking -- observed as a
    spurious terminal BRAKE right after the hold latched in SITL)."""
    result = run_sim(
        SCENARIOS["approach_brake"],
        sim_cfg=sim_config_for("approach_brake", noise_mm=40.0, seed=5),
        stack=StackConfig(),
    )
    actions = {a for _, a in result.stream_actions}
    assert "set_brake" not in actions and "set_loiter" not in actions
    assert result.relatch_count >= 1  # the hold still engages
