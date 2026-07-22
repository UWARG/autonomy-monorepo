"""Gate 0 -- the streamer's flight-action ladder (pure decision)."""

from flight_modes import ControlAction, FlightInputs, decide


def _inputs(**kw):
    base = dict(
        in_guided=True,
        ekf_ok=True,
        command_fresh=True,
        reflex_danger=False,
        estop_emergency=False,
        estop_recede=False,
        estop_hold=False,
    )
    base.update(kw)
    return FlightInputs(**base)


def test_nominal_streams_velocity():
    assert decide(_inputs()) is ControlAction.STREAM_VELOCITY


def test_not_guided_releases_to_pilot():
    assert decide(_inputs(in_guided=False)) is ControlAction.RELEASE
    # pilot-takeover beats everything, even a danger
    assert decide(_inputs(in_guided=False, reflex_danger=True)) is ControlAction.RELEASE


def test_reflex_danger_brakes_with_ekf():
    assert decide(_inputs(reflex_danger=True)) is ControlAction.SET_BRAKE


def test_reflex_danger_falls_back_to_zero_without_ekf():
    assert decide(_inputs(reflex_danger=True, ekf_ok=False)) is ControlAction.STREAM_ZERO


def test_proximity_emergency_brakes():
    assert decide(_inputs(estop_emergency=True, estop_recede=True)) is ControlAction.SET_BRAKE


def test_lost_target_loiters_with_ekf():
    assert decide(_inputs(estop_emergency=True, estop_recede=False)) is ControlAction.SET_LOITER


def test_lost_target_falls_back_to_zero_without_ekf():
    v = decide(_inputs(estop_emergency=True, estop_recede=False, ekf_ok=False))
    assert v is ControlAction.STREAM_ZERO


def test_stale_command_holds():
    assert decide(_inputs(command_fresh=False)) is ControlAction.STREAM_ZERO


def test_brief_dropout_hold():
    assert decide(_inputs(estop_hold=True)) is ControlAction.STREAM_ZERO


def test_stale_command_keeps_position_hold():
    """The re-latch-ratchet fix: a latched hold survives a stale command."""
    v = decide(_inputs(command_fresh=False, holding=True))
    assert v is ControlAction.HOLD_POSITION


def test_brief_dropout_keeps_position_hold():
    assert decide(_inputs(estop_hold=True, holding=True)) is ControlAction.HOLD_POSITION


def test_hold_never_survives_ekf_loss():
    v = decide(_inputs(command_fresh=False, holding=True, ekf_ok=False))
    assert v is ControlAction.STREAM_ZERO


def test_hold_never_survives_pilot_takeover():
    assert decide(_inputs(in_guided=False, holding=True)) is ControlAction.RELEASE


def test_hold_never_survives_emergency():
    v = decide(_inputs(estop_emergency=True, estop_recede=True, holding=True))
    assert v is ControlAction.SET_BRAKE
    v = decide(_inputs(estop_emergency=True, estop_recede=False, holding=True))
    assert v is ControlAction.SET_LOITER


def test_nominal_with_hold_still_streams_velocity():
    """While the command is fresh the streamer's HoldPolicy owns the overlay."""
    assert decide(_inputs(holding=True)) is ControlAction.STREAM_VELOCITY


def test_disarmed_emergencies_never_request_modes():
    """On the ground (disarmed, e.g. the props-off sign check) an emergency caps
    at zero velocity -- requesting BRAKE/LOITER while parked kicks the FC out of
    GUIDED and breaks bench testing for no safety benefit."""
    v = decide(_inputs(reflex_danger=True, armed=False))
    assert v is ControlAction.STREAM_ZERO
    v = decide(_inputs(estop_emergency=True, estop_recede=True, armed=False))
    assert v is ControlAction.STREAM_ZERO
    v = decide(_inputs(estop_emergency=True, estop_recede=False, armed=False))
    assert v is ControlAction.STREAM_ZERO


def test_armed_emergencies_still_latch_modes():
    assert decide(_inputs(reflex_danger=True, armed=True)) is ControlAction.SET_BRAKE
    v = decide(_inputs(estop_emergency=True, estop_recede=False, armed=True))
    assert v is ControlAction.SET_LOITER


def test_danger_precedence_over_lost():
    # both flags set -> proximity danger (BRAKE) wins over lost (LOITER)
    v = decide(_inputs(estop_emergency=True, estop_recede=True, reflex_danger=True))
    assert v is ControlAction.SET_BRAKE
