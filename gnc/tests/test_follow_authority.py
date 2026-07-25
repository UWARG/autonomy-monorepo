from follow_authority import (
    AuthorityAction,
    AuthorityConfig,
    AuthorityInputs,
    AuthorityState,
    FollowAuthority,
    StopReason,
)


def inputs(now=10.0, **overrides):
    values = dict(
        now_s=now,
        fc_state_rx_s=now,
        connected=True,
        mode="GUIDED",
        armed=True,
        airborne=True,
        target_valid=True,
        proximity_emergency=False,
        rc_kill=False,
    )
    values.update(overrides)
    return AuthorityInputs(**values)


def test_disabled_by_default_and_enable_requires_all_preconditions():
    authority = FollowAuthority()
    assert authority.step(inputs()).action is AuthorityAction.RELEASE
    assert not authority.request_enable(inputs(mode="LOITER"))
    assert not authority.request_enable(inputs(armed=False))
    assert not authority.request_enable(inputs(target_valid=False))
    assert authority.request_enable(inputs())
    assert authority.step(inputs()).action is AuthorityAction.STREAM


def test_props_off_hitl_bypasses_only_armed_airborne():
    authority = FollowAuthority(AuthorityConfig(props_off_bypass_airborne=True))
    assert authority.request_enable(inputs(armed=False, airborne=False))
    authority.disable()
    assert not authority.request_enable(
        inputs(mode="LOITER", armed=False, airborne=False)
    )


def test_rc_enable_needs_observed_low_then_rising_edge():
    authority = FollowAuthority()
    assert not authority.update_rc_enable(True, inputs())  # booted high is not an edge
    assert not authority.update_rc_enable(False, inputs())
    assert authority.update_rc_enable(True, inputs())
    authority.disable()
    assert not authority.update_rc_enable(True, inputs())  # held high cannot resume


def test_enable_event_cannot_reacquire_while_follow_owns_target():
    authority = FollowAuthority()
    assert authority.request_enable(inputs())
    assert authority.step(inputs()).state is AuthorityState.ACTIVE
    assert not authority.request_enable(inputs())
    assert authority.state is AuthorityState.ACTIVE


def test_rc_kill_zeroes_once_then_latches_release():
    authority = FollowAuthority()
    assert authority.request_enable(inputs())
    killed = authority.step(inputs(rc_kill=True))
    assert killed.action is AuthorityAction.ZERO
    assert killed.clear_target_lock
    assert killed.stop_reason is StopReason.RC_KILL
    assert authority.step(inputs()).action is AuthorityAction.RELEASE


def test_mode_exit_releases_and_guided_return_does_not_resume():
    authority = FollowAuthority()
    assert authority.request_enable(inputs())
    result = authority.step(inputs(mode="LAND"))
    assert result.action is AuthorityAction.RELEASE
    assert result.clear_target_lock
    assert result.stop_reason is StopReason.MODE_EXIT
    assert authority.step(inputs(mode="GUIDED")).action is AuthorityAction.RELEASE


def test_stale_fc_state_releases_immediately():
    authority = FollowAuthority()
    assert authority.request_enable(inputs())
    result = authority.step(inputs(now=11.0, fc_state_rx_s=10.0))
    assert result.action is AuthorityAction.RELEASE
    assert result.stop_reason is StopReason.FC_STATE_STALE


def test_disarm_or_landing_releases_in_production():
    authority = FollowAuthority()
    assert authority.request_enable(inputs())
    result = authority.step(inputs(armed=False))
    assert result.action is AuthorityAction.RELEASE
    assert result.stop_reason is StopReason.FLIGHT_STATE_EXIT


def test_brief_same_target_loss_holds_then_recovers():
    authority = FollowAuthority(AuthorityConfig(lost_target_timeout_s=1.0))
    assert authority.request_enable(inputs())
    assert authority.step(inputs()).state is AuthorityState.ACTIVE
    first = authority.step(inputs(now=10.1, target_valid=False))
    assert first.state is AuthorityState.BRIEF_LOSS
    assert first.action is AuthorityAction.ZERO
    recovered = authority.step(inputs(now=10.5, target_valid=True))
    assert recovered.state is AuthorityState.ACTIVE
    assert recovered.action is AuthorityAction.STREAM


def test_confirmed_loss_requests_loiter_and_clears_lock():
    authority = FollowAuthority(AuthorityConfig(lost_target_timeout_s=1.0))
    assert authority.request_enable(inputs())
    assert authority.step(inputs()).state is AuthorityState.ACTIVE
    authority.step(inputs(now=10.1, target_valid=False))
    result = authority.step(inputs(now=11.1, target_valid=False))
    assert result.action is AuthorityAction.LOITER
    assert result.clear_target_lock
    assert result.stop_reason is StopReason.TARGET_LOST


def test_persistent_out_of_validated_range_has_distinct_terminal_reason():
    authority = FollowAuthority(AuthorityConfig(lost_target_timeout_s=1.0))
    assert authority.request_enable(inputs())
    assert authority.step(inputs()).state is AuthorityState.ACTIVE
    hold = authority.step(
        inputs(
            now=10.1,
            target_valid=False,
            target_out_of_range=True,
        )
    )
    assert hold.state is AuthorityState.BRIEF_LOSS
    result = authority.step(
        inputs(
            now=11.1,
            target_valid=False,
            target_out_of_range=True,
        )
    )
    assert result.action is AuthorityAction.LOITER
    assert result.clear_target_lock
    assert result.stop_reason is StopReason.OUT_OF_VALIDATED_RANGE


def test_in_range_reacquisition_clears_out_of_range_loss_reason():
    authority = FollowAuthority(AuthorityConfig(lost_target_timeout_s=1.0))
    assert authority.request_enable(inputs())
    authority.step(inputs())
    authority.step(inputs(now=10.1, target_valid=False, target_out_of_range=True))
    recovered = authority.step(inputs(now=10.5, target_valid=True))
    assert recovered.state is AuthorityState.ACTIVE
    authority.step(inputs(now=10.6, target_valid=False))
    result = authority.step(inputs(now=11.7, target_valid=False))
    assert result.stop_reason is StopReason.TARGET_LOST


def test_proximity_emergency_requests_brake_terminally():
    authority = FollowAuthority()
    assert authority.request_enable(inputs())
    result = authority.step(inputs(proximity_emergency=True))
    assert result.action is AuthorityAction.BRAKE
    assert result.clear_target_lock
    assert result.stop_reason is StopReason.PROXIMITY
    assert not authority.request_enable(inputs())
    assert authority.step(inputs()).action is AuthorityAction.BRAKE


def test_reset_disables_and_clears_target():
    authority = FollowAuthority()
    assert authority.request_enable(inputs())
    authority.reset_target()
    assert not authority.enabled
    assert authority.stop_reason is StopReason.RESET_TARGET
