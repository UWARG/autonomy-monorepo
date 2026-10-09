import math

import pytest
from navigation.setpoint_router import SetpointMode, SetpointRouter


def test_position_to_velocity_switch_zeroes_before_velocity() -> None:
    router = SetpointRouter(velocity_lease_s=0.3)

    position = router.receive_position(command_allowed=True)
    velocity = router.receive_velocity(
        now_s=10.0,
        command_allowed=True,
        command_valid=True,
    )

    assert position.publish_position
    assert not position.publish_zero_velocity
    assert velocity.publish_zero_velocity
    assert velocity.publish_velocity
    assert router.mode == SetpointMode.VELOCITY


def test_velocity_to_position_switch_zeroes_before_position() -> None:
    router = SetpointRouter(velocity_lease_s=0.3)
    router.receive_velocity(
        now_s=10.0,
        command_allowed=True,
        command_valid=True,
    )

    decision = router.receive_position(command_allowed=True)

    assert decision.publish_zero_velocity
    assert decision.publish_position
    assert router.mode == SetpointMode.POSITION


@pytest.mark.parametrize("allowed", [False])
def test_disallowed_flight_state_never_forwards_commands(allowed: bool) -> None:
    router = SetpointRouter(velocity_lease_s=0.3)

    assert not router.receive_position(allowed).publish_position
    assert not router.receive_velocity(
        now_s=1.0,
        command_allowed=allowed,
        command_valid=True,
    ).publish_velocity
    assert router.mode == SetpointMode.IDLE


def test_pilot_takeover_zeroes_once_and_requires_a_fresh_command() -> None:
    router = SetpointRouter(velocity_lease_s=0.3)
    router.receive_velocity(
        now_s=1.0,
        command_allowed=True,
        command_valid=True,
    )

    takeover = router.update_flight_state(command_allowed=False)
    repeated_state = router.update_flight_state(command_allowed=False)
    guided_again = router.update_flight_state(command_allowed=True)

    assert takeover.publish_zero_velocity
    assert not repeated_state.publish_zero_velocity
    assert not guided_again.publish_velocity
    assert router.mode == SetpointMode.IDLE


def test_velocity_lease_expires_at_deadline_and_zeroes_once() -> None:
    router = SetpointRouter(velocity_lease_s=0.3)
    router.receive_velocity(
        now_s=5.0,
        command_allowed=True,
        command_valid=True,
    )

    before = router.expire_velocity(5.299)
    expired = router.expire_velocity(5.3)
    repeated = router.expire_velocity(6.0)

    assert not before.publish_zero_velocity
    assert expired.publish_zero_velocity
    assert not repeated.publish_zero_velocity
    assert router.mode == SetpointMode.IDLE


def test_fresh_velocity_refreshes_lease() -> None:
    router = SetpointRouter(velocity_lease_s=0.3)
    router.receive_velocity(
        now_s=2.0,
        command_allowed=True,
        command_valid=True,
    )
    router.receive_velocity(
        now_s=2.2,
        command_allowed=True,
        command_valid=True,
    )

    assert not router.expire_velocity(2.49).publish_zero_velocity
    assert router.expire_velocity(2.5).publish_zero_velocity


def test_invalid_velocity_revokes_active_owner() -> None:
    router = SetpointRouter(velocity_lease_s=0.3)
    router.receive_velocity(
        now_s=3.0,
        command_allowed=True,
        command_valid=True,
    )

    decision = router.receive_velocity(
        now_s=3.1,
        command_allowed=True,
        command_valid=False,
    )

    assert decision.publish_zero_velocity
    assert not decision.publish_velocity
    assert router.mode == SetpointMode.IDLE


@pytest.mark.parametrize("lease", [0.0, -0.1, math.inf, math.nan])
def test_invalid_velocity_lease_is_rejected(lease: float) -> None:
    with pytest.raises(ValueError):
        SetpointRouter(velocity_lease_s=lease)
