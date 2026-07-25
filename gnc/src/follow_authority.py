"""Pure, latched authority state machine for production target following."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class AuthorityState(Enum):
    DISABLED = "disabled"
    ACQUIRING = "acquiring"
    ACTIVE = "active"
    BRIEF_LOSS = "brief_loss"
    TERMINAL_BRAKE = "terminal_brake"
    TERMINAL_LOITER = "terminal_loiter"


class AuthorityAction(Enum):
    RELEASE = "release"
    STREAM = "stream"
    ZERO = "zero"
    BRAKE = "brake"
    LOITER = "loiter"


class StopReason(Enum):
    NONE = "none"
    NOT_ENABLED = "not_enabled"
    EXPLICIT_DISABLE = "explicit_disable"
    RESET_TARGET = "reset_target"
    ENABLE_REJECTED = "enable_rejected"
    RC_KILL = "rc_kill"
    MODE_EXIT = "mode_exit"
    FC_STATE_STALE = "fc_state_stale"
    FLIGHT_STATE_EXIT = "disarmed_or_not_airborne"
    PROXIMITY = "proximity_emergency"
    TARGET_LOST = "target_lost"
    OUT_OF_VALIDATED_RANGE = "target_out_of_validated_range"


@dataclass(frozen=True)
class AuthorityConfig:
    fc_state_freshness_s: float = 0.5
    lost_target_timeout_s: float = 1.0
    props_off_bypass_airborne: bool = False


@dataclass(frozen=True)
class AuthorityInputs:
    now_s: float
    fc_state_rx_s: Optional[float]
    connected: bool
    mode: str
    armed: bool
    airborne: bool
    target_valid: bool
    target_out_of_range: bool = False
    proximity_emergency: bool = False
    rc_kill: bool = False


@dataclass(frozen=True)
class AuthorityResult:
    state: AuthorityState
    action: AuthorityAction
    enabled: bool
    clear_target_lock: bool
    stop_reason: StopReason


class FollowAuthority:
    """Own follow authority and require a new enable edge after every release."""

    def __init__(self, config: AuthorityConfig = AuthorityConfig()) -> None:
        self.config = config
        self.state = AuthorityState.DISABLED
        self.stop_reason = StopReason.NOT_ENABLED
        self._loss_started_s: Optional[float] = None
        self._loss_out_of_range = False
        self._rc_enable_high = False
        self._rc_enable_seen = False

    @property
    def enabled(self) -> bool:
        return self.state in (
            AuthorityState.ACQUIRING,
            AuthorityState.ACTIVE,
            AuthorityState.BRIEF_LOSS,
        )

    def _state_fresh(self, inputs: AuthorityInputs) -> bool:
        return (
            inputs.connected
            and inputs.fc_state_rx_s is not None
            and 0.0
            <= inputs.now_s - inputs.fc_state_rx_s
            <= self.config.fc_state_freshness_s
        )

    def can_enable(self, inputs: AuthorityInputs) -> bool:
        flight_ready = self.config.props_off_bypass_airborne or (
            inputs.armed and inputs.airborne
        )
        return (
            self._state_fresh(inputs)
            and inputs.mode == "GUIDED"
            and flight_ready
            and inputs.target_valid
            and not inputs.rc_kill
            and not inputs.proximity_emergency
        )

    def request_enable(self, inputs: AuthorityInputs) -> bool:
        """Handle one explicit service/RC rising-edge event."""
        if self.state is not AuthorityState.DISABLED:
            return False
        if not self.can_enable(inputs):
            self.state = AuthorityState.DISABLED
            self.stop_reason = StopReason.ENABLE_REJECTED
            return False
        self.state = AuthorityState.ACQUIRING
        self.stop_reason = StopReason.NONE
        self._loss_started_s = inputs.now_s
        self._loss_out_of_range = False
        return True

    def update_rc_enable(self, high: bool, inputs: AuthorityInputs) -> bool:
        if not self._rc_enable_seen:
            self._rc_enable_seen = True
            self._rc_enable_high = high
            return False
        rising = high and not self._rc_enable_high
        self._rc_enable_high = high
        return self.request_enable(inputs) if rising else False

    def disable(self, reason: StopReason = StopReason.EXPLICIT_DISABLE) -> None:
        self.state = AuthorityState.DISABLED
        self.stop_reason = reason
        self._loss_started_s = None
        self._loss_out_of_range = False

    def reset_target(self) -> None:
        self.disable(StopReason.RESET_TARGET)

    def _result(self, action: AuthorityAction, clear: bool = False) -> AuthorityResult:
        return AuthorityResult(
            state=self.state,
            action=action,
            enabled=self.enabled,
            clear_target_lock=clear,
            stop_reason=self.stop_reason,
        )

    def step(self, inputs: AuthorityInputs) -> AuthorityResult:
        # A stale state or pilot mode takeover relinquishes the setpoint channel
        # immediately. Returning to GUIDED never re-enables it.
        if not self._state_fresh(inputs):
            was_enabled = self.state is not AuthorityState.DISABLED
            self.disable(StopReason.FC_STATE_STALE)
            return self._result(AuthorityAction.RELEASE, clear=was_enabled)
        if inputs.mode != "GUIDED":
            was_enabled = self.state is not AuthorityState.DISABLED
            self.disable(StopReason.MODE_EXIT)
            return self._result(AuthorityAction.RELEASE, clear=was_enabled)
        if not self.config.props_off_bypass_airborne and (
            not inputs.armed or not inputs.airborne
        ):
            was_enabled = self.state is not AuthorityState.DISABLED
            self.disable(StopReason.FLIGHT_STATE_EXIT)
            return self._result(AuthorityAction.RELEASE, clear=was_enabled)

        if inputs.rc_kill:
            was_enabled = self.state is not AuthorityState.DISABLED
            self.disable(StopReason.RC_KILL)
            return self._result(
                AuthorityAction.ZERO if was_enabled else AuthorityAction.RELEASE,
                clear=was_enabled,
            )

        if self.state is AuthorityState.TERMINAL_BRAKE:
            return self._result(AuthorityAction.BRAKE, clear=True)
        if self.state is AuthorityState.TERMINAL_LOITER:
            return self._result(AuthorityAction.LOITER, clear=True)
        if self.state is AuthorityState.DISABLED:
            return self._result(AuthorityAction.RELEASE)

        if inputs.proximity_emergency:
            self.state = AuthorityState.TERMINAL_BRAKE
            self.stop_reason = StopReason.PROXIMITY
            self._loss_started_s = None
            return self._result(AuthorityAction.BRAKE, clear=True)

        if self.state is AuthorityState.ACQUIRING:
            if inputs.target_valid:
                self.state = AuthorityState.ACTIVE
                self._loss_started_s = None
                self._loss_out_of_range = False
                return self._result(AuthorityAction.STREAM)
            self._loss_out_of_range |= inputs.target_out_of_range
            if (
                self._loss_started_s is not None
                and inputs.now_s - self._loss_started_s
                >= self.config.lost_target_timeout_s
            ):
                self.state = AuthorityState.TERMINAL_LOITER
                self.stop_reason = (
                    StopReason.OUT_OF_VALIDATED_RANGE
                    if self._loss_out_of_range
                    else StopReason.TARGET_LOST
                )
                return self._result(AuthorityAction.LOITER, clear=True)
            return self._result(AuthorityAction.ZERO)

        if inputs.target_valid:
            self.state = AuthorityState.ACTIVE
            self._loss_started_s = None
            self._loss_out_of_range = False
            return self._result(AuthorityAction.STREAM)

        if self._loss_started_s is None:
            self._loss_started_s = inputs.now_s
        self._loss_out_of_range |= inputs.target_out_of_range
        if inputs.now_s - self._loss_started_s >= self.config.lost_target_timeout_s:
            self.state = AuthorityState.TERMINAL_LOITER
            self.stop_reason = (
                StopReason.OUT_OF_VALIDATED_RANGE
                if self._loss_out_of_range
                else StopReason.TARGET_LOST
            )
            return self._result(AuthorityAction.LOITER, clear=True)
        self.state = AuthorityState.BRIEF_LOSS
        return self._result(AuthorityAction.ZERO)
