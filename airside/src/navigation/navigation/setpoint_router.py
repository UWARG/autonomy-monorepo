"""Pure setpoint ownership and velocity-lease state machine."""

from __future__ import annotations

import dataclasses
import enum
import math


class SetpointMode(enum.Enum):
    """Currently active command interface."""

    IDLE = "IDLE"
    POSITION = "POSITION"
    VELOCITY = "VELOCITY"


@dataclasses.dataclass(frozen=True)
class RoutingDecision:
    """Side effects the ROS adapter must apply for one ownership event."""

    publish_zero_velocity: bool = False
    publish_position: bool = False
    publish_velocity: bool = False


class SetpointRouter:
    """Own position/velocity arbitration without depending on ROS or MAVROS."""

    def __init__(self, velocity_lease_s: float) -> None:
        if not math.isfinite(velocity_lease_s) or velocity_lease_s <= 0.0:
            raise ValueError("velocity_lease_s must be finite and positive")
        self.velocity_lease_s = velocity_lease_s
        self.mode = SetpointMode.IDLE
        self.last_velocity_s: float | None = None

    def receive_position(self, command_allowed: bool) -> RoutingDecision:
        """Claim position ownership if flight state permits it."""

        if not command_allowed:
            return self.revoke()
        switched = self.mode == SetpointMode.VELOCITY
        self.mode = SetpointMode.POSITION
        self.last_velocity_s = None
        return RoutingDecision(
            publish_zero_velocity=switched,
            publish_position=True,
        )

    def receive_velocity(
        self,
        *,
        now_s: float,
        command_allowed: bool,
        command_valid: bool,
    ) -> RoutingDecision:
        """Claim or refresh velocity ownership if the command is safe."""

        if not command_allowed or not command_valid:
            return self.revoke()
        switched = self.mode == SetpointMode.POSITION
        self.mode = SetpointMode.VELOCITY
        self.last_velocity_s = now_s
        return RoutingDecision(
            publish_zero_velocity=switched,
            publish_velocity=True,
        )

    def update_flight_state(self, command_allowed: bool) -> RoutingDecision:
        """Release the active interface when the pilot or FC takes control."""

        if command_allowed:
            return RoutingDecision()
        return self.revoke()

    def expire_velocity(self, now_s: float) -> RoutingDecision:
        """Fail closed if the upstream velocity stream misses its lease."""

        if (
            self.mode == SetpointMode.VELOCITY
            and self.last_velocity_s is not None
            and now_s - self.last_velocity_s + 1e-12 >= self.velocity_lease_s
        ):
            return self.revoke()
        return RoutingDecision()

    def revoke(self) -> RoutingDecision:
        """Release ownership and request one explicit zero velocity if active."""

        was_active = self.mode != SetpointMode.IDLE
        self.mode = SetpointMode.IDLE
        self.last_velocity_s = None
        return RoutingDecision(publish_zero_velocity=was_active)
