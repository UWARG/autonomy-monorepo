from utils import (
    Attitude,
    MavStatusType,
    Coordinate,
    RcChannelsMessage,
)

from .connection import MavConnection

class MavComms:
    def __init__(self, connection: MavConnection) -> None:
        self.connection = connection

    def receive_position(self, position: Coordinate) -> bool:
        """Receive a position message from the drone."""
        return False

    def receive_attitude(self, attitude: Attitude) -> bool:
        """Receive an attitude message from the drone."""
        return False

    def receive_heading(self) -> float | None:
        """Receive the drone's compass heading in degrees, or None if unavailable."""

    def receive_rc_channels(self, rc_channels: RcChannelsMessage) -> bool:
        """Receive an RC_CHANNELS message from the drone."""
        return False

    def send_message(self, text: str, message_type: MavStatusType) -> bool:
        """Send a general-purpose text message to the ground station over STATUSTEXT."""
        return False

    def receive_message(self) -> str | None:
        """Receive a general-purpose text message from the ground station over STATUSTEXT, or None if unavailable."""


