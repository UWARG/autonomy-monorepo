from utils import (
    AttitudeMessage,
    MavStatusType,
    PositionMessage,
)

from .connection import MavConnection

class MavComms:
    def __init__(self, connection: MavConnection) -> None:
        self.connection = connection

    def receive_position(self, position: PositionMessage) -> bool:
        """Receive a position message from the drone."""

    def receive_attitude(self, attitude: AttitudeMessage) -> bool:
        """Receive an attitude message from the drone."""

    def receive_heading(self) -> float | None:
        """Receive the drone's compass heading in degrees, or None if unavailable."""

    def send_message(self, text: str, message_type: MavStatusType) -> bool:
        """Send a general-purpose text message to the ground station over STATUSTEXT."""

    def receive_message(self) -> str | None:
        """Receive a general-purpose text message from the ground station over STATUSTEXT, or None if unavailable."""


