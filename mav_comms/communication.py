from typing import Optional

from utils import (
    AttitudeMessage,
    PositionMessage,
)

from .connection import MavConnection

class MavComms:
    def __init__(self, connection: MavConnection) -> None:
        self.connection = connection

    def receive_position(self) -> Optional[PositionMessage]:
        """Receive a position message from the drone."""

    def receive_attitude(self) -> Optional[AttitudeMessage]:
        """Receive an ATTITUDE (MAVLink id 30) message from the drone.

        Returns a populated AttitudeMessage, or None if no message
        arrived within the timeout.
        """
        # TODO: `self.connection.master` is the pymavlink mavutil connection
        # that MavConnection will own.
        msg = self.connection.master.recv_match(
            type="ATTITUDE",
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            return None

        return AttitudeMessage(
            time_boot_ms=msg.time_boot_ms,
            roll=msg.roll,
            pitch=msg.pitch,
            yaw=msg.yaw,
            rollspeed=msg.rollspeed,
            pitchspeed=msg.pitchspeed,
            yawspeed=msg.yawspeed,
        )
