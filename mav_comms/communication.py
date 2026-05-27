from utils import ( 
    AttitudeMessage, 
    PositionMessage,
)

from .connection import MavConnection

class MavComms:
    def __init__(self, connection: MavConnection) -> None:
        self.connection = connection 

    def receive_position(self, position: PositionMessage) -> bool: 
        """Receive a position message from the drone."""

    def receive_attitude(self, attitude: AttitudeMessage) -> bool:
        """Receive an attitude message from the drone. 
            Returns True if a message arrived within the timeout.
        """
        # TODO: `self.connection.master` is the pymavlink mavutil connection
        # that MavConnection will own.
        msg = self.connection.master.recv_match(
            type="ATTITUDE",
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            return False

        attitude.time_boot_ms = msg.time_boot_ms
        attitude.roll = msg.roll
        attitude.pitch = msg.pitch
        attitude.yaw = msg.yaw
        attitude.rollspeed = msg.rollspeed
        attitude.pitchspeed = msg.pitchspeed
        attitude.yawspeed = msg.yawspeed
        return True

    
