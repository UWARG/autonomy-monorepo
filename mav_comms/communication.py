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
        """Receive an attitude message from the drone."""
    

    
