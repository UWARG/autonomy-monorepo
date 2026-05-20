import time 
from pymavlink import mavutil

from utils import ( 
    AttitudeMessage, 
    PositionMessage,
)

from .connection import MavConnection

class MavComms:
    def __init__(self, connection: MavConnection) -> None:
        self.connection = connection 

    def send_position(self, position: PositionMessage) -> bool: 
        """Send a position message to the drone."""

    def send_attitude(self, attitude: AttitudeMessage) -> bool: 
        """Send an attitude message to the drone."""
    

    
