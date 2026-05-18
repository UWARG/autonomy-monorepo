import logging 
from pymavlink import mavutil 

from utils import MAVLINK_TCP_HOST, MAVLINK_TCP_PORT

class MavConnection: 

    def __init__(
        self, 
        host: str = MAVLINK_TCP_HOST, 
        port: int = MAVLINK_TCP_PORT, 
    ) -> None: 
        self.host = host 
        self.port = port 
    
    def connect(self) -> bool: 
        """Establish a MAVLink connection to the drone."""

    def disconnect(self) -> None: 
        """Close the MAVLink connection."""

    def is_connected(self) -> bool: 
        """Check if the MAVLink connection is active."""

    