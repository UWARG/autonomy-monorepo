import logging 
import time 
from pymavlink import mavutil

class MavCommand: 
    def __init__(self, connection: MavConnection) -> None:
        self.connection = connection 

    def change_altitude(self, altitude: float) -> bool: 
        """Send a command to change the drone's altitude."""
    
    def change_yaw(self, yaw: float) -> bool: 
        """Send a command to change the drone's yaw."""

    
