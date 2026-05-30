from .constants import MAVLINK_TCP_HOST, MAVLINK_TCP_PORT
from pymavlink import mavutil
class MavConnection: 

    def __init__(
        self, 
        baud_rate: int,
        host: str = MAVLINK_TCP_HOST, 
        port: int = MAVLINK_TCP_PORT, 
    ) -> None: 
        self.host = host 
        self.port = port 
        self.baud_rate = baud_rate
        self.master = None

    def is_connected(self) -> bool: 
        """Check if the MAVLink connection is active."""
        return self.master is not None

    def send_heartbeat(self) -> bool: 
        """Send a heartbeat message to maintain the connection."""

    def receive_heartbeat(self) -> bool: 
        """Wait for a heartbeat message from the drone to confirm connection."""

    def connect(self) -> bool: 
        """Establish a connection to the MAVLink server."""
    
        connection_string = f"tcp:{self.host}:{self.port}"
       

        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=self.baud_rate)
            self.master.wait_heartbeat()  # blocks until heartbeat received, raises on timeout
            return True
        except Exception as e:
            print(f"Error occurred while connecting to MAVLink server: {e}")
            return False

    def disconnect(self) -> bool: 
        """Close the connection to the MAVLink server."""
        if self.master is None:
            print("No active connection to disconnect.")
            return False
        try:
            self.master.close()
            self.master = None
            return True
        except Exception as e:
            print(f"Error occurred while disconnecting from MAVLink server: {e}")
            return False