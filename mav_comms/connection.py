from .constants import MAVLINK_TCP_HOST, MAVLINK_TCP_PORT

class MavConnection: 

    def __init__(
        self, 
        host: str = MAVLINK_TCP_HOST, 
        port: int = MAVLINK_TCP_PORT, 
    ) -> None: 
        self.host = host 
        self.port = port 

    def is_connected(self) -> bool: 
        """Check if the MAVLink connection is active."""

    def send_heartbeat(self) -> bool: 
        """Send a heartbeat message to maintain the connection."""

    def receive_heartbeat(self) -> bool: 
        """Wait for a heartbeat message from the drone to confirm connection."""

    