from utils import MavlinkMessageType

from .constants import MAVLINK_TCP_HOST, MAVLINK_TCP_PORT

class MavConnection:

    def __init__(
        self,
        host: str = MAVLINK_TCP_HOST,
        port: int = MAVLINK_TCP_PORT,
    ) -> None:
        self.host = host
        self.port = port

    def connect(self) -> bool:
        """Open the MAVLink connection and block until the first heartbeat is received."""

    def is_connected(self) -> bool:
        """Check if the MAVLink connection is active."""

    def send_heartbeat(self) -> bool:
        """Send a heartbeat message to maintain the connection."""

    def receive_heartbeat(self) -> bool:
        """Wait for a heartbeat message from the drone to confirm connection."""

    def request_stream(self, stream_type: MavlinkMessageType, rate_hz: int) -> bool:
        """Request the drone start sending a MAVLink message stream at the given rate."""

