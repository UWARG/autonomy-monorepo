from .constants import MAVLINK_TCP_HOST, MAVLINK_TCP_PORT
from pymavlink import mavutil
import time
class MavConnection: 

    def __init__(
        self, 
        baud_rate: int,
    ) -> None: 
        self.baud_rate = baud_rate
        self.master = None
        self.heartbeat_interval = 1 
        self.heartbeat_timeout = 5 

    def is_connected(self) -> bool: 
        """Check if the MAVLink connection is active."""
        return self.master is not None

    def send_heartbeat(self) -> None: 
        """Send a heartbeat message to maintain the connection."""
        self.master.mav.heartbeat_send(
                                        mavutil.mavlink.MAV_TYPE_GCS,        
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
                                        0, 0, 0
                                    )
        
    def receive_heartbeat(self) -> bool:
        """Wait for a heartbeat message from the drone to confirm connection."""
        start_time = time.time()
        while time.time() - start_time < self.heartbeat_timeout:
            if self.master.wait_heartbeat(timeout=self.heartbeat_interval):
                print("Heartbeat received from drone.")
                return True
        print("Timed out waiting for heartbeat from drone.")
        return False


    def connect(self) -> bool: 
        """Establish a connection to the MAVLink server."""
    
        connection_string = "/dev/ttyAMA0"
       

        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=self.baud_rate)
            receive_heartbeat_success = self.receive_heartbeat()

            if not receive_heartbeat_success:
                print("Failed to receive heartbeat from drone.")
                return False
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