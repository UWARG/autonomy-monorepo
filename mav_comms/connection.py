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
        self.master.mav.heartbeat_send(
                                        mavutil.mavlink.MAV_TYPE_GCS,        
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
                                        0, 0, 0
                                    )
        
    def receive_heartbeat(self) -> bool: 
        """Wait for a heartbeat message from the drone to confirm connection."""
        while(not self.master.wait_heartbeat()):
            pass
        print ("Heartbeat received from drone.")
        return True


    def connect(self) -> bool: 
        """Establish a connection to the MAVLink server."""
    
        connection_string = f"{self.host}:{self.port}"
       

        try:
            self.master = mavutil.mavlink_connection(connection_string)
              # blocks until heartbeat received, raises on timeout
            receive_hearbeat_success = self.receive_heartbeat()

            #TO DO: currently timer never returns false, maybe add watchdog timer to prevent infinite loop
            if not receive_hearbeat_success:
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