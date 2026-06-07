import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose, Point, Quaternion, Header

TAG_ID = ""

class MavrosNode(Node):
    def __init__(self):
        super().__init__("mavros_comms")
        self.arming_client = self.create_client(CommandBool, "mavros/cmd/arming")
        self.setmode_client = self.create_client(SetMode, "mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "mavros/cmd/takeoff")

        self.state_subscriber = self.create_subscription(State, "mavros/state", self.state_callback, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, "mavros/setpoint_position/local",10)
        self.apriltag_subscriber = self.create_subscription(TFMessage,"",self.apriltag_callback,10) # run to see what topic apriltag node publishes to

        while not self.setmode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for setmode service...")
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for takeoff service...")
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        self.get_logger().info("All services ready")

    def apriltag_callback(self, msg: TFMessage):
        if len(msg.detections) == 0:
            self.get_logger().info("No apriltag detected")
            return
        for detection in msg.transforms:
            if detection.child_frame_id == TAG_ID:
                apriltag=detection
                break
        if apriltag is None:
            self.get_logger().info("Apriltag not found")
            return
        x=apriltag.transform.translation.x
        y=apriltag.transform.translation.y
        z=apriltag.transform.translation.z
        msg=PoseStamped(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="map"
            ),
            pose=Pose(
                position=Point(x=x,y=y,z=z),
                orientation=Quaternion(x=0,y=0,z=0,w=1)
            )
        )
        self.local_pos_pub.publish(msg)
    
    def arm(self):
        req=CommandBool.Request()
        req.value=True
        future=self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self,future)
        return future.result().success
    
    def set_mode(self, mode: str):
        req=SetMode.Request()
        req.custom_mode=mode
        future=self.setmode_client.call_async(req)
        rclpy.spin_until_future_complete(self,future)
        return future.result().success
    
    def takeoff(self, altitude: float):
        req=CommandTOL.Request()
        req.altitude=altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self,future)
        return future.result().success

            

def main(args=None):
    rclpy.init(args=args)
    node = MavrosNode()
    result=node.set_mode("GUIDED")
    if not result:
        raise RuntimeError("Failed to set mode")
    result=node.arm()
    if not result:
        raise RuntimeError("Failed to arm vehicle")
    result=node.takeoff(10)
    if not result:
        raise RuntimeError("Failed to takeoff")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
