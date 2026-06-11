import math
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from tf2_msgs.msg import TFMessage
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import LandingTarget


TAG_ID = "36h11_1"
class ManagerNode(Node):
    def __init__(self):
        super().__init__("mavros_comms")
        self.arming_client = self.create_client(CommandBool, "/mavros_container/arming")
        self.setmode_client = self.create_client(SetMode, "/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros_container/takeoff")

        self.precision_landing_pub = self.create_publisher(LandingTarget, "/mavros/UAS1/landing_target/raw",10)
        self.apriltag_subscriber = self.create_subscription(TFMessage,"/tf",self.apriltag_callback,10) # run to see what topic apriltag node publishes to
        self.rc_subscriber = self.create_subscription(RCIn, "/mavros/UAS1/rc/in", self.rc_callback, 10)

        self.create_timer(0.1, self.precision_landing_timer_callback)
        self.landing=False
        self.last_apriltag=None


        while not self.setmode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set mode service...")
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for takeoff service...")
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        self.get_logger().info("All services ready")


    def precision_landing_timer_callback(self):
        if self.landing:
            if self.last_apriltag is None:
                self.get_logger().info("No apriltag detected")
                return
            apriltag=LandingTarget()
            apriltag.header.stamp=self.get_clock().now().to_msg()
            apriltag.frame=9 # BODY_NED = FRD = 9
            apriltag.type=2 # vision_fiducial = 2
            apriltag.pose.position.x=-self.last_apriltag.transform.translation.y
            apriltag.pose.position.y=self.last_apriltag.transform.translation.x
            apriltag.pose.position.z=self.last_apriltag.transform.translation.z
            apriltag.distance=math.sqrt(
                self.last_apriltag.transform.translation.y**2+
                self.last_apriltag.transform.translation.x**2+
                self.last_apriltag.transform.translation.z**2
            )
            apriltag.pose.orientation.x=0
            apriltag.pose.orientation.y=0
            apriltag.pose.orientation.z=0
            apriltag.pose.orientation.w=1
            self.precision_landing_pub.publish(apriltag)
        

    def rc_callback(self, msg: RCIn):
        if not msg.channels:
            self.get_logger().info("No channels received")
            return
        if msg.channels[0]>1500:
            self.landing=True
            self.set_mode("LAND") # Set ardupilot pland param to 1
        if msg.channels[0]<1500:
            self.landing=False


    def apriltag_callback(self, msg: TFMessage):
        apriltag = None
        if len(msg.transforms) == 0:
            self.get_logger().info("No transforms detected")
            return
        apriltag=msg.transforms[0]
        if apriltag is None:
            self.get_logger().info("Apriltag not found")
            return
        self.last_apriltag = apriltag
    
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
    node = ManagerNode()
    """
    result=node.set_mode("GUIDED")
    if not result:
        raise RuntimeError("Failed to set mode")
    result=node.arm()
    if not result:
        raise RuntimeError("Failed to arm vehicle")
    result=node.takeoff(10)
    if not result:
        raise RuntimeError("Failed to takeoff")
    """
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
