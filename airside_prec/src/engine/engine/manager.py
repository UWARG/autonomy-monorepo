import math
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from tf2_msgs.msg import TFMessage
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import LandingTarget
from mavros_msgs.srv import MessageInterval
from mavros_msgs.msg import Mavlink
import struct
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

TAG_ID = "36h11_1"
class ManagerNode(Node):
    def __init__(self):
        super().__init__("mavros_comms")
        self.set_mode_client=self.create_client(SetMode,"/set_mode")
        self.mavlink_qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.precision_landing_pub = self.create_publisher(LandingTarget, "/mavros_container/raw",10)
        self.apriltag_subscriber = self.create_subscription(TFMessage,"/tf",self.apriltag_callback,10)
        self.raw_mavlink_subscriber = self.create_subscription(Mavlink, "/uas1/mavlink_source", self.rc_callback, self.mavlink_qos)
        self._mode_change_pending=False
        self.create_timer(0.1, self.precision_landing_timer_callback)
        self.landing=False
        self.last_apriltag=None

        while not self.set_mode_client.wait_for_service(timeout_sec=10):
            self.get_logger().info("Waiting for set mode service...")
        self.get_logger().info("All services ready")

    def precision_landing_timer_callback(self):
        if True: #replace with self.landing 
            if self.last_apriltag is None:
                self.get_logger().info("No apriltag detected")
                return
            apriltag=LandingTarget()
            apriltag.header.stamp=self.get_clock().now().to_msg()
            apriltag.frame=12
            apriltag.type=2 # vision_fiducial = 2
            #apriltag coordinate system to FRD
            apriltag.pose.position.x=self.last_apriltag.transform.translation.z
            apriltag.pose.position.y=self.last_apriltag.transform.translation.x
            apriltag.pose.position.z=self.last_apriltag.transform.translation.y
            apriltag.distance=math.sqrt(
                self.last_apriltag.transform.translation.y**2+
                self.last_apriltag.transform.translation.x**2+
                self.last_apriltag.transform.translation.z**2
            )
            apriltag.pose.orientation.x=0.0
            apriltag.pose.orientation.y=0.0
            apriltag.pose.orientation.z=0.0
            apriltag.pose.orientation.w=1.0
            self.precision_landing_pub.publish(apriltag)
        

    def rc_callback(self, msg: Mavlink):
        if msg.msgid!=65:
            return
        payload_bytes=bytearray()
        for val in msg.payload64:
            payload_bytes.extend(struct.pack("<Q",val&0xFFFFFFFFFFFFFFFF))
        payload_bytes=payload_bytes[:msg.len]
        try:
            data=struct.unpack("<I 18H BB", payload_bytes)
        except Exception as e:
            self.get_logger().error(f"Failed to unpack Mavlink message: {e}")
            return
        
        channels=data[1:19]
        if not channels:
            self.get_logger().info("No channels received")
            return
        want_landing = channels[6] > 1500
        if want_landing != self.landing:
            self.get_logger().info(f"Landing {want_landing}")
            self.landing = want_landing
            self.set_mode_async("LAND" if want_landing else "LOITER")
    


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
    
    def set_mode_async(self, mode: str):
        if self._mode_change_pending:
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        self._mode_change_pending = True
        future.add_done_callback(self._on_set_mode_done)

    def _on_set_mode_done(self, future):
        self._mode_change_pending = False

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
        node.get_logger().info("Starting manager node")
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
