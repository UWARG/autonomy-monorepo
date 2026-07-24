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
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

CAM_FORWARD_OFFSET=0.09
CAM_RIGHT_OFFSET=0.08
CAM_DOWN_OFFSET=0.18

TAG_ID = "36h11_1"
class ManagerNode(Node):
    def __init__(self):
        super().__init__("mavros_comms")
        self.mavlink_qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.precision_landing_pub = self.create_publisher(LandingTarget, "/mavros_container/raw",10)
        self.apriltag_subscriber = self.create_subscription(TFMessage,"/tf",self.apriltag_callback,10)
        self.raw_mavlink_subscriber = self.create_subscription(Mavlink, "/uas1/mavlink_source", self.rc_callback, self.mavlink_qos)
        self.create_timer(0.1, self.precision_landing_timer_callback)
        self.landing=False
        self.last_apriltag=None
        self.last_valid_apriltag_time=None


    def precision_landing_timer_callback(self):
        if not self.landing:
            return 
        if self.last_apriltag is None:
            self.get_logger().info("No apriltag detected")
            return
        if self.last_valid_apriltag_time is None:
            self.get_logger().info("No valid apriltag detected")
            return
        if self.get_clock().now() - self.last_valid_apriltag_time > Duration(seconds=0.3):
            self.get_logger().info("Apriltag too old")
            self.last_apriltag=None
            return
        apriltag=LandingTarget()
        apriltag.header.stamp=self.last_valid_apriltag_time.to_msg()
        apriltag.frame=12
        apriltag.type=2 # vision_fiducial = 2
        #apriltag coordinate system to FRD
        apriltag.pose.position.x=-self.last_apriltag.transform.translation.y+CAM_FORWARD_OFFSET
        #landing target plugin negates y and z so we need to negate them back since ros body convention expects FLU 
        apriltag.pose.position.y=-(self.last_apriltag.transform.translation.x+CAM_RIGHT_OFFSET)
        apriltag.pose.position.z=-(self.last_apriltag.transform.translation.z+CAM_DOWN_OFFSET)
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
        if msg.msgid!=65 and msg.msgid!=70:
            return
        payload_bytes=bytearray()
        for val in msg.payload64:
            payload_bytes.extend(struct.pack("<Q",val&0xFFFFFFFFFFFFFFFF))
        payload_bytes=payload_bytes[:msg.len]
        if msg.msgid==70:
            try:
                data = struct.unpack("<8H BB 8H", payload_bytes)
            except Exception as e:
                self.get_logger().error(f"Failed to unpack Mavlink message: {e}")
                return
            channels = data[0:8] + data[10:20]
            if not channels:
                self.get_logger().info("No channels received")
                return
        else: 
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

    def apriltag_callback(self, msg: TFMessage):
        apriltag = None
        if len(msg.transforms) == 0:
            self.get_logger().info("No transforms detected")
            self.last_apriltag = None
            return
        apriltag = msg.transforms[0]
        if apriltag is None:
            self.get_logger().info("Apriltag not found")
            self.last_apriltag = None
            return
        self.last_apriltag = apriltag
        self.last_valid_apriltag_time=self.get_clock().now()
            

def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    try:
        node.get_logger().info("Starting manager node")
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
