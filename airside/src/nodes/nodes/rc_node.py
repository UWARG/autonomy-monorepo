from rclpy.node import Node
from mavros_msgs.msg import Mavlink, RCIn
import struct
import rclpy
class RCNode(Node):
    def __init__(self):
        super().__init__("rc_node")
        self.rc_subscriber = self.create_subscription(Mavlink, "/uas1/mavlink_source", self.rc_callback, 10)
        self.rc_publisher = self.create_publisher(RCIn, "/rc", 10)

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
        rc_msg=RCIn()
        rc_msg.channels=list(channels)
        rc_msg.header=msg.header
        rc_msg.rssi=255
        self.rc_publisher.publish(rc_msg)

def main(args=None):
    rclpy.init(args=args)
    rc_node=RCNode()
    rclpy.spin(rc_node)
    rc_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

