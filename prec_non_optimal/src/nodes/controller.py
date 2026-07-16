from rclpy.node import Node
import rclpy
from geometry_msgs.msg import TwistStamped
from custom_interfaces.msg import Error


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.error_subscriber=self.create_subscription(Error, "/error", self.PI_control, 10)
        self.velocity_publisher=self.create_publisher(TwistStamped, "/setpoint_velocity/cmd_vel", 10)
        self.get_logger().info('Controller node initialized')

    def PI_control(self,error):
        #pid loop logic
        
        velocity=TwistStamped()
        velocity.linear.z=0.1 #change depend on computation time
        velocity.twist.angular.x=0
        velocity.twist.angular.y=0
        velocity.twist.angular.z=0
        self.velocity_publisher.publish(velocity)

def main(args=None):
    rclpy.init(args=args)
    controller=Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()