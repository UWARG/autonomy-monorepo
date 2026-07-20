from rclpy.node import Node
import rclpy
from geometry_msgs.msg import PositionTarget
from custom_interfaces.msg import Error
import time



class PI():
    def __init__(self,ki,kp,max_integral,max_output):
        self.ki=ki
        self.kp=kp
        self.integral=0
        self.previous_error=0
        self.previous_time=time.time_ns()/1e9
        self.max_integral=max_integral
        self.max_output=max_output
    def update(self,error):
        dt=time.time_ns()/1e9-self.previous_time
        self.previous_time=time.time_ns()/1e9
        if dt<=0:
            return 0
        self.integral+=error*dt
        current_integral=self.integral
        self.integral=max(-self.max_integral,min(self.max_integral,self.integral))
        proportional=self.kp*error
        integral=self.ki*self.integral
        output=proportional+integral
        clamped_output=max(-self.max_output,min(self.max_output,output))
        if clamped_output!=output:
            self.integral=current_integral-error*dt
        return clamped_output
    def update_prev_time(self):
        self.previous_time=time.time_ns()/1e9

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.error_subscriber=self.create_subscription(Error, "/error", self.PI_control, 10)
        self.velocity_publisher=self.create_publisher(PositionTarget, "/setpoint_raw/local", 10)
        self.get_logger().info('Controller node initialized')
        self.pi_x=PI(0.01,0.1,10,10)
        self.pi_y=PI(0.01,0.1,10,10)


    def PI_control(self,error):
        velocity=PositionTarget()
        now = self.get_clock().now()
        velocity.header.stamp = now.to_msg()
        velocity.header.frame_id = "base_link"
        velocity.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        velocity.coordinate_frame=PositionTarget.FRAME_BODY_NED
        #pid loop logic
        if not error.acceptable_consensus:
            velocity.velocity.x=0
            velocity.velocity.y=0
            velocity.velocity.z=-0.1
            self.pi_x.update_prev_time()
            self.pi_y.update_prev_time()
            self.velocity_publisher.publish(velocity)
            return
        velocity.velocity.x=-self.pi_y.update(error.y)
        velocity.velocity.y=self.pi_x.update(error.x)
        velocity.velocity.z=0.1 #change depend on computation time
        self.velocity_publisher.publish(velocity)

def main(args=None):
    rclpy.init(args=args)
    controller=Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()