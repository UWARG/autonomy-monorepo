from rclpy.node import Node
import rclpy
from mavros_msgs.msg import PositionTarget
from custom_interfaces.msg import Error
import time

HZ=20

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
        self.create_timer(1/HZ, self.publish_velocity)
        self.get_logger().info('Controller node initialized')
        self.pi_x=PI(0.01,0.1,10,10)
        self.pi_y=PI(0.01,0.1,10,10)
        self.vx=0
        self.vy=0
        self.vz=0

    def publish_velocity(self):
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
        velocity.velocity.x=self.vx
        velocity.velocity.y=self.vy
        velocity.velocity.z=self.vz
        self.velocity_publisher.publish(velocity)

    def PI_control(self,error):
        if error.below_last_landing_altitude:
            self.vx=0
            self.vy=0
            self.vz=-0.1
            self.pi_x.update_prev_time()
            self.pi_y.update_prev_time()
            return
        #pid loop logic
        if not error.valid_error:
            #ascend a bit to increase fov
            self.vz=-0.05
            self.pi_x.update_prev_time()
            self.pi_y.update_prev_time()
            return
        self.vx=-self.pi_y.update(error.y)
        self.vy=self.pi_x.update(error.x)
        self.vz=0.1 #change depend on computation time

def main(args=None):
    rclpy.init(args=args)
    controller=Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()