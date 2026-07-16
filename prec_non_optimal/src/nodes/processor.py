import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from sensor_msgs.msg import Imu
import cv2
import numpy as np
from mavros_msgs.msg import Mavlink
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from custom_interfaces.action import Takeoff
from custom_interfaces.action import Landing
from enums import Status
from std_msgs.msg import Float64
from custom_interfaces.msg import Error


class Processor(Node):
    def __init__(self) -> None:
        super().__init__("processor")
        self._cb_group=ReentrantCallbackGroup()
        self.image_subscriber = self.create_subscription(Image, "camera/image", self.process, 10, callback_group=self._cb_group)
        self.imu_subscriber = self.create_subscription(Imu, "imu/data", self.imu_callback, 10, callback_group=self._cb_group)
        self.takeoff_server=ActionServer(self, Takeoff, "takeoff", self.takeoff_callback, callback_group=self._cb_group)
        self.landing_server=ActionServer(self, Landing, "landing", self.landing_callback, callback_group=self._cb_group)

        self._fix_sub = self.create_subscription(NavSatFix,"global_position/global",self.fix_callback,qos_profile_sensor_data,callback_group=self._cb_group)
        self._rel_alt_sub = self.create_subscription(Float64,"global_position/rel_alt",self.rel_alt_callback,qos_profile_sensor_data,callback_group=self._cb_group)

        self.error_publisher=self.create_publisher(Error, "error", 10, callback_group=self._cb_group)

        self.imu_dict={0:[]}
        self.latitude=0.0
        self.longitude=0.0
        self.rel_alt=None
        self.last_altitude=0
        self.altitude_threshold=0.1
        self.status=Status.TAKEOFF
        self.last_image_altitude=7.5
        self.orb=cv2.ORB_create(nfeatures=1000)
        self._bridge=CvBridge()
        self.camera_intrinsics()
        self.get_logger().info("Processor initialized")
        self.roll=None
        self.pitch=None
        self.takeoff_goal_handle=None
        self.landing_goal_handle=None
        self.image_rate=0.1 #meters/image
        self.error_margin=0.02 #meters

    

    def fix_callback(self, msg: NavSatFix):
        self.latitude=msg.latitude
        self.longitude=msg.longitude

    def rel_alt_callback(self, msg: Float64):
        if msg.data<1.0:
            self.image_rate=0.1
            self.error_margin=0.02
        else:
            self.image_rate=0.25
            self.error_margin=0.05
        self.rel_alt=msg.data

    def landing_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Landing requested")
        self.landing_goal_handle=goal_handle
        result=Landing.Result()
        rate=self.create_rate(10)

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.success=False
                    goal_handle.canceled()
                    return result
                rate.sleep()
        finally:
            self.landing_goal_handle=None

    def takeoff_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Takeoff requested")
        self.takeoff_goal_handle=goal_handle
        self.last_altitude=0
        result=Takeoff.Result()
        rate=self.create_rate(10)

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.success=False
                    result.latitude=self.latitude
                    result.longitude=self.longitude
                    result.altitude=self.rel_alt if self.rel_alt is not None else 0.0
                    goal_handle.canceled()
                    return result
                if self.last_altitude>=self.last_image_altitude:
                    result.success=True
                    result.latitude=self.latitude
                    result.longitude=self.longitude
                    result.altitude=self.rel_alt if self.rel_alt is not None else 0.0
                    goal_handle.succeed()
                    return result
                rate.sleep()
        finally:
            self.takeoff_goal_handle=None

    def process(self, image: Image):
        if self.takeoff_goal_handle is None:
            return
        if self.rel_alt is None or self.pitch is None or self.roll is None:
            return
        if self.last_altitude>self.last_image_altitude:
            return
        if self.takeoff_goal_handle:
            if self.rel_alt-self.last_altitude>=self.image_rate-self.error_margin:
                gray=self.undistort_image(image)
                kp,des=self.generate_orb_descriptors(gray)
                if kp is None or des is None:
                    return
                self.imu_dict[self.rel_alt]=[kp,des,self.roll,self.pitch]
                self.last_altitude=self.rel_alt
        elif self.landing_goal_handle:
            pass



    def generate_orb_descriptors(self, image: Image):
        kp,des=self.orb.detectAndCompute(image, None)
        if not kp:
            return None, None
        kp_pts=np.array([kp.pt for kp in kp])
        return kp_pts,des
        
    def undistort_image(self, image: Image):
        image=self._bridge.imgmsg_to_cv2(image, "bgr8")
        dst=cv2.remap(image, self.mapx, self.mapy, cv2.INTER_LINEAR)
        x,y,w,h=self.roi
        dst=dst[y:y+h, x:x+w]
        gray=cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
        return gray
    
    def imu_callback(self, msg: Imu):
        if self.takeoff_goal_handle is None and self.landing_goal_handle is None:
            return
        q=msg.orientation
        if q.x==0.0 and q.y==0.0 and q.z==0.0 and q.w==0.0:
            self.get_logger().error("No orientation received")
            self.roll=None
            self.pitch=None
            return
        quaternion=[q.x, q.y, q.z, q.w]
        euler=R.from_quat(quaternion).as_euler("xyz", degrees=False)
        self.roll=euler[0]
        self.pitch=euler[1]
    
    def camera_intrinsics(self):
        with open(os.path.join(get_package_share_directory("engine"),"camera_info.yaml"), "r") as f:
            camera_info = yaml.safe_load(f)
            self.height=camera_info["height"]
            self.width=camera_info["width"]
            self.distortion_model=camera_info["distortion_model"]
            self.k=camera_info["camera_matrix"]["data"]
            self.d=camera_info["distortion_coefficients"]["data"]
            self.r=camera_info["rectification_matrix"]["data"]
            self.p=camera_info["projection_matrix"]["data"]
        reshaped_k=np.array(self.k).reshape(3,3)
        reshaped_d=np.array(self.d)
        self.new_camera_matrix,self.roi=cv2.getOptimalNewCameraMatrix(reshaped_k, reshaped_d, (self.width, self.height), 0)
        self.mapx,self.mapy=cv2.initUndistortRectifyMap(reshaped_k, reshaped_d, None, self.new_camera_matrix, (self.width, self.height), cv2.CV_32FC1)

if __name__ == "__main__":
    rclpy.init()
    processor=Processor()
    executor=MultiThreadedExecutor()
    executor.add_node(processor)
    try:
        executor.spin()
    finally:
        processor.destroy_node()
        rclpy.shutdown()
