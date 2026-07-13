import rclpy
from rclpy.node import Node
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
import struct
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from enums import Status


class Processor(Node):
    def __init__(self) -> None:
        super().__init__("processor")
        self.image_subscriber = self.create_subscription(Image, "camera/image", self.process, 10)
        self.altitude_subscriber = self.create_subscription(NavSatFix, "global_position/global", self.altitude_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)
        self.status_subscriber = self.create_subscription(String,"/uas_status",self.status_callback,10)
        self.imu_dict={0:[]}
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

    def status_callback(self, msg: String):
        self.status=Status(msg.data)

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
        if not msg.orientation:
            self.get_logger().error("No orientation received")
            self.roll=None
            self.pitch=None
            return
        quaternion=[msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler=R.from_quat(quaternion).as_euler("xyz", degrees=False)
        self.roll=euler[0]
        self.pitch=euler[1]

    def altitude_callback(self, msg: NavSatFix):
        self.altitude = msg.altitude
        if self.altitude<0.1:
            self.altitude_threshold=0.1
        elif self.altitude<7.5:
            self.altitude_threshold=0.25
    
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
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()