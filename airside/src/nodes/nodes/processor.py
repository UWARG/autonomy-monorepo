import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from sensor_msgs.msg import Imu
import cv2
import numpy as np
from sortedcontainers import SortedDict
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from custom_interfaces.action import Takeoff
from custom_interfaces.action import Landing
from std_msgs.msg import Float64
from custom_interfaces.msg import Error
import math

ACCEPTABLE_OFFSET=0.05

class Processor(Node):
    def __init__(self) -> None:
        super().__init__("processor")
        self._cb_group=ReentrantCallbackGroup()
        self._mutual_cb_group=MutuallyExclusiveCallbackGroup()
        self.image_subscriber = self.create_subscription(Image, "camera/image", self.image_callback, 1, callback_group=self._cb_group)
        self.imu_subscriber = self.create_subscription(Imu, "imu/data", self.imu_callback, 10, callback_group=self._cb_group)
        self.takeoff_server=ActionServer(self, Takeoff, "takeoff", self.takeoff_callback, callback_group=self._cb_group)
        self.landing_server=ActionServer(self, Landing, "landing", self.landing_callback, callback_group=self._cb_group)

        self._fix_sub = self.create_subscription(NavSatFix,"global_position/global",self.fix_callback,qos_profile_sensor_data,callback_group=self._cb_group)
        self._rel_alt_sub = self.create_subscription(Float64,"global_position/rel_alt",self.rel_alt_callback,qos_profile_sensor_data,callback_group=self._cb_group)

        self.error_publisher=self.create_publisher(Error, "error", 10, callback_group=self._cb_group)

        self.create_timer(0.1, self.process, callback_group=self._mutual_cb_group)

        self.imu_dict=SortedDict()
        self.image=None
        self.latitude=0.0
        self.longitude=0.0
        self.rel_alt=None
        self.last_altitude=0
        self._use_cuda=False
        try:
             self._use_cuda = hasattr(cv2, "cuda") and cv2.cuda.getCudaEnabledDeviceCount() > 0
        except Exception:
             self._use_cuda = False
        self.BFMatcher = cv2.cuda.DescriptorMatcher_createBFMatcher(cv2.NORM_HAMMING) if self._use_cuda else cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.altitude_threshold=0.1
        self.last_image_altitude=7.5
        self.orb=cv2.ORB_create(nfeatures=1000)
        self._bridge=CvBridge()
        self.camera_intrinsics()
        self.get_logger().info("Processor initialized")
        self.roll=None
        self.pitch=None
        self.takeoff_goal_handle=None
        self.landing_goal_handle=None
        self.landing_complete=False
        self.landing_failed=False
        self.image_rate=0.1 #meters/image
        self.error_margin=0.02 #meters
        self.landing_3d_points=[]
        self.takeoff_3d_points=[]
        self.last_landing_altitude=0.25
        self.align_altitude=1.5
        self.min_inlier_ratio=0.6

    def publish_invalid_error(self, align_before_descent: bool=False):
        self.error_publisher.publish(Error(
            x=0.0,y=0.0,angle=0.0,valid_error=False,
            below_last_landing_altitude=False,align_before_descent=align_before_descent,
            landing_complete=False,
        ))

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

    def fail_landing(self, reason: str):
        self.get_logger().error(reason)
        self.error_publisher.publish(Error(
            x=0.0,y=0.0,angle=0.0,valid_error=False,
            below_last_landing_altitude=False,align_before_descent=False,
            landing_complete=True,
        ))
        self.landing_failed=True

    def landing_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Landing requested")
        self.landing_goal_handle=goal_handle
        self.landing_complete=False
        self.landing_failed=False
        result=Landing.Result()
        if not self.imu_dict:
            self.fail_landing("Empty teach map; cannot land")
            result.success=False
            goal_handle.abort()
            self.landing_goal_handle=None
            return result
        rate=self.create_rate(10)
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.success=False
                    goal_handle.canceled()
                    return result
                if self.landing_failed:
                    result.success=False
                    goal_handle.abort()
                    return result
                if self.landing_complete:
                    result.success=True
                    goal_handle.succeed()
                    return result
                rate.sleep()
        finally:
            self.landing_goal_handle=None

    def takeoff_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Takeoff requested")
        self.takeoff_goal_handle=goal_handle
        self.last_altitude=0
        self.imu_dict.clear()
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

    def pixel_to_3d(self, x: float, y: float, roll: float, pitch: float, alt:float):
        x_px=x-self.cx
        y_px=y-self.cy
        pz=alt*math.cos(pitch)*math.cos(roll)
        px=(-x_px-math.sin(pitch)*self.fx)*pz/self.fx
        py=(-y_px+math.sin(roll)*self.fy)*pz/self.fy
        return px,py
    
    def image_callback(self, image: Image):
        self.image=image

    def process(self):
        if self.image is None:
            return
        if self.takeoff_goal_handle is None and self.landing_goal_handle is None:
            return
        if self.rel_alt is None or self.pitch is None or self.roll is None:
            return
        if self.last_altitude>self.last_image_altitude and self.takeoff_goal_handle:
            return
        if self.takeoff_goal_handle:
            #snapshot
            image=self.image
            rel_alt=self.rel_alt
            roll=self.roll
            pitch=self.pitch
            if rel_alt-self.last_altitude>=self.image_rate-self.error_margin:
                gray=self.undistort_image(image)
                kp,des=self.generate_orb_descriptors(gray)
                if kp is None or des is None:
                    return
                self.imu_dict[rel_alt]=[kp,des,roll,pitch]
                self.last_altitude=rel_alt
        elif self.landing_goal_handle:
            #snapshot
            image=self.image
            rel_alt=self.rel_alt
            land_roll=self.roll
            land_pitch=self.pitch
            align_before_descent=rel_alt<=self.align_altitude
            if not self.imu_dict:
                self.fail_landing("Empty map")
                return
            if rel_alt<=0.0+ACCEPTABLE_OFFSET:
                self.error_publisher.publish(Error(
                    x=0.0,y=0.0,angle=0.0,valid_error=False,
                    below_last_landing_altitude=False,align_before_descent=False,
                    landing_complete=True,
                ))
                self.landing_complete=True
                return
            if rel_alt<=self.last_landing_altitude:
                self.error_publisher.publish(Error(
                    x=0.0,y=0.0,angle=0.0,valid_error=False,
                    below_last_landing_altitude=True,align_before_descent=False,
                    landing_complete=False,
                ))
                return
            index=self.imu_dict.bisect_right(rel_alt)-1
            if index<0:
                self.get_logger().error("No takeoff key found")
                self.publish_invalid_error(align_before_descent)
                return
            key,entry=self.imu_dict.peekitem(index)
            kp_takeoff,des_takeoff,takeoff_roll,takeoff_pitch=entry
            gray=self.undistort_image(image)
            kp,des=self.generate_orb_descriptors(gray)
            if kp is None or des is None:
                self.publish_invalid_error(align_before_descent)
                return
            if kp_takeoff is None or des_takeoff is None or takeoff_roll is None or takeoff_pitch is None:
                self.publish_invalid_error(align_before_descent)
                return
            if self._use_cuda:
                gpu_landing_des=cv2.cuda.GpuMat()
                gpu_takeoff_des=cv2.cuda.GpuMat()
                gpu_landing_des.upload(des)
                gpu_takeoff_des.upload(des_takeoff)
                matches=self.BFMatcher.match(gpu_landing_des, gpu_takeoff_des)
            else:
                matches=self.BFMatcher.match(des, des_takeoff)
            matches=sorted(matches, key=lambda x: x.distance)
            if len(matches)<50:
                self.publish_invalid_error(align_before_descent)
                return
            good_matches=matches[:50]
            self.takeoff_3d_points=[]
            self.landing_3d_points=[]
            for match in good_matches:
                x_land_px,y_land_px=kp[match.queryIdx]
                x_takeoff_px,y_takeoff_px=kp_takeoff[match.trainIdx]
                if x_land_px is None or y_land_px is None or x_takeoff_px is None or y_takeoff_px is None:
                    continue
                self.get_logger().info(f"Landing match: {x_land_px}, {y_land_px}, {x_takeoff_px}, {y_takeoff_px}")
                x_land_3d,y_land_3d=self.pixel_to_3d(x_land_px,y_land_px,land_roll,land_pitch,rel_alt)
                x_takeoff_3d,y_takeoff_3d=self.pixel_to_3d(x_takeoff_px,y_takeoff_px,takeoff_roll,takeoff_pitch,key)
                self.takeoff_3d_points.append([x_takeoff_3d,y_takeoff_3d])
                self.landing_3d_points.append([x_land_3d,y_land_3d])
            if len(self.takeoff_3d_points)<50:
                self.publish_invalid_error(align_before_descent)
                return
            #implement RANSAC 
            H,inliers=cv2.estimateAffinePartial2D( #vector points from takeoff to landing so the translation correction should be negative in the x and y direction
                np.asarray(self.takeoff_3d_points,dtype=np.float32),
                np.asarray(self.landing_3d_points,dtype=np.float32),
                method=cv2.RANSAC,
                ransacReprojThreshold=0.02,
                maxIters=1000,
                confidence=0.99,
                refineIters=10,
                )
            if H is None or inliers is None:
                self.publish_invalid_error(align_before_descent)
                return
            inlier_ratio=float(np.count_nonzero(inliers))/len(inliers)
            if inlier_ratio<self.min_inlier_ratio:
                self.publish_invalid_error(align_before_descent)
                return
            translation_x=H[0,2]
            translation_y=H[1,2]
            rotation_angle=math.atan2(H[1,0], H[0,0])
            scale=math.sqrt(H[0,0]**2 + H[1,0]**2)
            if scale<=1.0-self.error_margin or scale>=1.0+self.error_margin:
                self.publish_invalid_error(align_before_descent)
                return
            error=Error()
            error.x=translation_x
            error.y=translation_y
            error.angle=rotation_angle
            error.valid_error=True
            error.below_last_landing_altitude=False
            error.align_before_descent=align_before_descent
            error.landing_complete=False
            self.error_publisher.publish(error)

    def generate_orb_descriptors(self, image: Image):
        kp,des=self.orb.detectAndCompute(image, None)
        if not kp:
            return None, None
        kp_pts=np.array([kp.pt for kp in kp])
        return kp_pts,des
        
    def undistort_image(self, image: Image):
        image=self._bridge.imgmsg_to_cv2(image, "rgb8")
        dst=cv2.remap(image, self.mapx, self.mapy, cv2.INTER_LINEAR)
        x,y,w,h=self.roi
        dst=dst[y:y+h, x:x+w]
        gray=cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)
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
        x,y,w,h=self.roi
        self.fx=self.new_camera_matrix[0,0]
        self.fy=self.new_camera_matrix[1,1]
        self.cx=self.new_camera_matrix[0,2]-x
        self.cy=self.new_camera_matrix[1,2]-y

def main(args=None):
    rclpy.init(args=args)
    processor=Processor()
    executor=MultiThreadedExecutor()
    executor.add_node(processor)
    try:
        executor.spin()
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
