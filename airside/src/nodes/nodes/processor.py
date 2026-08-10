import faulthandler
import math
import os
import sys

from numpy.ma import true_divide

# Import CUDA OpenCV *before* cv_bridge. If cv_bridge loads first it can bind the
# wrong OpenCV, and the first initUndistortRectifyMap then SIGSEGVs on Jetson.
import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sortedcontainers import SortedDict

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, Imu, NavSatFix, Range
from std_msgs.msg import Float64

from custom_interfaces.action import Landing, Takeoff
from custom_interfaces.msg import Error
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import SetMode

CAMERA_OFFSET=0.3

# A teach frame with too few features is worse than no frame at all: the repeat
# pass selects it by altitude, fails to match, and stalls. Frames captured low
# and fast during the climb are the ones that fall below this.
MIN_TEACH_KEYPOINTS=150

# If vision has been unavailable for this long, holding position cannot recover
# a fix. Either commit to the descent (last fix was inside the 1x full-descent
# core) or climb to widen the footprint and re-acquire (it was not).
VISION_TIMEOUT_S=2.0
REACQUIRE_VZ=0.1

# Descent authority tapers with alignment quality inside an altitude-proportional
# cone, instead of being hard-gated to zero by a fixed tolerance.
DESCENT_VZ=0.1
ALIGN_TOLERANCE_RATIO=0.15
MIN_ALIGN_TOLERANCE_M=0.05

# Once vision is exhausted we hand the aircraft to the autopilot's LAND mode
# rather than continuing to stream downward velocity setpoints in GUIDED: it
# owns touchdown detection and disarm, and it stops us driving the motors into
# the ground after the gear is loaded.
LAND_MODE="LAND"
ON_GROUND_SAMPLES=3


class Processor(Node):
    def __init__(self) -> None:
        super().__init__("processor")
        self._cb_group=ReentrantCallbackGroup()
        self._mutual_cb_group=MutuallyExclusiveCallbackGroup()
        self.image_subscriber = self.create_subscription(Image, "camera/image", self.image_callback, 1, callback_group=self._cb_group)
        self.imu_subscriber = self.create_subscription(Imu, "/mavros/imu/data", self.imu_callback, qos_profile_sensor_data, callback_group=self._cb_group)
        self.takeoff_server=ActionServer(self, Takeoff, "takeoff", self.takeoff_callback, cancel_callback=self.takeoff_cancel_callback, callback_group=self._cb_group)
        self.landing_server=ActionServer(self, Landing, "landing", self.landing_callback, cancel_callback=self.landing_cancel_callback, callback_group=self._cb_group)

        self._gps_sub = self.create_subscription(NavSatFix,"/mavros/global_position/global",self.fix_callback,qos_profile_sensor_data,callback_group=self._cb_group)
        self._rel_alt_sub = self.create_subscription(Float64,"/mavros/global_position/rel_alt",self.rel_alt_callback,qos_profile_sensor_data,callback_group=self._cb_group)
        self._range_sub = self.create_subscription(Range,"/mavros/rangefinder_lidar",self.range_callback,qos_profile_sensor_data,callback_group=self._cb_group)
        self.error_publisher=self.create_publisher(Error, "error", 10, callback_group=self._cb_group)

        self.create_timer(0.1, self.process, callback_group=self._mutual_cb_group)

        self.imu_dict=SortedDict()
        self.image=None
        self.latitude=0.0
        self.longitude=0.0
        self.rel_alt=None
        self.last_altitude=0
        self.altitude_threshold=0.1
        self.last_image_altitude=7.5
        self.range=None
        # CPU OpenCV setup before any CUDA context — mixing the other way
        # SIGSEGV'd on this Jetson OpenCV build after createBFMatcher.
        self.get_logger().info("init: ORB_create")
        self.orb=cv2.ORB_create(nfeatures=1000)
        self.get_logger().info("init: CvBridge")
        self._bridge=CvBridge()
        self.get_logger().info("init: camera_intrinsics")
        self.camera_intrinsics()
        self._use_cuda = False
        self.BFMatcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.get_logger().info("init: CUDA BFMatcher")
        if hasattr(cv2, "cuda") and cv2.cuda.getCudaEnabledDeviceCount() > 0:
            self.BFMatcher = cv2.cuda.DescriptorMatcher_createBFMatcher(cv2.NORM_HAMMING)
            self._use_cuda = True
            self.get_logger().info("Using CUDA BFMatcher")
        else:
            self.get_logger().warn("CUDA unavailable, using CPU BFMatcher")
        self.get_logger().info("Processor initialized")
        self.roll=None
        self.pitch=None
        self.yaw=None
        self.takeoff_goal_handle=None
        self.landing_goal_handle=None
        self.landing_complete=False
        self.landing_failed=False
        self.last_valid_time=None
        self.last_valid_xy=None
        self.landed_state=None
        self.land_handoff_done=False
        self.on_ground_count=0
        self._agl=0.0
        self._extended_state_sub=self.create_subscription(
            ExtendedState,"/mavros/extended_state",self.extended_state_callback,
            qos_profile_sensor_data,callback_group=self._cb_group,
        )
        self._set_mode_client=self.create_client(
            SetMode,"/mavros/set_mode",callback_group=self._cb_group,
        )
        self.image_rate=0.1 #meters/image
        self.error_margin=0.02 #meters
        self.landing_3d_points=[]
        self.takeoff_3d_points=[]
        self.last_landing_altitude=0.4 #alt to go straight down
        self.min_inlier_ratio=0.4
        self.lowe_ratio=0.55

    def range_callback(self, msg: Range):
        if not math.isfinite(msg.range) or msg.range <= 0.0:
            self.range = None
            return
        self.range = msg.range
        if self.range < 1.0:
            self.image_rate = 0.1
            self.error_margin = 0.02
        else:
            self.image_rate = 0.25
            self.error_margin = 0.05

    def extended_state_callback(self, msg: ExtendedState):
        self.landed_state=msg.landed_state

    def _handoff_to_land_mode(self):
        """Stop guiding and let the autopilot's LAND mode finish the touchdown."""
        if self.land_handoff_done:
            return
        if not self._set_mode_client.service_is_ready():
            # Do not latch the handoff: retry next tick rather than waiting
            # forever for a ground report from a mode we never entered.
            self.get_logger().error(
                "set_mode service unavailable; cannot hand off to LAND", throttle_duration_sec=2.0
            )
            return
        request=SetMode.Request()
        request.custom_mode=LAND_MODE
        self._set_mode_client.call_async(request)
        self.land_handoff_done=True
        self.on_ground_count=0
        # Tell the controller to stop commanding so we are not fighting LAND mode.
        self.error_publisher.publish(Error(
            x=0.0,y=0.0,angle=0.0,yaw_error=0.0,vz=0.0,
            valid_error=False,landing_complete=True,
        ))
        self.get_logger().info(f"Vision exhausted: handing off to {LAND_MODE} mode")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds/1e9

    def _handle_stale_vision(self, yaw_error: float=0.0) -> bool:
        """Break the deadlock once vision has been unavailable for a while.

        Holding position cannot recover a fix, because the view never changes.
        If we were aligned when we last saw the ground, finish on the rangefinder;
        if we were not, climb to widen the footprint and re-acquire. Returns True
        if a command was published.
        """
        if self.last_valid_time is None or self.last_valid_xy is None:
            return False
        if self._now()-self.last_valid_time<VISION_TIMEOUT_S:
            return False
        # Commit only if last fix was inside the full-descent core (1x cone).
        # Live vision still tapers out to 2x; blind commit is irreversible.
        tolerance=max(MIN_ALIGN_TOLERANCE_M,ALIGN_TOLERANCE_RATIO*self._agl)
        if self.last_valid_xy<=tolerance:
            self.error_publisher.publish(Error(
                x=0.0,y=0.0,angle=0.0,yaw_error=yaw_error,vz=DESCENT_VZ,
                valid_error=False,landing_complete=False,
            ))
            return True
        if self.imu_dict and self._agl>=self.imu_dict.peekitem(-1)[0]:
            # Already at the top of the teach map: climbing further cannot help.
            self.get_logger().error(
                f"Vision stale at {self._agl:.2f} m, {self.last_valid_xy:.2f} m off "
                "target, and above the top of the teach map",throttle_duration_sec=2.0,
            )
            return False
        self.get_logger().warn(
            f"Vision stale, {self.last_valid_xy:.2f} m off target "
            f"(> {tolerance:.2f} m): climbing to re-acquire",throttle_duration_sec=2.0,
        )
        self.error_publisher.publish(Error(
            x=0.0,y=0.0,angle=0.0,yaw_error=yaw_error,vz=-REACQUIRE_VZ,
            valid_error=False,landing_complete=False,
        ))
        return True

    def publish_invalid_error(self, yaw_error: float=0.0):
        if self._handle_stale_vision(yaw_error):
            return
        self.error_publisher.publish(Error(
            x=0.0,y=0.0,angle=0.0,yaw_error=yaw_error,vz=0.0,
            valid_error=False,landing_complete=False,
        ))

    @staticmethod
    def wrap_pi(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def fix_callback(self, msg: NavSatFix):
        self.latitude=msg.latitude
        self.longitude=msg.longitude

    def rel_alt_callback(self, msg: Float64):
        self.rel_alt=msg.data

    def fail_landing(self, reason: str):
        self.get_logger().error(reason)
        self.error_publisher.publish(Error(
            x=0.0,y=0.0,angle=0.0,vz=0.0,valid_error=False,
            landing_complete=True,
        ))
        self.landing_failed=True

    def landing_cancel_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Landing cancelled")
        return CancelResponse.ACCEPT

    def landing_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Landing requested")
        self.landing_goal_handle=goal_handle
        self.landing_complete=False
        self.landing_failed=False
        self.last_valid_time=None
        self.last_valid_xy=None
        self.land_handoff_done=False
        self.on_ground_count=0
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
    def takeoff_cancel_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Takeoff cancelled")
        return CancelResponse.ACCEPT

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
        if self.range is None or self.pitch is None or self.roll is None or self.yaw is None:
            return
        if self.last_altitude>self.last_image_altitude and self.takeoff_goal_handle:
            return
        agl=self.range-CAMERA_OFFSET
        self._agl=agl
        if self.takeoff_goal_handle:
            #snapshot
            image=self.image
            roll=self.roll
            pitch=self.pitch
            yaw=self.yaw
            if agl-self.last_altitude>=self.image_rate-self.error_margin:
                gray=self.undistort_image(image)
                kp,des=self.generate_orb_descriptors(gray)
                if kp is None or des is None:
                    return
                if len(kp)<MIN_TEACH_KEYPOINTS:
                    # Do not advance last_altitude: retry on the next tick so the
                    # map simply starts higher rather than storing a dead frame.
                    self.get_logger().warn(
                        f"Teach frame at {agl:.2f} m rejected: {len(kp)} keypoints "
                        f"(< {MIN_TEACH_KEYPOINTS})"
                    )
                    return
                self.imu_dict[agl]=[kp,des,roll,pitch,yaw]
                self.last_altitude=agl
                cv2.imwrite(os.path.join("/images", f"takeoff_{agl:.2f}.png"), gray)
                self.get_logger().info(f"Takeoff image saved to /images/takeoff_{agl:.2f}.png")
        elif self.landing_goal_handle:
            #snapshot
            image=self.image
            land_roll=self.roll
            land_pitch=self.pitch
            land_yaw=self.yaw
            if self.land_handoff_done:
                # LAND mode owns the aircraft: just wait for the FCU to confirm
                # ground contact, debounced against a single spurious sample.
                if self.landed_state==ExtendedState.LANDED_STATE_ON_GROUND:
                    self.on_ground_count+=1
                else:
                    self.on_ground_count=0
                if self.on_ground_count>=ON_GROUND_SAMPLES:
                    self.get_logger().info("FCU reports on ground: landing complete")
                    self.landing_complete=True
                return
            if not self.imu_dict:
                self.fail_landing("Empty map")
                return
            if agl<=self.last_landing_altitude:
                self._handoff_to_land_mode()
                return
            index=self.imu_dict.bisect_right(agl)-1
            if index==0:
                self._handoff_to_land_mode()
                return
            if index<0:
                self.get_logger().error("No takeoff key found")
                self.publish_invalid_error()
                return
            key,entry=self.imu_dict.peekitem(index)
            kp_takeoff,des_takeoff,takeoff_roll,takeoff_pitch,takeoff_yaw=entry
            if takeoff_yaw is None:
                self.publish_invalid_error()
                return
            yaw_error=self.wrap_pi(takeoff_yaw-land_yaw)
            gray=self.undistort_image(image)
            kp,des=self.generate_orb_descriptors(gray)
            if not os.path.exists(os.path.join("/images", f"landing_{key:.2f}.png")):
                cv2.imwrite(os.path.join("/images", f"landing_{key:.2f}.png"), gray)
                self.get_logger().info(f"Landing image saved to /images/landing_{key:.2f}.png")
            if kp is None or des is None:
                self.publish_invalid_error(yaw_error)
                return
            if kp_takeoff is None or des_takeoff is None or takeoff_roll is None or takeoff_pitch is None:
                self.publish_invalid_error(yaw_error)
                return
            if self._use_cuda:
                gpu_landing_des=cv2.cuda.GpuMat()
                gpu_takeoff_des=cv2.cuda.GpuMat()
                gpu_landing_des.upload(des)
                gpu_takeoff_des.upload(des_takeoff)
                knn_matches=self.BFMatcher.knnMatch(gpu_landing_des, gpu_takeoff_des, 2)
            else:
                knn_matches=self.BFMatcher.knnMatch(des, des_takeoff, 2)
            # Lowe ratio test. Ground texture (asphalt, grass, concrete aggregate)
            # produces many near-identical descriptors, so a small Hamming distance
            # alone does not mean the correspondence is right. Keep a match only when
            # its best candidate is clearly better than its runner-up.
            matches=[
                pair[0] for pair in knn_matches
                if len(pair)==2 and pair[0].distance<self.lowe_ratio*pair[1].distance
            ]
            if len(matches)<10:
                self.get_logger().error(f"Not enough matches: {len(matches)}")
                self.publish_invalid_error(yaw_error)
                return
            self.takeoff_3d_points=[]
            self.landing_3d_points=[]
            for match in matches:
                x_land_px,y_land_px=kp[match.queryIdx]
                x_takeoff_px,y_takeoff_px=kp_takeoff[match.trainIdx]
                if x_land_px is None or y_land_px is None or x_takeoff_px is None or y_takeoff_px is None:
                    continue
                x_land_3d,y_land_3d=self.pixel_to_3d(x_land_px,y_land_px,land_roll,land_pitch,agl)
                x_takeoff_3d,y_takeoff_3d=self.pixel_to_3d(x_takeoff_px,y_takeoff_px,takeoff_roll,takeoff_pitch,key)
                self.takeoff_3d_points.append([x_takeoff_3d,y_takeoff_3d])
                self.landing_3d_points.append([x_land_3d,y_land_3d])
            if len(self.takeoff_3d_points)<10:
                self.get_logger().error(f"Not enough takeoff points: {len(self.takeoff_3d_points)}")
                self.publish_invalid_error(yaw_error)
                return
            #implement RANSAC 
            H,inliers=cv2.estimateAffinePartial2D( #vector points from takeoff to landing so the translation correction should be negative in the x and y direction
                np.asarray(self.takeoff_3d_points,dtype=np.float32),
                np.asarray(self.landing_3d_points,dtype=np.float32),
                method=cv2.RANSAC,
                ransacReprojThreshold=0.1,
                maxIters=1000,
                confidence=0.99,
                refineIters=10,
                )
            if H is None or inliers is None:
                self.publish_invalid_error(yaw_error)
                self.get_logger().error("RANSAC failed")
                return
            inlier_ratio=float(np.count_nonzero(inliers))/len(inliers)
            if inlier_ratio<self.min_inlier_ratio:
                self.publish_invalid_error(yaw_error)
                self.get_logger().error(f"Inlier ratio too low: {inlier_ratio:.2f}")
                return
            translation_x=H[0,2]
            translation_y=H[1,2]
            rotation_angle=math.atan2(H[1,0], H[0,0])
            scale=math.sqrt(H[0,0]**2 + H[1,0]**2)
            if scale<=1.0-self.error_margin or scale>=1.0+self.error_margin:
                if self._handle_stale_vision(yaw_error):
                    return
                # Nudge altitude toward the teach key so scale can converge to ~1.
                if key<agl:  # above teach key → descend
                    scale_vz=0.1
                elif key>agl:  # below teach key → ascend
                    scale_vz=-0.1
                else:
                    scale_vz=0.0
                self.error_publisher.publish(Error(
                    x=0.0,y=0.0,angle=0.0,yaw_error=yaw_error,vz=scale_vz,
                    valid_error=False,landing_complete=False,
                ))
                self.get_logger().error(
                    f"Scale out of margin: {scale:.2f} (key={key:.2f}, agl={agl:.2f}, vz={scale_vz})"
                )
                return
            self.get_logger().info(f"Yaw delta (imu): {math.degrees(yaw_error):.1f} deg, measured rotation: {math.degrees(rotation_angle):.1f} deg")
            # Altitude-proportional alignment cone: require tight centring only as
            # the ground approaches, and taper descent authority with alignment
            # quality instead of stopping dead outside a fixed tolerance.
            xy_error=math.hypot(translation_x,translation_y)
            align_tolerance=max(MIN_ALIGN_TOLERANCE_M,ALIGN_TOLERANCE_RATIO*agl)
            taper=max(0.0,min(1.0,2.0-xy_error/align_tolerance))
            self.last_valid_time=self._now()
            self.last_valid_xy=xy_error
            error=Error()
            error.x=translation_x
            error.y=translation_y
            error.angle=rotation_angle
            error.yaw_error=yaw_error
            error.vz=DESCENT_VZ*taper
            error.valid_error=True
            error.landing_complete=False
            self.error_publisher.publish(error)

    def generate_orb_descriptors(self, image: Image):
        kp,des=self.orb.detectAndCompute(image, None)
        if not kp:
            return None, None
        kp_pts=np.array([pt.pt for pt in kp])
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
            self.yaw=None
            return
        quaternion=[q.x, q.y, q.z, q.w]
        euler=R.from_quat(quaternion).as_euler("xyz", degrees=False)
        self.roll=euler[0]
        self.pitch=euler[1]
        self.yaw=euler[2]
    
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
        reshaped_k=np.asarray(self.k, dtype=np.float64).reshape(3, 3)
        reshaped_d=np.asarray(self.d, dtype=np.float64).reshape(-1)
        size=(int(self.width), int(self.height))
        self.new_camera_matrix,self.roi=cv2.getOptimalNewCameraMatrix(reshaped_k, reshaped_d, size, 0)
        self.mapx,self.mapy=cv2.initUndistortRectifyMap(
            reshaped_k, reshaped_d, None, self.new_camera_matrix, size, cv2.CV_32FC1
        )
        x,y,w,h=self.roi
        self.fx=self.new_camera_matrix[0,0]
        self.fy=self.new_camera_matrix[1,1]
        self.cx=self.new_camera_matrix[0,2]-x
        self.cy=self.new_camera_matrix[1,2]-y

def main(args=None):
    faulthandler.enable(file=sys.stderr, all_threads=True)
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
