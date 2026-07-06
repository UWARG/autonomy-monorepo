from src.abstract_camera import AbstractCamera
from rerun_node import RerunNode
import depthai as dai
import time

class Oakd(AbstractCamera):
    """The oakd camera class implementation

    Camera Model: Luxinos OakD Pro
    """

    def __init__(self, 
                 res_x: int, 
                 res_y: int, 
                 fps: int,
                 *, # Enforces use of param=
                 slam_enabled: bool = False,
                 inference_enabled: bool = False,
                 rerun_enabled: bool = False):
        """Initializing the oakd camera class 
        
        This specific function has feature flagging due to the amount of features on the oakd and the resources each can consume.
        
        Args:
            res_x: see base class
            res_y: see base class
            depth_enabled: Whether depth on the oakd is enabled
            slam_enabled: Whether slam on the oakd is enabled. Note, this also enables vio which is required for slam
            inference_enabled: Whether inference on the oakd is enabled
        """
        super().__init__(res_x, res_y)
        self.fps = fps
        self.slam_enabled = slam_enabled
        self.inference_enabled = inference_enabled
        self.rerun_enabled = rerun_enabled
        self.pipeline = dai.Pipeline()

    def start(self) -> bool:
        """Oakd implementation for the start abstract method

        Returns:
            See base class
        """
        self.left_cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=self.fps)
        self.right_cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=self.fps)

        if self.slam_enabled:
            self.cam_imu = self.pipeline.create(dai.node.IMU)
            self.visual_odom = self.pipeline.create(dai.node.BasaltVIO)
            self.slam = self.pipeline.create(dai.node.RTABMapSLAM)
            self.stereo = self.pipeline.create(dai.node.StereoDepth)
            self.params = {"RGBD/CreateOccupancyGrid": "true",
                            "Grid/3D": "true",
                            "Rtabmap/SaveWMState": "true"}
            self.slam.setParams(self.params)

            if self.rerun_enabled:
                self.rerun_viewer = self.pipeline.create(RerunNode)

            self.cam_imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
            self.cam_imu.setBatchReportThreshold(1)
            self.cam_imu.setMaxBatchReports(10)

            self.stereo.setExtendedDisparity(False)
            self.stereo.setLeftRightCheck(True)
            self.stereo.setSubpixel(True)
            self.stereo.setRectifyEdgeFillColor(0)
            self.stereo.enableDistortionCorrection(True)
            self.stereo.initialConfig.setLeftRightCheckThreshold(10)
            self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
            
            self.left_cam.requestOutput((self.res_x, self.res_y)).link(self.stereo.left)
            self.right_cam.requestOutput((self.res_x, self.res_y)).link(self.stereo.right)
            self.stereo.syncedLeft.link(self.visual_odom.left)
            self.stereo.syncedRight.link(self.visual_odom.right)
            self.stereo.depth.link(self.slam.depth)
            self.stereo.rectifiedLeft.link(self.slam.rect)
            self.cam_imu.out.link(self.visual_odom.imu)

            self.visual_odom.transform.link(self.slam.odom)
            if self.rerun_enabled:
                self.slam.transform.link(self.rerun_viewer.inputTrans)
                self.slam.passthroughRect.link(self.rerun_viewer.inputImg)
                self.slam.occupancyGridMap.link(self.rerun_viewer.inputGrid)
                self.slam.obstaclePCL.link(self.rerun_viewer.inputObstaclePCL)
                self.slam.groundPCL.link(self.rerun_viewer.inputGroundPCL)

            self.pipeline.start()
            while self.pipeline.isRunning():
                time.sleep(1)

        return True

    def stop(self) -> bool: 
        """Oakd implementation for the stop abstract method

        Returns:
            See base class
        """
        pass

    def capture_frame(self) -> None: 
        """Oakd implementation for the capture_frame abstract method

        Returns:
            See base class
        """
        pass

