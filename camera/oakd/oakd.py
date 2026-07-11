from src.abstract_camera import AbstractCamera
from rerun_node import RerunNode
import depthai as dai
import time
import numpy as np
from utils import quat_rotate

from src.frame import CameraFrame

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
        self.rerun_enabled = rerun_enabled
        self.pipeline = dai.Pipeline()

    def start(self) -> bool:
        """Oakd implementation for the start abstract method

        Returns:
            See base class
        """
        self.cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        self.video_queue = self.cam.requestOutput((self.res_x, self.res_y)).createOutputQueue(maxSize=1, blocking=False)

        if self.slam_enabled:
            self.left_cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=self.fps)
            self.right_cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=self.fps)
            self.cam_imu = self.pipeline.create(dai.node.IMU)
            self.visual_odom = self.pipeline.create(dai.node.BasaltVIO)
            self.slam = self.pipeline.create(dai.node.RTABMapSLAM)
            self.depth = self.pipeline.create(dai.node.StereoDepth, presetMode=dai.node.StereoDepth.PresetMode.FAST_DENSITY)
            self.slam.setDatabasePath(str("./building.db"))
            self.params = {
                        "RGBD/CreateOccupancyGrid": "true",
                        "Grid/3D": "true",
                        "Rtabmap/SaveWMState": "true",
                        "RGBD/ProximityBySpace": "false",
                    }
            self.slam.setParams(self.params)

            if self.rerun_enabled:
                self.rerun_viewer = self.pipeline.create(RerunNode)

            self.cam_imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
            self.cam_imu.setBatchReportThreshold(1)
            self.cam_imu.setMaxBatchReports(10)

            self.depth.setExtendedDisparity(False)
            self.depth.setLeftRightCheck(True)
            self.depth.setSubpixel(True)
            self.depth.setRectifyEdgeFillColor(0)
            self.depth.enableDistortionCorrection(True)
            self.depth.initialConfig.setLeftRightCheckThreshold(10)
            self.depth.setDepthAlign(dai.CameraBoardSocket.CAM_B)
            
            self.left_cam.requestOutput((self.res_x, self.res_y)).link(self.depth.left)
            self.right_cam.requestOutput((self.res_x, self.res_y)).link(self.depth.right)
            self.depth.syncedLeft.link(self.visual_odom.left)
            self.depth.syncedRight.link(self.visual_odom.right)
            self.depth.depth.link(self.slam.depth)
            self.depth.rectifiedLeft.link(self.slam.rect)
            self.cam_imu.out.link(self.visual_odom.imu)

            self.visual_odom.transform.link(self.slam.odom)

            self.required_cam_capabilties = dai.ImgFrameCapability()
            self.required_cam_capabilties.fps.fixed(self.fps)
            self.required_cam_capabilties.enableUndistortion = True

            # TODO: replace model
            self.det_nn = self.pipeline.create(dai.node.DetectionNetwork).build(self.cam, "yolov6-nano", self.required_cam_capabilties)

            self.spatial_calc = self.pipeline.create(dai.node.SpatialLocationCalculator)
            self.spatial_calc.initialConfig.setCalculateSpatialKeypoints(True)
            self.det_nn.out.link(self.spatial_calc.inputDetections)

            self.det_nn.passthrough.link(self.depth.inputAlignTo)
            self.depth.depth.link(self.spatial_calc.inputDepth)

            self.spatial_output_queue = self.spatial_calc.outputDetections.createOutputQueue(maxSize=1, blocking=False)
            self.slam_transforms = self.slam.transform.createOutputQueue(maxSize=1, blocking=False)

            if self.rerun_enabled:
                self.slam.transform.link(self.rerun_viewer.inputTrans)
                self.slam.passthroughRect.link(self.rerun_viewer.inputImg)
                self.slam.occupancyGridMap.link(self.rerun_viewer.inputGrid)
                self.slam.obstaclePCL.link(self.rerun_viewer.inputObstaclePCL)
                self.slam.groundPCL.link(self.rerun_viewer.inputGroundPCL)

        self.pipeline.start()
        while self.pipeline.isRunning() and self.slam_enabled:
            spatialData = self.spatial_output_queue.get()
            transData = self.slam_transforms.get()

            assert isinstance(spatialData, dai.SpatialImgDetections)

            cached_t_world = None
            q = None

            if isinstance(transData, dai.TransformData):
                t = transData.getTranslation()
                q = transData.getQuaternion()
                cached_t_world = np.array([t.x, t.y, t.z])

            for (i, det) in enumerate(spatialData.detections):
                if det.label != 39:
                    continue
                depthCoordinate = det.spatialCoordinates
                text = f"X: {depthCoordinate.x / 1000} m, Y: {depthCoordinate.y / 1000} m, Z: {depthCoordinate.z / 1000} m"
                print(f"====={det.label}=====")
                print(text)

                if cached_t_world is not None and q is not None:
                    cam_flu = np.array([depthCoordinate.z / 1000, -depthCoordinate.x / 1000.0, -depthCoordinate.y / 1000.0])
                    world_pos = quat_rotate(q.qx, q.qy, q.qz, q.qw, cam_flu) + cached_t_world
                    print(f"X: {world_pos[0]}, Y: {world_pos[1]}, Z: {world_pos[2]}")

            time.sleep(0.1)

        return True

    def stop(self) -> bool: 
        """Oakd implementation for the stop abstract method

        Returns:
            See base class
        """
        self.pipeline.stop()
        del self.pipeline

        return True

    def capture_frame(self) -> CameraFrame | None: 
        """Oakd implementation for the capture_frame abstract method

        Returns:
            See base class
        """
        video_in = self.video_queue.get()

        assert isinstance(video_in, dai.ImgFrame)
        camera_frame = CameraFrame(rgb=video_in.getCvFrame(), depth=None, rgb_down=None)       

        return camera_frame
