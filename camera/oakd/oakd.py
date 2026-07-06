from ..src.abstract_camera import AbstractCamera
import depthai as dai

class Oakd(AbstractCamera):
    """The oakd camera class implementation

    Camera Model: Luxinos OakD Pro
    """

    def __init__(self, 
                 res_x: int, 
                 res_y: int, 
                 *, # Enforces use of param=
                 depth_enabled: bool = False,
                 slam_enabled: bool = False,
                 inference_enabled: bool = False ):
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
        self.depth_enabled = depth_enabled
        self.slam_enabled = slam_enabled
        self.inference_enabled = inference_enabled

    def start(self) -> bool:
        """Oakd implementation for the start abstract method

        Returns:
            See base class
        """
        return True

    def stop(self) -> bool: 
        """Oakd implementation for the stop abstract method

        Returns:
            See base class
        """
        pass

    def capture_frame(self) -> CameraFrame | None: 
        """Oakd implementation for the capture_frame abstract method

        Returns:
            See base class
        """
        pass

