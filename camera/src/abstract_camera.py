""" 
AbstractCamera - An interface that every camera must implement. 
Includes shared logic, so the same function can be called no matter what camera 
is being used. 
"""

import abc

from .frame import CameraFrame

class AbstractCamera(abc.ABC): 
    """Abstract Camera class that all cameras must inherit from.

    Attributes:
        res_x: The x resolution of the camera
        res_y: The y resolution of the camera
    """

    def __init__(self, res_x: int, res_y: int):
        """Basic initializer for all cameras

        Args:
            res_x: Defines camera width (x resolution)
            res_y: Defines camera height (y resolution)
        """
        self.res_x = res_x
        self.res_y = res_y
        pass

    @abc.abstractmethod
    def start(self) -> bool: 
        """Initializes the camera, returns True on Success.

        This is an abstract method, must be implemented by child classes

        Returns:
            True if the initialization is success 
            False is there is a failure
        """
        return False
    
    @abc.abstractmethod
    def stop(self) -> bool: 
        """Stop the Camera and release any resources.

        This is an abstract method, must be implemented by child classes

        Returns:
            True if the initialization is success 
            False is there is a failure
        """
        pass

    @abc.abstractmethod
    def capture_frame(self) -> CameraFrame | None: 
        """Camera specific frame capture logic, returns a CameraFrame or None if capture failed.

        This is an abstract method, must be implemented by child classes

        Returns:
            CameraFrame if it successfully captures a frame
            None if there is an error capturing a frame
        """
        pass

