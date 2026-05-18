""" 
AbstractCamera - An interface that every camera must implement. 
Includes shared logic, so the same function can be called no matter what camera 
is being used. 
"""

import abc 
import threading
import queue 
from typing import List 

import cv2 
import numpy as np 

from utils import (
    Target, 
    Coordinate, 
    Colours, 
)

class AbstractCamera(abc.ABC): 
    """Abstract Camera class that all cameras must inherit from."""

    def __init__(self):
        pass

    def start(self) -> bool: 
        """Initialize the Camera, returns True on Success"""
        pass

    def stop(self) -> None: 
        """Stop the Camera and release any resources."""
        pass

    def latest_frame(self) -> CameraFrame | None: 
        """Returns the latest frame from the camera, or None if no frame is available."""
        pass

    def run(self) -> None: 
        """Main loop for the camera, should be run in a separate thread."""
        pass

    @abc.abstractmethod
    def _initialize_camera(self) -> bool: 
        """Camera specific initialization logic, returns True on Success"""
        pass

    @abc.abstractmethod
    def _capture_frame(self) -> CameraFrame | None: 
        """Camera specific frame capture logic, returns a CameraFrame or None if capture failed."""
        pass

