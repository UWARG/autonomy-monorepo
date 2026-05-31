"""
AbstractCamera - An interface that every camera must implement.
Includes shared logic, so the same function can be called no matter what camera
is being used.
"""

import abc
import threading
import time

from .frame import CameraFrame


class AbstractCamera(abc.ABC):
    """Abstract Camera class that all cameras must inherit from."""

    def __init__(self, frame_interval_s: float = 0.0) -> None:
        if frame_interval_s < 0:
            raise ValueError("frame_interval_s must be >= 0")

        self._frame_interval_s = frame_interval_s
        self._running = False
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._last_frame: CameraFrame | None = None

    def start(self) -> bool:
        """Initializes the camera, returns True on Success."""
        if self._running:
            return True

        if not self.initialize_camera():
            return False

        self._stop_event.clear()
        self._running = True
        return True

    def stop(self) -> None:
        """Stop the Camera and release any resources."""
        self._running = False
        self._stop_event.set()
        self.close_camera()

    def run(self) -> None:
        """Main loop for the camera, should be run in a separate thread."""
        if not self._running and not self.start():
            raise RuntimeError("Camera failed to initialize.")

        while self._running and not self._stop_event.is_set():
            frame = self.capture_frame()
            if frame is not None:
                with self._frame_lock:
                    self._last_frame = frame

            if self._frame_interval_s > 0:
                time.sleep(self._frame_interval_s)

    @abc.abstractmethod
    def initialize_camera(self) -> bool: 
        """Camera specific initialization logic, returns True on Success"""
        pass

    @abc.abstractmethod
    def capture_frame(self) -> CameraFrame | None: 
        """Camera specific frame capture logic, returns a CameraFrame or None if capture failed."""
        pass

    @abc.abstractmethod
    def close_camera(self) -> None:
        """Camera specific shutdown logic."""
        pass
