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

    def __init__(
        self,
        frame_interval_s: float = 0.0,
        startup_retries: int = 1,
    ) -> None:
        if frame_interval_s < 0:
            raise ValueError("frame_interval_s must be >= 0")
        if startup_retries < 0:
            raise ValueError("startup_retries must be >= 0")

        self._frame_interval_s = frame_interval_s
        self._startup_retries = startup_retries
        self._running = False
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._last_frame: CameraFrame | None = None
        self._thread: threading.Thread | None = None

    def _initialize_with_retries(self) -> bool:
        attempts = self._startup_retries + 1
        for attempt in range(attempts):
            if self.initialize_camera():
                return True

            self.close_camera()
            if attempt < attempts - 1:
                time.sleep(0.1)

        return False

    def start(self) -> bool:
        """Initializes the camera, returns True on Success."""
        if self._running:
            return True

        self._stop_event.clear()
        if self._initialize_with_retries():
            self._running = True
            self._thread = threading.Thread(
                target=self.run,
                name=f"{type(self).__name__}Thread",
                daemon=True,
            )
            self._thread.start()
            return True

        self._running = False
        self._thread = None
        self._stop_event.set()
        return False

    def stop(self) -> None:
        """Stop the Camera and release any resources."""
        if not self._running:
            return

        self._running = False
        self._stop_event.set()
        if (
            self._thread is not None
            and self._thread.is_alive()
            and threading.current_thread() is not self._thread
        ):
            self._thread.join()
        self._thread = None
        self.close_camera()

    def run(self) -> None:
        """Main loop for the camera, should be run in a separate thread."""
        if not self._running:
            raise RuntimeError("Camera must be started before run().")

        consecutive_capture_failures = 0
        while self._running and not self._stop_event.is_set():
            frame = self.capture_frame()
            if frame is None:
                consecutive_capture_failures += 1
                if consecutive_capture_failures > self._startup_retries:
                    if not self._initialize_with_retries():
                        self.stop()
                        break
                    consecutive_capture_failures = 0
            else:
                consecutive_capture_failures = 0
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
        """Returns a CameraFrame or None if capture failed."""
        pass

    @abc.abstractmethod
    def close_camera(self) -> None:
        """Camera specific shutdown logic."""
        pass
