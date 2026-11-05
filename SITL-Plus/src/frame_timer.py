"""Wall-clock scheduling for sensor and visualization frames."""

import time


class FrameTimer:
    """Schedule frames without adding processing time to the frame interval."""

    def __init__(self, fps):
        if fps <= 0:
            raise ValueError("Frame rate must be positive")
        self.interval = 1 / fps
        self.deadline = time.monotonic()

    def ready(self):
        """Return whether a frame is due, skipping missed frame slots."""
        now = time.monotonic()
        if now < self.deadline:
            return False
        elapsed_slots = int((now - self.deadline) / self.interval) + 1
        self.deadline += elapsed_slots * self.interval
        return True

    def wait(self):
        """Wait only for the unused portion of the frame interval."""
        delay = self.deadline - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self.ready()
