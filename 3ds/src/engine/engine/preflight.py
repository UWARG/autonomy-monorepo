"""Preflight decisions independent of ROS, using monotonic receive times."""

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

OK, WARN, ERROR, STALE = range(4)
PREARM_CHECK = 1 << 28


@dataclass(frozen=True)
class Check:
    level: int
    message: str


class Stream:
    """Bounded receive history for freshness and minimum-rate checks."""

    def __init__(self, timeout: float, min_hz: float = 0.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        if not math.isfinite(min_hz) or min_hz < 0:
            raise ValueError("min_hz must be finite and nonnegative")
        self.timeout = timeout
        self.min_hz = min_hz
        self.times = deque(maxlen=1000)

    def tick(self, now: float):
        self.times.append(now)

    def check(self, now: float) -> Check:
        if not self.times:
            return Check(STALE, "No messages received")
        age = now - self.times[-1]
        if age < 0 or age > self.timeout:
            return Check(STALE, f"Last message age {age:.1f}s; limit {self.timeout}s")
        if self.min_hz:
            recent = [t for t in self.times if now - t <= 5.0]
            if len(recent) < 2:
                return Check(WARN, "Waiting for rate samples")
            elapsed = now - recent[0]
            rate = (len(recent) - 1) / elapsed if elapsed > 0 else 0.0
            if rate < self.min_hz:
                return Check(WARN, f"Rate {rate:.1f}Hz below {self.min_hz}Hz")
        return Check(OK, "Fresh messages received")


def prearm_check(present: int, enabled: int, health: int, armed: bool) -> Check:
    """Use positive SYS_STATUS evidence, never absence of STATUSTEXT errors."""
    if armed:
        return Check(ERROR, "Already armed; preflight readiness applies while disarmed")
    if not (present & enabled & PREARM_CHECK):
        return Check(STALE, "FCU pre-arm check unavailable or disabled")
    if not health & PREARM_CHECK:
        return Check(
            ERROR, "ArduPilot pre-arm checks failing; inspect MAVROS statustext"
        )
    return Check(OK, "ArduPilot reports pre-arm checks passed")


def image_check(data, width, height, step, encoding, min_mean, min_std) -> Check:
    """Inspect supported 8-bit pixels, excluding padding and alpha channels."""
    channels = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    if encoding not in channels:
        return Check(ERROR, f"Unsupported image encoding: {encoding}")
    count = channels[encoding]
    if width <= 0 or height <= 0 or step < width * count or len(data) < height * step:
        return Check(ERROR, "Empty or malformed image")
    rows = np.frombuffer(data, dtype=np.uint8, count=height * step).reshape(
        height, step
    )
    pixels = rows[:, : width * count].reshape(height, width, count)
    # Sample at most roughly 64x64 pixels to keep the camera callback bounded.
    pixels = pixels[:: max(1, height // 64), :: max(1, width // 64), : min(count, 3)]
    # Spatial variation, not variation between the RGB channels of a solid image.
    mean = float(pixels.mean())
    std = float(pixels.astype(float).std(axis=(0, 1)).mean())
    if mean < min_mean:
        return Check(ERROR, f"Image too dark (mean={mean:.1f})")
    if std < min_std:
        return Check(ERROR, f"Image appears blank (spatial std={std:.1f})")
    return Check(OK, f"Image usable (mean={mean:.1f}, spatial std={std:.1f})")


def overall(checks: dict[str, Check]) -> Check:
    failures = [
        f"{name}: {check.message}"
        for name, check in checks.items()
        if check.level != OK
    ]
    if failures:
        return Check(max(check.level for check in checks.values()), "; ".join(failures))
    if not checks:
        return Check(STALE, "No checks configured")
    return Check(OK, "Ready for takeoff")
