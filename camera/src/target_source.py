import abc
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Optional


@dataclass
class TargetObservation:
    """One detection of the tracked person"""

    x_mm: float  # right
    y_mm: float  # down
    z_mm: float  # forward / depth (> 0, 0 means invalid)
    tracked: bool = True
    timestamp: float = field(default_factory=time.time)


class AbstractTargetSource(abc.ABC):

    def start(self) -> bool:
        return self.initialize()

    def stop(self) -> None:
        """"""
        
    @abc.abstractmethod
    def initialize(self) -> bool:
        """"""

    @abc.abstractmethod
    def get_target(self) -> Optional[TargetObservation]:
        """Latest observation, or None on no-detection/failure (never raises)."""


class SimTargetSource(AbstractTargetSource):
    def __init__(
        self,
        clock: Callable[[], float] = time.time,
        z_centre_mm: float = 3000.0,
        z_amplitude_mm: float = 800.0,
        x_amplitude_mm: float = 900.0,
        period_s: float = 12.0,
    ) -> None:
        self._clock = clock
        self._t0 = 0.0
        self._z_centre = z_centre_mm
        self._z_amp = z_amplitude_mm
        self._x_amp = x_amplitude_mm
        self._period = period_s

    def initialize(self) -> bool:
        self._t0 = self._clock()
        return True

    def get_target(self) -> Optional[TargetObservation]:
        t = self._clock() - self._t0
        w = 2.0 * math.pi / self._period
        z = self._z_centre + self._z_amp * math.sin(w * t)
        x = self._x_amp * math.sin(0.5 * w * t)
        y = 0.0  
        return TargetObservation(x_mm=x, y_mm=y, z_mm=z, tracked=True, timestamp=self._clock())

# Piecewise-linear Z bias correction: (raw_z_mm, offset_to_subtract_mm).
_Z_CAL_ANCHORS = [
    (527.0, 27.5),
    (1075.0, 75.1),
    (1573.0, 73.2),
    (1951.0, -48.7),
    (2200.0, 0.0),
]

def calibrate_z(raw_z_mm: float) -> float:
    """Apply the measured Z bias correction, trust the factory value beyond 2200 mm"""
    if raw_z_mm <= _Z_CAL_ANCHORS[0][0]:
        return raw_z_mm - _Z_CAL_ANCHORS[0][1]
    if raw_z_mm >= _Z_CAL_ANCHORS[-1][0]:
        return raw_z_mm  # beyond the validated range, trust the device
    for (z0, off0), (z1, off1) in zip(_Z_CAL_ANCHORS, _Z_CAL_ANCHORS[1:]):
        if z0 <= raw_z_mm <= z1:
            frac = (raw_z_mm - z0) / (z1 - z0)
            offset = off0 + frac * (off1 - off0)
            return raw_z_mm - offset
    return raw_z_mm


def calibrate_xy(raw_x_mm: float, raw_y_mm: float, raw_z_mm: float, cal_z_mm: float):
    """Rescale X/Y by the depth-correction ratio"""
    if raw_z_mm == 0.0:
        return raw_x_mm, raw_y_mm
    ratio = cal_z_mm / raw_z_mm
    return raw_x_mm * ratio, raw_y_mm * ratio


def select_closest_tracked(tracklets, tracked_status):
    """Pick the nearest (min spatial z) tracklet whose status == ``tracked_status``."""
    candidates = [t for t in tracklets if t.status == tracked_status]
    if not candidates:
        return None
    return min(candidates, key=lambda t: t.spatialCoordinates.z)


class RealTargetSource(AbstractTargetSource):
    def __init__(self, poll_fn=None, tracked_status="TRACKED", clock: Callable[[], float] = time.time):
        self._poll_fn = poll_fn
        self._tracked_status = tracked_status
        self._clock = clock

    def initialize(self) -> bool:
        return self._poll_fn is not None

    def get_target(self) -> Optional[TargetObservation]:
        if self._poll_fn is None:
            return None
        closest = select_closest_tracked(self._poll_fn(), self._tracked_status)
        if closest is None:
            return None
        sc = closest.spatialCoordinates
        if sc.z <= 0.0:  # invalid depth sentinel
            return None
        cal_z = calibrate_z(sc.z)
        cal_x, cal_y = calibrate_xy(sc.x, sc.y, sc.z, cal_z)
        return TargetObservation(x_mm=cal_x, y_mm=cal_y, z_mm=cal_z, tracked=True, timestamp=self._clock())
