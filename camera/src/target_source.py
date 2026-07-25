"""Target-source abstractions and OAK-D spatial-tracklet adaptation.

Coordinates are camera-frame millimetres: +X right, +Y down, +Z forward.
The DepthAI queue adapter deliberately has no ROS dependency so ownership,
timestamp, and calibration behaviour can be tested without camera hardware.
"""

from __future__ import annotations

import abc
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Optional

MAX_VALIDATED_RANGE_MM = 3000.0


@dataclass(frozen=True)
class TargetObservation:
    """One capture-stamped observation of the explicitly selected person."""

    x_mm: float
    y_mm: float
    z_mm: float
    track_id: int = -1
    sequence_num: int = 0
    capture_time_s: float = field(default_factory=time.time)
    received_time_s: Optional[float] = None
    tracked: bool = True
    detector_capture_time_s: Optional[float] = None
    detector_sequence_num: Optional[int] = None
    detector_confirmed: bool = True
    within_validated_range: bool = True

    @property
    def timestamp(self) -> float:
        """Compatibility alias; new code should use ``capture_time_s``."""
        return self.capture_time_s


@dataclass(frozen=True)
class TrackletPacket:
    """Host-synchronised metadata accompanying a DepthAI Tracklets message."""

    tracklets: tuple[Any, ...]
    capture_time_s: float
    sequence_num: int
    received_time_s: float
    detector_capture_time_s: Optional[float] = None
    detector_sequence_num: Optional[int] = None
    detector_confirmed: bool = True


class DepthAITrackletProvider:
    """Non-blocking adapter for a DepthAI ``Tracklets`` output queue.

    ``getTimestamp()`` is used because it is host-clock synchronised by
    DepthAI. This makes capture age comparable to ROS/system time. The queue
    can be injected in tests; importing DepthAI is not required here.
    """

    def __init__(
        self,
        output_queue: Any,
        detector_queue: Any = None,
        clock: Callable[[], float] = time.time,
        host_sync_now_fn: Optional[Callable[[], Any]] = None,
    ) -> None:
        self._queue = output_queue
        self._detector_queue = detector_queue
        self._clock = clock
        self._host_sync_now_fn = host_sync_now_fn
        self._detector_metadata: dict[int, float] = {}

    @staticmethod
    def _seconds(value: Any) -> float:
        if hasattr(value, "total_seconds"):
            return float(value.total_seconds())
        return float(value)

    def _capture_time_s(self, packet: Any, received_time_s: float) -> float:
        capture_time_s = self._seconds(packet.getTimestamp())
        if self._host_sync_now_fn is not None:
            # Translate DepthAI's host-steady epoch into the system/ROS epoch
            # while retaining the packet's actual capture age.
            host_sync_now_s = self._seconds(self._host_sync_now_fn())
            capture_time_s = received_time_s - (host_sync_now_s - capture_time_s)
        return capture_time_s

    def _drain_detector_metadata(self, received_time_s: float) -> None:
        if self._detector_queue is None:
            return
        while True:
            packet = self._detector_queue.tryGet()
            if packet is None:
                break
            sequence = int(packet.getSequenceNum())
            capture_time_s = self._capture_time_s(packet, received_time_s)
            self._detector_metadata[sequence] = capture_time_s
        # Keep enough history to tolerate queue reordering without growing forever.
        if len(self._detector_metadata) > 64:
            keep_from = sorted(self._detector_metadata)[-32]
            self._detector_metadata = {
                sequence: stamp
                for sequence, stamp in self._detector_metadata.items()
                if sequence >= keep_from
            }

    def poll(self) -> Optional[TrackletPacket]:
        received_time_s = self._clock()
        self._drain_detector_metadata(received_time_s)
        packet = self._queue.tryGet()
        if packet is None:
            return None
        received_time_s = self._clock()
        self._drain_detector_metadata(received_time_s)
        capture_time_s = self._capture_time_s(packet, received_time_s)
        sequence_num = int(packet.getSequenceNum())
        if self._detector_queue is None:
            detector_confirmed = True
            detector_sequence_num = sequence_num
            detector_capture_time_s = capture_time_s
        else:
            detector_confirmed = sequence_num in self._detector_metadata
            prior_sequences = [
                sequence
                for sequence in self._detector_metadata
                if sequence <= sequence_num
            ]
            detector_sequence_num = (
                sequence_num
                if detector_confirmed
                else max(prior_sequences, default=None)
            )
            detector_capture_time_s = (
                self._detector_metadata.get(detector_sequence_num)
                if detector_sequence_num is not None
                else None
            )
        return TrackletPacket(
            tracklets=tuple(packet.tracklets),
            capture_time_s=capture_time_s,
            sequence_num=sequence_num,
            received_time_s=received_time_s,
            detector_capture_time_s=detector_capture_time_s,
            detector_sequence_num=detector_sequence_num,
            detector_confirmed=detector_confirmed,
        )


class AbstractTargetSource(abc.ABC):
    def start(self) -> bool:
        return self.initialize()

    def stop(self) -> None:
        pass

    @abc.abstractmethod
    def initialize(self) -> bool:
        pass

    @abc.abstractmethod
    def get_target(self) -> Optional[TargetObservation]:
        """Return the latest selected target, or ``None`` without raising."""


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
        self._sequence = 0
        self._z_centre = z_centre_mm
        self._z_amp = z_amplitude_mm
        self._x_amp = x_amplitude_mm
        self._period = period_s

    def initialize(self) -> bool:
        self._t0 = self._clock()
        self._sequence = 0
        return True

    def get_target(self) -> TargetObservation:
        now = self._clock()
        t = now - self._t0
        w = 2.0 * math.pi / self._period
        self._sequence += 1
        return TargetObservation(
            x_mm=self._x_amp * math.sin(0.5 * w * t),
            y_mm=0.0,
            z_mm=self._z_centre + self._z_amp * math.sin(w * t),
            track_id=1,
            sequence_num=self._sequence,
            capture_time_s=now,
        )


# Piecewise-linear Z bias correction: (raw_z_mm, offset_to_subtract_mm).
# These anchors are provisional; the guide's physical XYZ matrix is the gate
# for making any accuracy claim.
_Z_CAL_ANCHORS = [
    (527.0, 27.5),
    (1075.0, 75.1),
    (1573.0, 73.2),
    (1951.0, -48.7),
    (2200.0, 0.0),
]


def calibrate_z(raw_z_mm: float) -> float:
    """Apply the provisional measured Z bias correction."""
    if raw_z_mm <= _Z_CAL_ANCHORS[0][0]:
        return raw_z_mm - _Z_CAL_ANCHORS[0][1]
    if raw_z_mm >= _Z_CAL_ANCHORS[-1][0]:
        return raw_z_mm
    for (z0, off0), (z1, off1) in zip(_Z_CAL_ANCHORS, _Z_CAL_ANCHORS[1:]):
        if z0 <= raw_z_mm <= z1:
            fraction = (raw_z_mm - z0) / (z1 - z0)
            return raw_z_mm - (off0 + fraction * (off1 - off0))
    return raw_z_mm


def calibrate_xy(
    raw_x_mm: float, raw_y_mm: float, raw_z_mm: float, cal_z_mm: float
) -> tuple[float, float]:
    """Preserve projection geometry by applying the Z ratio to both X and Y.

    This is mathematically consistent for XYZ projected from the same depth,
    but it is not evidence that either lateral axis meets the hardware gate.
    """
    if raw_z_mm == 0.0:
        return raw_x_mm, raw_y_mm
    ratio = cal_z_mm / raw_z_mm
    return raw_x_mm * ratio, raw_y_mm * ratio


def _status_name(tracklet: Any) -> str:
    status = tracklet.status
    return str(getattr(status, "name", status)).upper()


def _track_id(tracklet: Any) -> int:
    return int(tracklet.id)


def _range_mm(tracklet: Any) -> float:
    spatial = tracklet.spatialCoordinates
    return math.sqrt(
        float(spatial.x) ** 2 + float(spatial.y) ** 2 + float(spatial.z) ** 2
    )


def _valid_tracklets(
    tracklets: Iterable[Any],
    tracked_status: str,
    max_range_mm: Optional[float] = None,
) -> list[Any]:
    expected = tracked_status.upper()
    return [
        tracklet
        for tracklet in tracklets
        if _status_name(tracklet) == expected
        and math.isfinite(float(tracklet.spatialCoordinates.z))
        and float(tracklet.spatialCoordinates.z) > 0.0
        and math.isfinite(_range_mm(tracklet))
        and (max_range_mm is None or _range_mm(tracklet) <= max_range_mm)
    ]


def select_closest_tracked(
    tracklets: Iterable[Any],
    tracked_status: str,
    max_range_mm: Optional[float] = MAX_VALIDATED_RANGE_MM,
) -> Any | None:
    """Choose the nearest valid tracklet during explicit acquisition only."""
    candidates = _valid_tracklets(tracklets, tracked_status, max_range_mm)
    return min(candidates, key=_range_mm, default=None)


def observation_from_tracklet(
    tracklet: Any, packet: TrackletPacket
) -> TargetObservation:
    """Calibrate one valid spatial tracklet while preserving capture metadata."""
    spatial = tracklet.spatialCoordinates
    cal_z = calibrate_z(float(spatial.z))
    cal_x, cal_y = calibrate_xy(
        float(spatial.x), float(spatial.y), float(spatial.z), cal_z
    )
    return TargetObservation(
        x_mm=cal_x,
        y_mm=cal_y,
        z_mm=cal_z,
        track_id=_track_id(tracklet),
        sequence_num=packet.sequence_num,
        capture_time_s=packet.capture_time_s,
        received_time_s=packet.received_time_s,
        detector_capture_time_s=(
            packet.detector_capture_time_s or packet.capture_time_s
        ),
        detector_sequence_num=(
            packet.detector_sequence_num
            if packet.detector_sequence_num is not None
            else packet.sequence_num
        ),
        detector_confirmed=packet.detector_confirmed,
        within_validated_range=(
            math.sqrt(cal_x**2 + cal_y**2 + cal_z**2) <= MAX_VALIDATED_RANGE_MM
        ),
    )


class RealTargetSource(AbstractTargetSource):
    """Sticky owner of one DepthAI spatial track ID.

    Calling :meth:`enable` acquires the nearest valid person in that packet.
    Subsequent polls ignore every other ID. A missing/LOST tracklet returns
    ``None`` while retaining ownership for same-ID reacquisition. Only
    :meth:`disable`/``reset_target`` clears ownership.
    """

    def __init__(
        self,
        poll_fn: Optional[Callable[[], Any]] = None,
        tracked_status: str = "TRACKED",
        max_validated_range_mm: float = MAX_VALIDATED_RANGE_MM,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._poll_fn = poll_fn
        self._tracked_status = tracked_status
        self._max_validated_range_mm = max_validated_range_mm
        self._clock = clock
        self._enabled = False
        self._locked_track_id: Optional[int] = None
        self._pending_packet: Optional[TrackletPacket] = None

    @property
    def locked_track_id(self) -> Optional[int]:
        return self._locked_track_id

    @property
    def enabled(self) -> bool:
        return self._enabled

    def initialize(self) -> bool:
        return self._poll_fn is not None

    def _poll(self) -> Optional[TrackletPacket]:
        if self._poll_fn is None:
            return None
        value = self._poll_fn()
        if value is None:
            return None
        if isinstance(value, TrackletPacket):
            return value
        now = self._clock()
        return TrackletPacket(tuple(value), now, 0, now)

    def enable(self, packet: Optional[TrackletPacket] = None) -> bool:
        """Acquire the nearest currently valid person; never switch afterward."""
        if self._enabled or self._locked_track_id is not None:
            return False
        packet = packet or self._poll()
        if packet is None:
            return False
        chosen = select_closest_tracked(
            packet.tracklets,
            self._tracked_status,
            self._max_validated_range_mm,
        )
        if chosen is None:
            return False
        self._locked_track_id = _track_id(chosen)
        self._enabled = True
        self._pending_packet = packet
        return True

    def disable(self) -> None:
        self._enabled = False
        self._locked_track_id = None
        self._pending_packet = None

    reset_target = disable

    def get_target(self) -> Optional[TargetObservation]:
        if not self._enabled or self._locked_track_id is None:
            return None
        packet = self._pending_packet
        self._pending_packet = None
        if packet is None:
            packet = self._poll()
        if packet is None:
            return None
        matching = next(
            (
                tracklet
                for tracklet in _valid_tracklets(packet.tracklets, self._tracked_status)
                if _track_id(tracklet) == self._locked_track_id
            ),
            None,
        )
        if matching is None:
            return None
        return observation_from_tracklet(matching, packet)
