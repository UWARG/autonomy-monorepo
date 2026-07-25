"""Latest-observation filtering, raw safety, timing, and control computation."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

from follow_controller import FollowConfig, FollowSetpoint, compute_setpoint
from range_rate import ReflexConfig, ReflexMonitor


@dataclass(frozen=True)
class RuntimeObservation:
    x_m: float
    y_m: float
    z_m: float
    track_id: int
    sequence_num: int
    capture_time_s: float
    receive_time_s: float
    detector_capture_time_s: Optional[float] = None
    detector_sequence_num: Optional[int] = None
    detector_confirmed: bool = True
    spatial_control_valid: bool = True

    @property
    def range_m(self) -> float:
        return math.sqrt(self.x_m**2 + self.y_m**2 + self.z_m**2)


@dataclass(frozen=True)
class RuntimeOutput:
    fresh: bool
    target_age_s: float
    raw_range_m: Optional[float]
    proximity_emergency: bool
    setpoint: Optional[FollowSetpoint]
    filtered_xyz_m: Optional[tuple[float, float, float]]


@dataclass(frozen=True)
class TimingMetrics:
    effective_fps: float
    detector_fps: float
    tracker_fps: float
    latency_p50_s: float
    latency_p95_s: float
    latency_p99_s: float
    sequence_gaps: int
    detector_sequence_gaps: int
    tracker_sequence_gaps: int
    tracker_only_frames: int


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentile
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def choose_stream_hz(source_fps: float) -> float:
    """20 Hz minimum, source FPS rounded up to 5 Hz, 50 Hz maximum."""
    rounded = math.ceil(max(source_fps, 0.0) / 5.0) * 5.0
    return min(50.0, max(20.0, rounded))


def choose_ema_alpha(frame_period_s: float, max_delay_s: float = 0.05) -> float:
    """Smallest 0.1-step alpha whose DC group delay is within the budget."""
    for tenth in range(1, 11):
        alpha = tenth / 10.0
        delay = ((1.0 - alpha) / alpha) * frame_period_s
        if delay <= max_delay_s + 1e-12:
            return alpha
    return 1.0


class LatestObservationController:
    """Use raw range for safety and EMA XYZ only for follow commands."""

    def __init__(
        self,
        follow: FollowConfig = FollowConfig(),
        reflex: ReflexConfig = ReflexConfig(),
        freshness_s: float = 0.3,
        ema_alpha: float = 1.0,
        metrics_window: int = 600,
        detector_stride: int = 1,
    ) -> None:
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0, 1]")
        if detector_stride not in (1, 2):
            raise ValueError("detector_stride must be 1 or 2")
        self.follow = follow
        self.freshness_s = freshness_s
        self.ema_alpha = ema_alpha
        self.detector_stride = detector_stride
        self._reflex = ReflexMonitor(reflex)
        self._latest: Optional[RuntimeObservation] = None
        self._filtered: Optional[tuple[float, float, float]] = None
        self._last_detector_sequence: Optional[int] = None
        self._last_tracker_sequence: Optional[int] = None
        self._detector_capture_times: Deque[float] = deque(maxlen=metrics_window)
        self._tracker_capture_times: Deque[float] = deque(maxlen=metrics_window)
        self._latencies: Deque[float] = deque(maxlen=metrics_window)
        self._detector_sequence_gaps = 0
        self._tracker_sequence_gaps = 0
        self._tracker_only_frames = 0
        self._proximity_emergency = False

    @property
    def latest(self) -> Optional[RuntimeObservation]:
        return self._latest

    def clear(self) -> None:
        self._latest = None
        self._filtered = None
        self._last_detector_sequence = None
        self._last_tracker_sequence = None
        self._reflex.reset()
        self._proximity_emergency = False

    def update(self, observation: RuntimeObservation) -> bool:
        values = (observation.x_m, observation.y_m, observation.z_m)
        if not all(math.isfinite(value) for value in values) or observation.z_m <= 0.0:
            return False
        if self._last_tracker_sequence is not None:
            if observation.sequence_num <= self._last_tracker_sequence:
                return False
            if observation.sequence_num > self._last_tracker_sequence + 1:
                self._tracker_sequence_gaps += (
                    observation.sequence_num - self._last_tracker_sequence - 1
                )
        self._last_tracker_sequence = observation.sequence_num
        self._tracker_capture_times.append(observation.capture_time_s)

        # Propagated tracklets retain identity but never refresh spatial safety,
        # range rate, control, or detector timing.
        if not observation.detector_confirmed:
            self._tracker_only_frames += 1
            return False

        detector_capture_s = (
            observation.detector_capture_time_s
            if observation.detector_capture_time_s is not None
            else observation.capture_time_s
        )
        detector_sequence = (
            observation.detector_sequence_num
            if observation.detector_sequence_num is not None
            else observation.sequence_num
        )
        if self._last_detector_sequence is not None:
            if detector_sequence <= self._last_detector_sequence:
                return False
            delta = detector_sequence - self._last_detector_sequence
            self._detector_sequence_gaps += max(
                0, math.ceil(delta / self.detector_stride) - 1
            )
        self._last_detector_sequence = detector_sequence
        self._detector_capture_times.append(detector_capture_s)
        self._latencies.append(
            max(0.0, observation.receive_time_s - detector_capture_s)
        )
        if not observation.spatial_control_valid:
            return False

        self._latest = RuntimeObservation(
            x_m=observation.x_m,
            y_m=observation.y_m,
            z_m=observation.z_m,
            track_id=observation.track_id,
            sequence_num=detector_sequence,
            capture_time_s=detector_capture_s,
            receive_time_s=observation.receive_time_s,
            detector_capture_time_s=detector_capture_s,
            detector_sequence_num=detector_sequence,
            detector_confirmed=True,
            spatial_control_valid=True,
        )
        if self._filtered is None:
            self._filtered = values
        else:
            alpha = self.ema_alpha
            self._filtered = tuple(
                alpha * sample + (1.0 - alpha) * previous
                for sample, previous in zip(values, self._filtered)
            )
        # Capture time is essential here: burst-delivered packets must retain
        # their true spacing for closing-rate prediction.
        self._proximity_emergency = self._reflex.update(
            observation.range_m, detector_capture_s
        )
        return True

    def evaluate(self, now_s: float) -> RuntimeOutput:
        if self._latest is None:
            return RuntimeOutput(False, math.inf, None, False, None, self._filtered)
        age = max(0.0, now_s - self._latest.capture_time_s)
        fresh = age <= self.freshness_s
        raw_range = self._latest.range_m
        danger = self._proximity_emergency if fresh else False
        setpoint = None
        if fresh and self._filtered is not None:
            x_m, y_m, z_m = self._filtered
            setpoint = compute_setpoint(
                x_m * 1000.0, y_m * 1000.0, z_m * 1000.0, self.follow
            )
        return RuntimeOutput(fresh, age, raw_range, danger, setpoint, self._filtered)

    def metrics(self) -> TimingMetrics:
        detector_periods = [
            later - earlier
            for earlier, later in zip(
                self._detector_capture_times,
                list(self._detector_capture_times)[1:],
            )
            if later > earlier
        ]
        tracker_periods = [
            later - earlier
            for earlier, later in zip(
                self._tracker_capture_times,
                list(self._tracker_capture_times)[1:],
            )
            if later > earlier
        ]
        detector_fps = (
            1.0 / (sum(detector_periods) / len(detector_periods))
            if detector_periods
            else 0.0
        )
        tracker_fps = (
            1.0 / (sum(tracker_periods) / len(tracker_periods))
            if tracker_periods
            else 0.0
        )
        latencies = list(self._latencies)
        return TimingMetrics(
            effective_fps=detector_fps,
            detector_fps=detector_fps,
            tracker_fps=tracker_fps,
            latency_p50_s=_percentile(latencies, 0.50),
            latency_p95_s=_percentile(latencies, 0.95),
            latency_p99_s=_percentile(latencies, 0.99),
            sequence_gaps=self._detector_sequence_gaps,
            detector_sequence_gaps=self._detector_sequence_gaps,
            tracker_sequence_gaps=self._tracker_sequence_gaps,
            tracker_only_frames=self._tracker_only_frames,
        )
