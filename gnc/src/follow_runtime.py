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
    latency_p50_s: float
    latency_p95_s: float
    latency_p99_s: float
    sequence_gaps: int


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
    ) -> None:
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0, 1]")
        self.follow = follow
        self.freshness_s = freshness_s
        self.ema_alpha = ema_alpha
        self._reflex = ReflexMonitor(reflex)
        self._latest: Optional[RuntimeObservation] = None
        self._filtered: Optional[tuple[float, float, float]] = None
        self._last_sequence: Optional[int] = None
        self._capture_times: Deque[float] = deque(maxlen=metrics_window)
        self._latencies: Deque[float] = deque(maxlen=metrics_window)
        self._sequence_gaps = 0
        self._proximity_emergency = False

    @property
    def latest(self) -> Optional[RuntimeObservation]:
        return self._latest

    def clear(self) -> None:
        self._latest = None
        self._filtered = None
        self._last_sequence = None
        self._reflex.reset()
        self._proximity_emergency = False

    def update(self, observation: RuntimeObservation) -> bool:
        values = (observation.x_m, observation.y_m, observation.z_m)
        if not all(math.isfinite(value) for value in values) or observation.z_m <= 0.0:
            return False
        if self._last_sequence is not None:
            if observation.sequence_num <= self._last_sequence:
                return False
            if observation.sequence_num > self._last_sequence + 1:
                self._sequence_gaps += observation.sequence_num - self._last_sequence - 1
        self._last_sequence = observation.sequence_num
        self._latest = observation
        self._capture_times.append(observation.capture_time_s)
        self._latencies.append(max(0.0, observation.receive_time_s - observation.capture_time_s))
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
            observation.range_m, observation.capture_time_s
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
        periods = [
            later - earlier
            for earlier, later in zip(self._capture_times, list(self._capture_times)[1:])
            if later > earlier
        ]
        fps = 1.0 / (sum(periods) / len(periods)) if periods else 0.0
        latencies = list(self._latencies)
        return TimingMetrics(
            effective_fps=fps,
            latency_p50_s=_percentile(latencies, 0.50),
            latency_p95_s=_percentile(latencies, 0.95),
            latency_p99_s=_percentile(latencies, 0.99),
            sequence_gaps=self._sequence_gaps,
        )
