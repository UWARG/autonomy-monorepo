"""Detect and predict confirmed one-dimensional back-and-forth traffic."""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass

from utils.src.waypoint_utils import east_north_coordinate_offset_m

from .models import Point2D, TrafficAircraft
from .traffic import is_valid_traffic_aircraft

_MAX_HISTORY_S = 180.0
_MAX_SAMPLE_GAP_S = 2.5
_MIN_SAMPLES = 10
_MIN_SPAN_M = 8.0
_REVERSAL_CONFIRMATION_M = 3.0
_MOTION_THRESHOLD_TOLERANCE_M = 0.1
_MIN_TURN_PERIOD_S = 4.0
_MAX_CYCLE_PERIOD_S = 120.0
_MAX_PERIOD_VARIATION = 0.35
_MAX_CROSS_TRACK_M = 3.0
_MAX_ENDPOINT_SPREAD_M = 4.0
# Share of one sample interval's travel kept as endpoint uncertainty after
# reconstructing turn points (covers speed changes around the turn).
_RESIDUAL_SLACK_FRACTION = 0.25

# Shape of an aircraft's path, from its first turn.
UNKNOWN = "UNKNOWN"  # no turn yet: straight flight or stationary
BACK_AND_FORTH = "BACK_AND_FORTH"  # first turn was a reversal
NOT_BACK_AND_FORTH = "NOT_BACK_AND_FORTH"  # first turn was a corner (polygon)
# A first turn of at least this much (degrees) is a reversal. A triangle's
# corners turn 120 degrees, a square's 90.
_REVERSAL_MIN_TURN_DEG = 150.0
# A step heading this far (degrees) off the current leg starts a turn; two
# such steps in a row confirm it.
_TURN_DEVIATION_DEG = 45.0
# Steps shorter than this (meters) are merged with the next, so a hovering or
# slow aircraft's GPS jitter is not read as turning.
_MIN_STEP_M = 1.0
# The first leg's heading is taken once the aircraft has moved this far (m).
_MIN_LEG_M = 5.0


@dataclass(frozen=True, slots=True)
class _Sample:
    timestamp_s: float
    lat: float
    lon: float


@dataclass(frozen=True, slots=True)
class BackAndForthPrediction:
    """A reflected-motion model in the caller's local ENU frame."""

    center: Point2D
    axis_x: float
    axis_y: float
    lower_m: float
    upper_m: float
    current_m: float
    direction: int
    speed_mps: float
    period_s: float
    uncertainty_m: float
    uncertainty_growth_mps: float

    def along_at(self, elapsed_s: float) -> float:
        """Position along the axis (meters from ``center``) after ``elapsed_s``."""

        span_m = self.upper_m - self.lower_m
        travel_m = (
            self.current_m
            - self.lower_m
            + self.direction * self.speed_mps * max(0.0, elapsed_s)
        )
        phase_m = travel_m % (2.0 * span_m)
        return (
            self.lower_m + phase_m
            if phase_m <= span_m
            else self.upper_m - (phase_m - span_m)
        )

    def point_along(self, along_m: float) -> Point2D:
        return Point2D(
            self.center.x + self.axis_x * along_m,
            self.center.y + self.axis_y * along_m,
        )

    def position_at(self, elapsed_s: float) -> Point2D:
        along_m = self.along_at(elapsed_s)
        return Point2D(
            self.center.x + self.axis_x * along_m,
            self.center.y + self.axis_y * along_m,
        )

    def uncertainty_at(self, elapsed_s: float) -> float:
        return self.uncertainty_m + self.uncertainty_growth_mps * max(
            0.0, elapsed_s
        )


class BackAndForthHistory:
    """Keeps bounded per-aircraft history and exposes only repeatable patterns."""

    def __init__(self) -> None:
        self._samples: dict[int, deque[_Sample]] = {}

    def reset(self) -> None:
        self._samples.clear()

    def observe(
        self,
        traffic: tuple[TrafficAircraft, ...],
        *,
        sample_time_s: float | None,
        origin_lat: float,
        origin_lon: float,
    ) -> dict[int, BackAndForthPrediction]:
        if sample_time_s is None or not math.isfinite(sample_time_s):
            return {}

        observed: set[int] = set()
        for aircraft in traffic:
            observed.add(aircraft.aircraft_index)
            samples = self._samples.setdefault(
                aircraft.aircraft_index, deque()
            )
            if samples and sample_time_s <= samples[-1].timestamp_s:
                continue
            if (
                samples
                and sample_time_s - samples[-1].timestamp_s > _MAX_SAMPLE_GAP_S
            ):
                samples.clear()
            if not is_valid_traffic_aircraft(aircraft):
                samples.clear()
                continue

            samples.append(
                _Sample(sample_time_s, aircraft.lat, aircraft.lon)
            )
            while (
                samples
                and sample_time_s - samples[0].timestamp_s > _MAX_HISTORY_S
            ):
                samples.popleft()

        for aircraft_index in tuple(self._samples):
            if aircraft_index not in observed:
                del self._samples[aircraft_index]

        return self._predictions(origin_lat=origin_lat, origin_lon=origin_lon)

    def classification(self, aircraft_index: int) -> str:
        """The shape of the aircraft's path so far, from its first turn."""

        samples = self._samples.get(aircraft_index)
        if not samples:
            return UNKNOWN
        first = samples[0]
        return classify_path(
            tuple(
                Point2D(
                    *east_north_coordinate_offset_m(
                        first.lat, first.lon, sample.lat, sample.lon
                    )
                )
                for sample in samples
            )
        )

    def recent_tracks(
        self, *, window_s: float, origin_lat: float, origin_lon: float
    ) -> dict[int, tuple[Point2D, ...]]:
        """
        The positions over the last ``window_s`` of each aircraft whose path
        is back and forth, oldest first, in the caller's local ENU frame.
        """

        tracks = {}
        for aircraft_index, samples in self._samples.items():
            if not samples or self.classification(aircraft_index) != BACK_AND_FORTH:
                continue
            newest_s = samples[-1].timestamp_s
            tracks[aircraft_index] = tuple(
                Point2D(
                    *east_north_coordinate_offset_m(
                        origin_lat, origin_lon, sample.lat, sample.lon
                    )
                )
                for sample in samples
                if newest_s - sample.timestamp_s <= window_s
            )
        return tracks

    def _predictions(
        self, *, origin_lat: float, origin_lon: float
    ) -> dict[int, BackAndForthPrediction]:
        predictions = {}
        for aircraft_index, samples in self._samples.items():
            if self.classification(aircraft_index) != BACK_AND_FORTH:
                continue
            prediction = _detect_pattern(
                tuple(samples), origin_lat=origin_lat, origin_lon=origin_lon
            )
            if prediction is not None:
                predictions[aircraft_index] = prediction
        return predictions


def _angle_deg(ax: float, ay: float, bx: float, by: float) -> float:
    cosine = (ax * bx + ay * by) / (math.hypot(ax, ay) * math.hypot(bx, by))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def first_turn_deg(points: tuple[Point2D, ...]) -> float | None:
    """
    How sharply (degrees) the path through ``points`` first turns: about 180
    for a reversal, 120 at a triangle's corner. None if it has not turned.
    """

    if len(points) < 2:
        return None
    anchor = points[0]
    leg_index = next(
        (
            index
            for index, point in enumerate(points)
            if math.hypot(point.x - anchor.x, point.y - anchor.y) >= _MIN_LEG_M
        ),
        None,
    )
    if leg_index is None:
        return None
    leg_x = points[leg_index].x - anchor.x
    leg_y = points[leg_index].y - anchor.y
    previous = points[leg_index]
    deviating_steps = 0
    for point in points[leg_index + 1 :]:
        step_x, step_y = point.x - previous.x, point.y - previous.y
        step_m = math.hypot(step_x, step_y)
        if step_m < _MIN_STEP_M:
            continue
        previous = point
        turn_deg = _angle_deg(leg_x, leg_y, step_x, step_y)
        if turn_deg <= _TURN_DEVIATION_DEG:
            deviating_steps = 0
            # Follow gentle drift in the leg's heading.
            leg_norm = math.hypot(leg_x, leg_y)
            leg_x = leg_x / leg_norm + step_x / step_m
            leg_y = leg_y / leg_norm + step_y / step_m
            continue
        deviating_steps += 1
        if deviating_steps >= 2:
            # The second step is fully on the new leg; the first may cut
            # the corner.
            return turn_deg
    return None


def classify_path(points: tuple[Point2D, ...]) -> str:
    """
    ``BACK_AND_FORTH`` if the path's first turn is a reversal,
    ``NOT_BACK_AND_FORTH`` if it is a corner, ``UNKNOWN`` before any turn.
    """

    turn_deg = first_turn_deg(points)
    if turn_deg is None:
        return UNKNOWN
    return BACK_AND_FORTH if turn_deg >= _REVERSAL_MIN_TURN_DEG else NOT_BACK_AND_FORTH


def _detect_pattern(
    samples: tuple[_Sample, ...], *, origin_lat: float, origin_lon: float
) -> BackAndForthPrediction | None:
    if len(samples) < _MIN_SAMPLES:
        return None
    if samples[-1].timestamp_s - samples[0].timestamp_s > _MAX_HISTORY_S:
        return None
    if any(
        later.timestamp_s - earlier.timestamp_s > _MAX_SAMPLE_GAP_S
        for earlier, later in zip(samples, samples[1:])
    ):
        return None

    points = [
        east_north_coordinate_offset_m(
            origin_lat, origin_lon, sample.lat, sample.lon
        )
        for sample in samples
    ]
    center_x = statistics.fmean(point[0] for point in points)
    center_y = statistics.fmean(point[1] for point in points)
    variance_x = statistics.fmean((x - center_x) ** 2 for x, _ in points)
    variance_y = statistics.fmean((y - center_y) ** 2 for _, y in points)
    covariance = statistics.fmean(
        (x - center_x) * (y - center_y) for x, y in points
    )
    angle = 0.5 * math.atan2(2.0 * covariance, variance_x - variance_y)
    axis_x, axis_y = math.cos(angle), math.sin(angle)
    across_x, across_y = -axis_y, axis_x

    along = [
        (x - center_x) * axis_x + (y - center_y) * axis_y
        for x, y in points
    ]
    cross_track = [
        abs((x - center_x) * across_x + (y - center_y) * across_y)
        for x, y in points
    ]
    if max(cross_track) > _MAX_CROSS_TRACK_M:
        return None

    turns = _confirmed_turns(samples, along)
    if len(turns) < 5:
        return None
    recent_turns = turns[-5:]
    first_cycle_s = (
        recent_turns[2][0].timestamp_s - recent_turns[0][0].timestamp_s
    )
    second_cycle_s = (
        recent_turns[4][0].timestamp_s - recent_turns[2][0].timestamp_s
    )
    if min(first_cycle_s, second_cycle_s) < _MIN_TURN_PERIOD_S:
        return None
    if max(first_cycle_s, second_cycle_s) > _MAX_CYCLE_PERIOD_S:
        return None
    period_variation = abs(first_cycle_s - second_cycle_s) / max(
        first_cycle_s, second_cycle_s
    )
    if period_variation > _MAX_PERIOD_VARIATION:
        return None

    recent = range(max(1, len(samples) - 8), len(samples))
    recent_rates = [
        (along[index] - along[index - 1])
        / (samples[index].timestamp_s - samples[index - 1].timestamp_s)
        for index in recent
        if samples[index].timestamp_s > samples[index - 1].timestamp_s
    ]
    moving_rates = [rate for rate in recent_rates if abs(rate) >= 0.2]
    if not moving_rates:
        return None
    direction = 1 if moving_rates[-1] > 0 else -1
    rate_speed_mps = statistics.median(abs(rate) for rate in moving_rates)
    sample_dt_s = max(
        samples[index].timestamp_s - samples[index - 1].timestamp_s
        for index in recent
    )
    # The furthest sample can be up to one sample interval of travel short of
    # where the aircraft turned.
    sampling_slack_m = rate_speed_mps * sample_dt_s

    def turn_point_m(turn: tuple[_Sample, float, int, int]) -> float:
        # At constant speed, the furthest sample and its neighbour on the far
        # side of the turn fix where the turn was.
        _, extreme_m, index, side = turn
        neighbours = [
            other for other in (index - 1, index + 1) if 0 <= other < len(along)
        ]
        neighbour = max(neighbours, key=lambda other: side * along[other])
        gap_s = abs(samples[neighbour].timestamp_s - samples[index].timestamp_s)
        overshoot_m = 0.5 * (
            side * (along[neighbour] - extreme_m) + rate_speed_mps * gap_s
        )
        return extreme_m + side * min(max(overshoot_m, 0.0), sampling_slack_m)

    same_end_a = [turn_point_m(turn) for turn in recent_turns[::2]]
    same_end_b = [turn_point_m(turn) for turn in recent_turns[1::2]]
    endpoint_a_spread = max(same_end_a) - min(same_end_a)
    endpoint_b_spread = max(same_end_b) - min(same_end_b)
    if max(endpoint_a_spread, endpoint_b_spread) > max(
        _MAX_ENDPOINT_SPREAD_M, sampling_slack_m
    ):
        return None

    lower_m = min(statistics.fmean(same_end_a), statistics.fmean(same_end_b))
    upper_m = max(statistics.fmean(same_end_a), statistics.fmean(same_end_b))
    span_m = upper_m - lower_m
    if span_m < _MIN_SPAN_M:
        return None

    current_m = along[-1]
    endpoint_tolerance_m = max(
        min(2.0, _MAX_ENDPOINT_SPREAD_M / 2.0), sampling_slack_m
    )
    if (
        current_m < lower_m - endpoint_tolerance_m
        or current_m > upper_m + endpoint_tolerance_m
    ):
        return None
    endpoint_error_m = max(lower_m - current_m, current_m - upper_m, 0.0)
    current_m = min(upper_m, max(lower_m, current_m))

    # The net motion over the last interval points the old way if the aircraft
    # turned partway through it. Replay that interval at cruise speed from the
    # sample before, reflecting at the endpoints, to get the current direction.
    if len(samples) >= 3:
        before_m = along[-3]
        previous_m = min(upper_m, max(lower_m, along[-2]))
        if abs(previous_m - before_m) > 1e-9:
            heading_up = previous_m > before_m
            phase_m = (
                previous_m - lower_m
                if heading_up
                else 2.0 * span_m - (previous_m - lower_m)
            )
            phase_m += rate_speed_mps * (
                samples[-1].timestamp_s - samples[-2].timestamp_s
            )
            direction = 1 if phase_m % (2.0 * span_m) < span_m else -1

    period_s = statistics.fmean((first_cycle_s, second_cycle_s))
    # Average speed that reproduces the observed period over the modeled span,
    # so the reflected model stays in phase even if the aircraft slows or
    # pauses at the ends.
    speed_mps = 2.0 * span_m / period_s
    if speed_mps < 0.2:
        return None

    endpoint_spread_m = max(endpoint_a_spread, endpoint_b_spread)
    # Time per end not spent cruising (pausing or slowing to turn). The model
    # turns instantly, so near a turn it can be off by that much travel.
    pause_s = max(0.0, period_s - 2.0 * span_m / rate_speed_mps) / 2.0
    uncertainty_m = (
        max(1.0, max(cross_track), endpoint_spread_m, endpoint_error_m)
        + _RESIDUAL_SLACK_FRACTION * sampling_slack_m
        + rate_speed_mps * pause_s
    )
    # Phase drift from period variation, plus the position error from moving at
    # the average speed while the aircraft actually moves at its cruise speed.
    uncertainty_growth_mps = span_m * abs(first_cycle_s - second_cycle_s) / (
        first_cycle_s * second_cycle_s
    ) + abs(rate_speed_mps - speed_mps)
    return BackAndForthPrediction(
        center=Point2D(center_x, center_y),
        axis_x=axis_x,
        axis_y=axis_y,
        lower_m=lower_m,
        upper_m=upper_m,
        current_m=current_m,
        direction=direction,
        speed_mps=speed_mps,
        period_s=period_s,
        uncertainty_m=uncertainty_m,
        uncertainty_growth_mps=uncertainty_growth_mps,
    )


def _confirmed_turns(
    samples: tuple[_Sample, ...], positions_m: list[float]
) -> list[tuple[_Sample, float, int, int]]:
    """Each confirmed turn: its extreme sample, position, index and side (+1 upper)."""

    turns: list[tuple[_Sample, float, int, int]] = []
    direction = 0
    extreme_index = 0
    for index in range(1, len(positions_m)):
        position_m = positions_m[index]
        extreme_m = positions_m[extreme_index]
        if direction == 0:
            if abs(position_m - extreme_m) >= (
                _REVERSAL_CONFIRMATION_M - _MOTION_THRESHOLD_TOLERANCE_M
            ):
                direction = 1 if position_m > extreme_m else -1
                extreme_index = index
            continue

        if direction * (position_m - extreme_m) >= 0.0:
            extreme_index = index
        elif direction * (extreme_m - position_m) >= (
            _REVERSAL_CONFIRMATION_M - _MOTION_THRESHOLD_TOLERANCE_M
        ):
            turns.append(
                (samples[extreme_index], extreme_m, extreme_index, direction)
            )
            direction *= -1
            extreme_index = index
    return turns
