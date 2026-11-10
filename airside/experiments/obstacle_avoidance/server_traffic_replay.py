#!/usr/bin/env python3
"""Validate a receive-only AEAC capture and map a route encounter into SITL."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

EARTH_RADIUS_M = 6_371_000.0
NOMINAL_ROUTE_SPEED_MPS = 1.5
LIVE_TRAFFIC_MARGIN_M = 5.75
REPLAY_COMPLETION_MARGIN_S = 20.0


@dataclass(frozen=True)
class Aircraft:
    index: int
    lat: float
    lon: float
    altitude_m: float
    speed_mps: float
    heading_deg: float
    keepaway_m: float
    vertical_keepaway_m: float


@dataclass(frozen=True)
class Frame:
    time_s: float
    aircraft: tuple[Aircraft, ...]


@dataclass(frozen=True)
class RouteMatch:
    aircraft_index: int
    segment_index: int
    event_time_s: float
    centerline_distance_m: float
    along_segment_m: float
    segment_length_m: float
    frame_count: int
    capture_duration_s: float
    min_interval_s: float
    max_interval_s: float


def east_north(
    origin: tuple[float, float], point: tuple[float, float]
) -> tuple[float, float]:
    east = (
        math.radians(point[1] - origin[1])
        * EARTH_RADIUS_M
        * math.cos(math.radians(origin[0]))
    )
    north = math.radians(point[0] - origin[0]) * EARTH_RADIUS_M
    return east, north


def _number(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"invalid {label}")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"invalid {label}") from error
    if not math.isfinite(number):
        raise ValueError(f"invalid {label}")
    return number


def load_route(path: Path) -> tuple[tuple[float, float], ...]:
    source = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(source, dict) or not isinstance(source.get("waypoints"), list):
        raise ValueError("route must contain a waypoints list")
    points = []
    for waypoint in source["waypoints"]:
        lat = _number(waypoint["lat"], "route latitude")
        lon = _number(waypoint["lon"], "route longitude")
        if not -90 <= lat <= 90 or not -180 <= lon <= 180:
            raise ValueError("route coordinate outside WGS84 range")
        points.append((lat, lon))
    if len(points) < 2:
        raise ValueError("route needs at least two waypoints")
    return tuple(points)


def load_route_altitudes(path: Path) -> tuple[float, ...]:
    """Load the relative-altitude target for every route waypoint."""

    source = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(source, dict) or not isinstance(source.get("waypoints"), list):
        raise ValueError("route must contain a waypoints list")
    altitudes = tuple(
        _number(waypoint.get("alt"), "route altitude")
        for waypoint in source["waypoints"]
    )
    if len(altitudes) < 2:
        raise ValueError("route needs at least two waypoints")
    return altitudes


def load_capture(path: Path) -> tuple[Frame, ...]:
    frames: list[Frame] = []
    first_time: float | None = None
    previous_time = -math.inf
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not line.strip():
            continue
        sample = json.loads(line)
        timestamp = _number(sample.get("received_monotonic_s"), "capture time")
        if timestamp <= previous_time:
            raise ValueError(f"non-increasing capture time at line {line_number}")
        previous_time = timestamp
        if first_time is None:
            first_time = timestamp
        if sample.get("problems"):
            raise ValueError(f"invalid traffic snapshot at line {line_number}")
        seen: set[int] = set()
        aircraft: list[Aircraft] = []
        for item in sample["aircraft"]:
            index_number = _number(item["aircraft_index"], "aircraft index")
            if not index_number.is_integer():
                raise ValueError(f"non-integer aircraft index at line {line_number}")
            index = int(index_number)
            if index in seen or not 0 <= index <= 255:
                raise ValueError(
                    f"duplicate or invalid aircraft index at line {line_number}"
                )
            seen.add(index)
            lat = _number(item["latitude_deg"], "latitude")
            lon = _number(item["longitude_deg"], "longitude")
            speed = _number(item["speed_mps"], "speed")
            keepaway = _number(item["horizontal_keepaway_m"], "keep-away")
            vertical = _number(item["vertical_keepaway_m"], "vertical keep-away")
            if not -90 <= lat <= 90 or not -180 <= lon <= 180:
                raise ValueError(f"aircraft outside WGS84 range at line {line_number}")
            if speed < 0 or keepaway <= 0 or vertical < 0:
                raise ValueError(
                    f"invalid aircraft motion or radius at line {line_number}"
                )
            aircraft.append(
                Aircraft(
                    index,
                    lat,
                    lon,
                    _number(item["altitude_agl_m"], "altitude AGL"),
                    speed,
                    _number(item["heading_deg_true"], "heading"),
                    keepaway,
                    vertical,
                )
            )
        frames.append(Frame(timestamp - first_time, tuple(aircraft)))
    if len(frames) < 2:
        raise ValueError("capture needs at least two traffic snapshots")
    return tuple(frames)


def _segment_distance(
    point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]
) -> tuple[float, float, float]:
    x, y = east_north(start, point)
    dx, dy = east_north(start, end)
    length_sq = dx * dx + dy * dy
    if length_sq <= 0:
        raise ValueError("route contains a zero-length segment")
    fraction = max(0.0, min(1.0, (x * dx + y * dy) / length_sq))
    return (
        math.hypot(x - fraction * dx, y - fraction * dy),
        fraction * math.sqrt(length_sq),
        math.sqrt(length_sq),
    )


def route_length_m(route: tuple[tuple[float, float], ...]) -> float:
    """Length of one closed lap through the configured route."""

    return sum(
        math.hypot(*east_north(start, end))
        for start, end in zip(route, route[1:] + route[:1])
    )


def required_replay_duration_s(
    route: tuple[tuple[float, float], ...],
) -> float:
    """Minimum capture window for a nominal lap plus avoidance time."""

    return route_length_m(route) / NOMINAL_ROUTE_SPEED_MPS + REPLAY_COMPLETION_MARGIN_S


def validate_replay_window(
    frames: tuple[Frame, ...],
    route: tuple[tuple[float, float], ...],
    start_s: float,
) -> float:
    """Validate an explicitly selected, timing-preserving full-lap window."""

    if not math.isfinite(start_s) or start_s < frames[0].time_s:
        raise ValueError("replay start must be within the capture")
    required_s = required_replay_duration_s(route)
    available_s = frames[-1].time_s - start_s
    if available_s < required_s:
        raise ValueError(
            f"capture has {available_s:.1f}s after replay start; "
            f"full route requires at least {required_s:.1f}s"
        )
    return start_s + required_s


def mission_path_indices(
    route_count: int, entry_index: int, *, next_lap: bool = True
) -> tuple[tuple[int, ...], int]:
    """Mirror the engine's closed-lap path window for one route entry."""

    if route_count < 2 or not 1 <= entry_index <= route_count:
        raise ValueError("invalid route entry")
    last = route_count + 2 if next_lap else route_count
    first = -1 if entry_index >= 1 else 0
    entries = range(max(entry_index - 2, first), min(entry_index + 2, last) + 1)
    indices = tuple(entry % route_count for entry in entries)
    return indices, entry_index - entries.start


def match_route(
    frames: tuple[Frame, ...],
    route: tuple[tuple[float, float], ...],
    *,
    own_index: int | None = None,
    start_s: float | None = None,
    end_s: float | None = None,
) -> RouteMatch | None:
    """Find a moving index whose captured position intersects an inflated route corridor."""

    segments = tuple(zip(route, route[1:] + route[:1]))
    sightings: dict[int, int] = {}
    candidates = []
    for frame in frames:
        if start_s is not None and frame.time_s < start_s:
            continue
        if end_s is not None and frame.time_s > end_s:
            continue
        for aircraft in frame.aircraft:
            if aircraft.index == own_index or aircraft.speed_mps <= 0.1:
                continue
            sightings[aircraft.index] = sightings.get(aircraft.index, 0) + 1
            for segment_index, (start, end) in enumerate(segments):
                distance, along, length = _segment_distance(
                    (aircraft.lat, aircraft.lon), start, end
                )
                # Match the live moving-traffic margin and 1 m clearance.
                buffer_m = LIVE_TRAFFIC_MARGIN_M
                if distance <= aircraft.keepaway_m + buffer_m + 1.0:
                    candidates.append(
                        (
                            distance,
                            aircraft.index,
                            segment_index,
                            frame.time_s,
                            along,
                            length,
                        )
                    )
    candidates = [candidate for candidate in candidates if sightings[candidate[1]] >= 3]
    if not candidates:
        return None
    usable = [
        candidate
        for candidate in candidates
        if candidate[3] >= candidate[4] / NOMINAL_ROUTE_SPEED_MPS - 2.0
        and frames[-1].time_s
        >= max(0.0, candidate[3] - candidate[4] / NOMINAL_ROUTE_SPEED_MPS)
        + candidate[5] / NOMINAL_ROUTE_SPEED_MPS
        + 5.0
    ]
    distance, index, segment, event_time, along, length = min(usable or candidates)
    intervals = [
        current.time_s - previous.time_s
        for previous, current in zip(frames, frames[1:])
    ]
    return RouteMatch(
        index,
        segment,
        event_time,
        distance,
        along,
        length,
        len(frames),
        frames[-1].time_s,
        min(intervals),
        max(intervals),
    )


def truth_at(
    frames: tuple[Frame, ...], elapsed_s: float, index: int
) -> Aircraft | None:
    """Interpolate captured positions for independent SITL distance measurement."""

    if elapsed_s < frames[0].time_s or elapsed_s > frames[-1].time_s:
        return None
    for left, right in zip(frames, frames[1:]):
        if not left.time_s <= elapsed_s <= right.time_s:
            continue
        a = next((item for item in left.aircraft if item.index == index), None)
        b = next((item for item in right.aircraft if item.index == index), None)
        if a is None or b is None:
            return a if elapsed_s == left.time_s else None
        fraction = (elapsed_s - left.time_s) / (right.time_s - left.time_s)
        return Aircraft(
            index,
            a.lat + fraction * (b.lat - a.lat),
            a.lon + fraction * (b.lon - a.lon),
            a.altitude_m + fraction * (b.altitude_m - a.altitude_m),
            a.speed_mps + fraction * (b.speed_mps - a.speed_mps),
            a.heading_deg,
            a.keepaway_m,
            a.vertical_keepaway_m,
        )
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", required=True, type=Path)
    parser.add_argument("--route", required=True, type=Path)
    parser.add_argument("--own-aircraft-index", type=int)
    parser.add_argument("--replay-start-s", type=float, default=0.0)
    args = parser.parse_args()
    frames = load_capture(args.capture)
    route = load_route(args.route)
    try:
        replay_end_s = validate_replay_window(frames, route, args.replay_start_s)
    except ValueError as error:
        print(
            json.dumps(
                {
                    "result": "CAPTURE_INSUFFICIENT_FOR_REPLAY",
                    "reason": str(error),
                    "frames": len(frames),
                    "capture_duration_s": frames[-1].time_s,
                    "route_length_m": route_length_m(route),
                    "required_duration_s": required_replay_duration_s(route),
                    "replay_start_s": args.replay_start_s,
                },
                sort_keys=True,
            )
        )
        return 2
    match = match_route(
        frames,
        route,
        own_index=args.own_aircraft_index,
        start_s=args.replay_start_s,
        end_s=replay_end_s,
    )
    if match is None:
        print(json.dumps({"result": "NO_MOVING_ROUTE_MATCH", "frames": len(frames)}))
        return 2
    result = {
        "result": "MATCH",
        **match.__dict__,
        "replay_start_s": args.replay_start_s,
        "replay_end_s": replay_end_s,
        "route_length_m": route_length_m(route),
        "required_duration_s": required_replay_duration_s(route),
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if result["result"] == "MATCH" else 2


if __name__ == "__main__":
    raise SystemExit(main())
