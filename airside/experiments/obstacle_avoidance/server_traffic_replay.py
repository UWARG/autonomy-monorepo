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


def match_route(
    frames: tuple[Frame, ...],
    route: tuple[tuple[float, float], ...],
    *,
    own_index: int | None = None,
) -> RouteMatch | None:
    """Find a moving index whose captured position intersects an inflated route corridor."""

    segments = tuple(zip(route, route[1:] + route[:1]))
    sightings: dict[int, int] = {}
    candidates = []
    for frame in frames:
        for aircraft in frame.aircraft:
            if aircraft.index == own_index or aircraft.speed_mps <= 0.1:
                continue
            sightings[aircraft.index] = sightings.get(aircraft.index, 0) + 1
            for segment_index, (start, end) in enumerate(segments):
                distance, along, length = _segment_distance(
                    (aircraft.lat, aircraft.lon), start, end
                )
                # Use the same nominal live buffer and planner clearance as #237.
                buffer_m = max(5.0, (aircraft.speed_mps + 1.5) * 1.5)
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
        if candidate[3] >= candidate[4] / 1.5 - 2.0
        and frames[-1].time_s
        >= max(0.0, candidate[3] - candidate[4] / 1.5) + candidate[5] / 1.5 + 5.0
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


def replay_start_s(match: RouteMatch, frames: tuple[Frame, ...]) -> float:
    """Choose a capture window that retains the original relative timestamps."""

    lead_s = match.along_segment_m / 1.5
    start_s = max(0.0, match.event_time_s - lead_s)
    if match.event_time_s < lead_s - 2.0:
        raise ValueError("capture lacks lead-in before route encounter")
    if frames[-1].time_s - start_s < match.segment_length_m / 1.5 + 5.0:
        raise ValueError("capture ends before the route leg can complete")
    return start_s


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
    args = parser.parse_args()
    frames = load_capture(args.capture)
    route = load_route(args.route)
    match = match_route(frames, route, own_index=args.own_aircraft_index)
    if match is None:
        print(json.dumps({"result": "NO_MOVING_ROUTE_MATCH", "frames": len(frames)}))
        return 2
    result = {"result": "MATCH", **match.__dict__}
    try:
        result["replay_start_s"] = replay_start_s(match, frames)
    except ValueError as error:
        result["result"] = "CAPTURE_INSUFFICIENT_FOR_REPLAY"
        result["reason"] = str(error)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["result"] == "MATCH" else 2


if __name__ == "__main__":
    raise SystemExit(main())
