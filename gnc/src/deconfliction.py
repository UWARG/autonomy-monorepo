from __future__ import annotations

import math

from utils.src.coordinates import anchor_for_boundary, coordinate_to_enu, enu_to_coordinate
from utils.src.types import Coordinate, PathConflict

Point = tuple[float, float]


def orientation(a: Point, b: Point, c: Point) -> float:
    """Positive if c is left of a-b, negative if its right, and 0 if on the line"""
    ax, ay = a
    bx, by = b
    cx, cy = c
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def closest_point_on_segment(p: Point, a: Point, b: Point) -> Point:
    """The point on segment a-b nearest to p"""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return a # a and b are the same point 
    # How far along a-b the nearest point, clamped to the ends -> projection factor
    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t = max(0.0, min(1.0, t)) 
    return (ax + t * dx, ay + t * dy)


def closest_points(p1: Point, p2: Point, q1: Point, q2: Point) -> tuple[Point, Point]:
    """The closest pair of points, one on segment p1-p2 and one on q1-q2"""
    # If the segments don't cross, the closest pair always includes an end
    # of one of them.
    candidates = [
        (p1, closest_point_on_segment(p1, q1, q2)),
        (p2, closest_point_on_segment(p2, q1, q2)),
        (closest_point_on_segment(q1, p1, p2), q1),
        (closest_point_on_segment(q2, p1, p2), q2),
    ]

    # If they cross, the crossing point is closer still.
    d1 = orientation(q1, q2, p1)
    d2 = orientation(q1, q2, p2)
    d3 = orientation(p1, p2, q1)
    d4 = orientation(p1, p2, q2)
    if d1 * d2 < 0 and d3 * d4 < 0:
        t = d1 / (d1 - d2)
        crossing = (p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))
        candidates.append((crossing, closest_point_on_segment(crossing, q1, q2)))

    return min(candidates, key=lambda pair: math.dist(*pair))


def to_segments(points: list[Point]) -> list[tuple[Point, Point]]:
    """Each consecutive pair of setpoints as a (start, end) segment.

    A drone with only its current position is hovering, so it becomes a
    single zero-length segment that other drones must still keep clear of.
    """
    if len(points) == 1:
        return [(points[0], points[0])]
    return list(zip(points, points[1:]))


def find_conflicts(
        paths: dict[str, list[Coordinate]], min_separation_m: float
) -> list[PathConflict]:
    """Every pair of drone segments that come closer than ``min_separation_m``.

    ``paths`` maps each drone ID to its setpoints, starting with its current
    position. Altitude is ignored: separation is measured horizontally.

    Brute force: every segment is checked against every segment of every
    other drone. With S segments in total that is O(S^2) checks, each O(1).
    """
    if min_separation_m <= 0:
        raise ValueError("min_separation_m must be positive")
    all_points = [point for setpoints in paths.values() for point in setpoints]
    if not all_points:
        return []
    # Local (east, north) meters around the centre of all setpoints.
    anchor = anchor_for_boundary(all_points)

    drones = list(paths) # List of the drone names
    # Converts each Coordinate into (east, north) meters
    # to_segments() pairs consecutive points into legs, result stored under drone name 
    segments = {
        drone: to_segments([coordinate_to_enu(anchor, point) for point in paths[drone]])
        for drone in drones
    }

    conflicts = []
    for i, drone_a in enumerate(drones):
        for drone_b in drones[i + 1:]:
            for index_a, (p1, p2) in enumerate(segments[drone_a]): # Every leg of one drone against every leg of the other 
                for index_b, (q1, q2) in enumerate(segments[drone_b]):
                    point_a, point_b = closest_points(p1, p2, q1, q2) # Calculate how close the two legs get
                    distance = math.dist(point_a, point_b)
                    if distance < min_separation_m:
                        # Hands back the conflicts & fills one PathConflict with both drones and leg #'s
                        # As well as closest spots to the distance, with enu_to_coordinate() converting spots
                        # Back to coordinates
                        conflicts.append(
                            PathConflict(
                                drone_a,
                                index_a,
                                drone_b,
                                index_b,
                                enu_to_coordinate(anchor, *point_a),
                                enu_to_coordinate(anchor, *point_b),
                                distance,
                            )
                        )
    return conflicts


def find_conflicts_sweep(
    paths: dict[str, list[Coordinate]], min_separation_m: float
) -> list[PathConflict]:
    """Same conflicts as ``find_conflicts``, using sweep and prune.

    Segments are swept left to right by their east (x) range, and each one is
    only compared with segments whose x and y ranges come within
    ``min_separation_m`` of it.

    Sorting is O(S log S). In the worst case, when every segment overlaps
    every other in x, it is still O(S^2) comparisons.
    """
    if min_separation_m <= 0:
        raise ValueError("min_separation_m must be positive")
    all_points = [point for setpoints in paths.values() for point in setpoints]
    if not all_points:
        return []
    anchor = anchor_for_boundary(all_points)

    # One entry per segment: its x and y ranges, which drone and leg it is,
    # and its two end points.
    entries = []
    for drone, setpoints in paths.items():
        points = [coordinate_to_enu(anchor, point) for point in setpoints]
        for index, (p1, p2) in enumerate(to_segments(points)):
            xs = (p1[0], p2[0])
            ys = (p1[1], p2[1])
            entries.append((min(xs), max(xs), min(ys), max(ys), drone, index, p1, p2))
    # Sweep left to right by where each segment starts.
    entries.sort(key=lambda entry: entry[0])

    conflicts = []
    active = []
    for entry in entries:
        min_x, max_x, min_y, max_y, drone, index, p1, p2 = entry

        # Prune: segments ending this far left can't reach this or any later segment.
        active = [other for other in active if other[1] + min_separation_m > min_x]

        for o_min_x, o_max_x, o_min_y, o_max_y, o_drone, o_index, q1, q2 in active:
            if o_drone == drone:
                continue  # a drone's own legs don't count
            # Skip if their y ranges are too far apart to be in conflict.
            if o_min_y - max_y >= min_separation_m or min_y - o_max_y >= min_separation_m:
                continue
            point_a, point_b = closest_points(q1, q2, p1, p2)
            distance = math.dist(point_a, point_b)
            if distance < min_separation_m:
                conflicts.append(
                    PathConflict(
                        o_drone,
                        o_index,
                        drone,
                        index,
                        enu_to_coordinate(anchor, *point_a),
                        enu_to_coordinate(anchor, *point_b),
                        distance,
                    )
                )
        active.append(entry)
    return conflicts
