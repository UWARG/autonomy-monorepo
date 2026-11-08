from __future__ import annotations

Point = tuple[float, float]

def orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    ax, ay = a
    bx, by = b
    cx, cy = c
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def on_segment(
        p:tuple[float, float], a: tuple[float, float], b: tuple[float, float]
) -> bool:
    # True only if p lies within the box spanned by a and b & meaningful when its already colinear with a and b 
    px, py = p
    ax, ay = a
    bx, by = b
    return min(ax, bx) <= px <= max(ax, bx) and min(ay, by) <= py <= max(ay, by)


def segments_intersect(
        p1:tuple[float, float], p2:tuple[float, float], q1:tuple[float, float], q2:tuple[float, float]
) -> bool:
    # True if segment p1-p2 and segment q1-q2 touch or cross
    # Touching an endpoint or overlapping along same line also count

    d1 = orientation(q1, q2, p1)
    d2 = orientation(q1, q2, p2)
    d3 = orientation(p1, p2, q1)
    d4 = orientation(p1, p2, q2)

    # Each segment's ends are on opposite sides of the other segment
    if d1 * d2 < 0 and d3 * d4 < 0:
        return True

    # An end lies on the other segment's line: check it is within the segment
    if d1 == 0 and on_segment(p1, q1, q2):
        return True
    if d2 == 0 and on_segment(p2, q1, q2):
        return True
    if d3 == 0 and on_segment(q1, p1, p2):
        return True
    if d4 == 0 and on_segment(q2, p1, p2):
        return True

    return False

def intersection(p1: Point, p2: Point, q1: Point, q2: Point) -> tuple[Point, Point] | None:
    """Segment p1-p2 meets segment q1-q2, as (start, end)
    A crossing or touch returns the same point twice. 
    An overlap along the shared line returns both ends of the shared point.
    Returns none if the segments do not meet"""

    if not segments_intersect(p1, p2, q1, q2):
        return None

    d1 = orientation(q1, q2, p1)
    d2 = orientation(q1, q2, p2)
    if d1 != d2:
        # The side value changes evenly along P from d1 to d2, so it reaches
        # 0 (Q's line) a fraction t of the way from p1 to p2.
        t = d1 / (d1 - d2)
        point = (p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))
        return point, point

    # Both segments lie on one line: the shared part runs between the ends
    # that lie on the other segment.
    shared = [p for p in (p1, p2) if on_segment(p, q1, q2)]
    shared += [q for q in (q1, q2) if on_segment(q, p1, p2)]
    return min(shared), max(shared)