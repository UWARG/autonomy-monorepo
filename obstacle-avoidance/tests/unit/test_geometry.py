import math

import pytest

from obstacle_avoidance.geometry import (
    angular_distance,
    point_segment_distance,
    wrap_angle,
)
from obstacle_avoidance.models import Point2D


def test_point_segment_distance_uses_clamped_projection() -> None:
    start = Point2D(0.0, 0.0)
    end = Point2D(4.0, 0.0)
    assert point_segment_distance(Point2D(2.0, 3.0), start, end) == 3.0
    assert point_segment_distance(Point2D(6.0, 0.0), start, end) == 2.0


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(0.0, 0.0), (math.pi, -math.pi), (3.0 * math.pi, -math.pi)],
)
def test_wrap_angle(angle: float, expected: float) -> None:
    assert wrap_angle(angle) == pytest.approx(expected)


def test_angular_distance_crosses_wrap_boundary() -> None:
    assert angular_distance(math.radians(179), math.radians(-179)) == pytest.approx(
        math.radians(2)
    )
