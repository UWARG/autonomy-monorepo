import pytest

from deconfliction import on_segment, orientation, segments_intersect, intersection


def test_orientation_left():
    assert orientation((0, 0), (10, 10), (0, 10)) > 0


def test_orientation_right():
    assert orientation((0, 0), (-10, 10), (0, 10)) < 0


def test_orientation_colinear():
    assert orientation((0, 0), (10, 10), (5, 5)) == 0


def test_on_segment_middle():
    assert on_segment((5, 5), (0, 0), (10, 10))


def test_on_segment_endpoint():
    assert on_segment((10, 10), (0, 0), (10, 10))


def test_on_segment_past_end():
    assert not on_segment((20, 20), (0, 0), (10, 10))
    assert not on_segment((-5, -5), (0, 0), (10, 10))


def test_on_segment_vertical_boundary():
    assert on_segment((10, 5), (10, 0), (10, 20))
    assert not on_segment((10, 25), (10, 0), (10, 20))

@pytest.mark.parametrize(
    ("p1", "p2", "q1", "q2", "expected"),
    [
        ((0, 0), (10, 10), (0, 10), (10, 0), True),     # X crossing
        ((0, 0), (10, 0), (0, 5), (10, 5), False),      # parallel
        ((0, 0), (10, 0), (5, 0), (5, 10), True),       # T touch
        ((0, 0), (5, 0), (5, 0), (5, 10), True),        # end to end
        ((10, 0), (10, 20), (10, 5), (10, 30), True),   # overlap on shared boundary
        ((0, 0), (5, 0), (7, 0), (10, 0), False),       # same line, gap
        ((0, 0), (1, 1), (5, 5), (6, 8), False),        # far apart
    ],
    ids=["x", "parallel", "t", "end_to_end", "overlap", "collinear_gap", "far_apart"],
)
def test_segments_intersect(p1, p2, q1, q2, expected):
    assert segments_intersect(p1, p2, q1, q2) is expected
    # Swapping the segments must not change the answer.
    assert segments_intersect(q1, q2, p1, p2) is expected

@pytest.mark.parametrize(
    ("p1", "p2", "q1", "q2", "expected"),
    [
        ((0, 0), (10, 10), (0, 10), (10, 0), ((5, 5), (5, 5))),        # X crossing
        ((0, 0), (10, 0), (5, 0), (5, 10), ((5, 0), (5, 0))),          # T touch
        ((0, 0), (5, 0), (5, 0), (5, 10), ((5, 0), (5, 0))),           # end to end
        ((10, 0), (10, 20), (10, 5), (10, 30), ((10, 5), (10, 20))),   # overlap
        ((0, 0), (10, 0), (0, 5), (10, 5), None),                      # parallel
    ],
    ids=["x", "t", "end_to_end", "overlap", "parallel"],
)
def test_intersection(p1, p2, q1, q2, expected):
    assert intersection(p1, p2, q1, q2) == expected
    # Swapping the segments must give the same place.
    assert intersection(q1, q2, p1, p2) == expected