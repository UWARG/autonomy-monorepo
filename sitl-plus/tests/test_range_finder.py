"""Tests for the Range_Finder class."""

from range_finder import Range_Finder


def test_range_finder_init(_bullet_connect):
    """Range finder initialization stores the expected configuration."""
    range_finder_obj = Range_Finder(port=6004, direction=[0, 0, -1], dist=100)
    assert range_finder_obj.port == 6004
    assert range_finder_obj.direction == [0, 0, -1]
    assert range_finder_obj.dist == 100


def test_range_finder_update(_bullet_connect, range_finder_obj):
    """Updating the range finder produces a distance reading."""
    range_finder_obj.update()
    assert range_finder_obj.range is not None
