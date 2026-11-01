"""Tests for source-stamped SITL speed measurements."""

from pose_speed import horizontal_speed_mps


def test_uses_pose_stamp_interval_not_callback_arrival_interval() -> None:
    assert horizontal_speed_mps((0.0, 0.0, 100.0), (0.7, 0.0, 101.0)) == 0.7


def test_rejects_duplicate_or_missing_stamps() -> None:
    assert horizontal_speed_mps((0.0, 0.0, 100.0), (1.0, 0.0, 100.0)) is None
    assert horizontal_speed_mps((0.0, 0.0, 0.0), (1.0, 0.0, 101.0)) is None


def test_long_measurement_gap_uses_whole_gap() -> None:
    assert horizontal_speed_mps((0.0, 0.0, 100.0), (7.0, 0.0, 110.0)) == 0.7
