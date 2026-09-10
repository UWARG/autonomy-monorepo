import math

from obstacle_avoidance import Point2D, SectorScan, sector_scan_to_snapshot


def test_sector_scan_projects_returns_into_world_frame() -> None:
    scan = SectorScan(
        ranges_m=(2.0, None, 4.0),
        angle_offset_rad=-math.pi / 2.0,
        angle_increment_rad=math.pi / 2.0,
        timestamp_s=12.0,
    )
    snapshot = sector_scan_to_snapshot(
        scan,
        sensor_position=Point2D(10.0, 5.0),
        sensor_heading_rad=math.pi / 2.0,
        obstacle_radius_m=0.4,
    )
    assert snapshot.healthy
    assert len(snapshot.obstacles) == 2
    assert snapshot.obstacles[0].center == Point2D(12.0, 5.0)
    assert snapshot.obstacles[1].center.x == 6.0
    assert math.isclose(snapshot.obstacles[1].center.y, 5.0, abs_tol=1e-9)


def test_invalid_sector_marks_snapshot_unhealthy() -> None:
    scan = SectorScan(
        ranges_m=(0.0, float("nan")),
        angle_offset_rad=0.0,
        angle_increment_rad=0.1,
        timestamp_s=1.0,
    )
    snapshot = sector_scan_to_snapshot(scan, Point2D(0.0, 0.0), 0.0, 0.5)
    assert not snapshot.healthy
    assert snapshot.obstacles == ()
