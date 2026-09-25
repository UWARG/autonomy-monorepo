from __future__ import annotations

import math
import unittest

from engine.obstacle_navigation import (
    ActiveNavigationClock,
    NavigationGoal,
    NavigationTelemetry,
    ObstacleAwareController,
    ObstacleNavigationConfig,
    prepare_sector_scan,
)
from obstacle_avoidance import CircleObstacle, ObstacleSnapshot, Point2D

NOW = 100.0
LATITUDE = 43.0
LONGITUDE = -80.0


def telemetry(**overrides: object) -> NavigationTelemetry:
    values: dict[str, object] = {
        "east_m": 0.0,
        "north_m": 0.0,
        "yaw_enu_rad": 0.0,
        "latitude": LATITUDE,
        "longitude": LONGITUDE,
        "relative_altitude_m": 10.0,
        "armed": True,
        "mode": "GUIDED",
        "pose_received_s": NOW,
        "fix_received_s": NOW,
        "altitude_received_s": NOW,
        "state_received_s": NOW,
        "fresh_after_guided_entry": True,
    }
    values.update(overrides)
    return NavigationTelemetry(**values)  # type: ignore[arg-type]


def goal(
    *, east_m: float = 0.0, north_m: float = 10.0, altitude_m: float = 10.0
) -> NavigationGoal:
    return NavigationGoal(east_m, north_m, altitude_m)


def clear_snapshot(
    *, healthy: bool = True, timestamp_s: float = NOW
) -> ObstacleSnapshot:
    return ObstacleSnapshot(obstacles=(), timestamp_s=timestamp_s, healthy=healthy)


class ScanConversionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ObstacleNavigationConfig()

    def convert(self, ranges: tuple[float, ...], **overrides: object):
        values: dict[str, object] = {
            "ranges_m": ranges,
            "angle_min_rad": -math.pi,
            "angle_increment_rad": 2.0 * math.pi / len(ranges),
            "range_min_m": 0.2,
            "range_max_m": 20.0,
            "frame_id": "base_link",
            "source_stamp_s": 50.0,
            "previous_source_stamp_s": 49.9,
            "now_ros_s": 50.0,
            "received_monotonic_s": NOW,
            "config": self.config,
        }
        values.update(overrides)
        return prepare_sector_scan(**values)  # type: ignore[arg-type]

    def test_positive_infinity_is_explicit_clear(self) -> None:
        converted = self.convert((math.inf,) * 8)
        self.assertIsNone(converted.reason)
        self.assertTrue(converted.scan.healthy)
        self.assertEqual(converted.scan.ranges_m, (None,) * 8)

    def test_finite_ranges_are_preserved(self) -> None:
        converted = self.convert((1.0, 2.0, 3.0, 4.0))
        self.assertIsNone(converted.reason)
        self.assertEqual(converted.scan.ranges_m, (1.0, 2.0, 3.0, 4.0))

    def test_nan_and_negative_infinity_fail_closed(self) -> None:
        for invalid in (math.nan, -math.inf):
            with self.subTest(invalid=invalid):
                converted = self.convert((math.inf, invalid, math.inf, math.inf))
                self.assertEqual(converted.reason, "INVALID_SCAN_RANGE")
                self.assertFalse(converted.scan.healthy)

    def test_wrong_frame_and_partial_coverage_fail_closed(self) -> None:
        wrong_frame = self.convert((math.inf,) * 8, frame_id="camera")
        partial = self.convert((math.inf,) * 8, angle_increment_rad=math.pi / 8.0)
        self.assertEqual(wrong_frame.reason, "INVALID_SCAN_FRAME")
        self.assertEqual(partial.reason, "INCOMPLETE_SCAN_COVERAGE")

    def test_stale_future_and_frozen_stamps_fail_closed(self) -> None:
        stale = self.convert(
            (math.inf,) * 8,
            source_stamp_s=49.0,
            previous_source_stamp_s=48.0,
        )
        future = self.convert(
            (math.inf,) * 8,
            source_stamp_s=51.0,
            previous_source_stamp_s=50.0,
        )
        frozen = self.convert((math.inf,) * 8, source_stamp_s=49.9)
        self.assertEqual(stale.reason, "STALE_SCAN")
        self.assertEqual(future.reason, "FUTURE_SCAN")
        self.assertEqual(frozen.reason, "FROZEN_SCAN")


class ControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = ObstacleAwareController()

    def test_clear_path_uses_enu_north_sign(self) -> None:
        decision = self.controller.step(
            now_s=NOW,
            goal=goal(north_m=10.0),
            telemetry=telemetry(),
            obstacles=clear_snapshot(),
        )
        self.assertTrue(decision.should_publish)
        self.assertAlmostEqual(decision.east_mps, 0.0, places=5)
        self.assertGreater(decision.north_mps, 0.0)
        self.assertLessEqual(decision.north_mps, 2.0)
        self.assertEqual(decision.planner_status, "PATH_FOUND")

    def test_east_goal_uses_positive_enu_x(self) -> None:
        decision = self.controller.step(
            now_s=NOW,
            goal=goal(east_m=10.0, north_m=0.0),
            telemetry=telemetry(),
            obstacles=clear_snapshot(),
        )
        self.assertGreater(decision.east_mps, 0.0)
        self.assertAlmostEqual(decision.north_mps, 0.0, places=5)

    def test_vertical_velocity_tracks_relative_altitude(self) -> None:
        climb = self.controller.step(
            now_s=NOW,
            goal=goal(altitude_m=13.0),
            telemetry=telemetry(),
            obstacles=clear_snapshot(),
        )
        self.assertEqual(climb.up_mps, 1.0)
        self.controller.reset()
        descend = self.controller.step(
            now_s=NOW,
            goal=goal(altitude_m=7.0),
            telemetry=telemetry(),
            obstacles=clear_snapshot(),
        )
        self.assertEqual(descend.up_mps, -1.0)

    def test_blocked_or_unhealthy_map_holds_all_axes(self) -> None:
        blocked = ObstacleSnapshot(
            obstacles=(CircleObstacle(Point2D(0.0, 0.0), 20.0),),
            timestamp_s=NOW,
        )
        for snapshot in (blocked, clear_snapshot(healthy=False)):
            with self.subTest(snapshot=snapshot):
                self.controller.reset()
                decision = self.controller.step(
                    now_s=NOW,
                    goal=goal(),
                    telemetry=telemetry(),
                    obstacles=snapshot,
                )
                self.assertTrue(decision.should_publish)
                self.assertEqual(
                    (decision.east_mps, decision.north_mps, decision.up_mps),
                    (0.0, 0.0, 0.0),
                )
                self.assertEqual(decision.planner_status, "NO_PATH")

    def test_stale_telemetry_and_waiting_after_guided_entry_hold(self) -> None:
        stale = self.controller.step(
            now_s=NOW,
            goal=goal(),
            telemetry=telemetry(pose_received_s=NOW - 2.0),
            obstacles=clear_snapshot(),
        )
        self.assertEqual(stale.reason, "STALE_TELEMETRY")
        waiting = self.controller.step(
            now_s=NOW,
            goal=goal(),
            telemetry=telemetry(fresh_after_guided_entry=False),
            obstacles=clear_snapshot(),
        )
        self.assertEqual(waiting.reason, "WAITING_FOR_FRESH_DATA")
        self.assertEqual(waiting.north_mps, 0.0)

    def test_pilot_takeover_and_disarm_release_command_lane(self) -> None:
        for state in (telemetry(mode="LOITER"), telemetry(armed=False)):
            with self.subTest(state=state):
                decision = self.controller.step(
                    now_s=NOW,
                    goal=goal(),
                    telemetry=state,
                    obstacles=clear_snapshot(),
                )
                self.assertFalse(decision.should_publish)
                self.assertEqual(decision.planner_status, "RELEASED")

    def test_goal_requires_valid_map_then_reports_reached(self) -> None:
        at_goal = goal(north_m=0.0, altitude_m=10.0)
        no_scan = self.controller.step(
            now_s=NOW,
            goal=at_goal,
            telemetry=telemetry(),
            obstacles=None,
        )
        self.assertFalse(no_scan.goal_reached)
        reached = self.controller.step(
            now_s=NOW,
            goal=at_goal,
            telemetry=telemetry(),
            obstacles=clear_snapshot(),
        )
        self.assertTrue(reached.goal_reached)
        self.assertEqual(
            (reached.east_mps, reached.north_mps, reached.up_mps),
            (0.0, 0.0, 0.0),
        )


class ActiveNavigationClockTests(unittest.TestCase):
    def test_holds_and_backward_clock_steps_do_not_consume_timeout(self) -> None:
        clock = ActiveNavigationClock()
        clock.reset(10.0)
        self.assertEqual(clock.advance(11.0, count=True), 1.0)
        self.assertEqual(clock.advance(20.0, count=False), 1.0)
        self.assertEqual(clock.advance(19.0, count=True), 1.0)
        self.assertEqual(clock.advance(21.0, count=True), 3.0)


if __name__ == "__main__":
    unittest.main()
