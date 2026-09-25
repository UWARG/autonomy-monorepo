from __future__ import annotations

import unittest
from unittest.mock import Mock

import py_trees
from engine.behaviors.navigation.obstacle_aware_fly_to_waypoint import (
    ObstacleAwareFlyToWaypoint,
)
from engine.obstacle_navigation import NavigationDecision
from mavros_msgs.msg import State


class ObstacleAwareBehaviorLifecycleTests(unittest.TestCase):
    def test_inactive_timer_does_not_touch_controller_or_publish(self) -> None:
        behavior = ObstacleAwareFlyToWaypoint()
        behavior._active = False
        behavior._waypoint = object()
        behavior._controller = Mock()
        behavior._publish_velocity = Mock()

        behavior._control_cycle()

        behavior._controller.step.assert_not_called()
        behavior._publish_velocity.assert_not_called()

    def test_terminate_in_guided_sends_zero_then_releases_owner(self) -> None:
        behavior = ObstacleAwareFlyToWaypoint()
        behavior._active = True
        behavior._latest_state = State(armed=True, mode="GUIDED")
        behavior._controller = Mock()
        behavior._publish_velocity = Mock()

        behavior.terminate(py_trees.common.Status.SUCCESS)

        behavior._publish_velocity.assert_called_once_with(0.0, 0.0, 0.0)
        behavior._controller.planner.reset.assert_called_once_with()
        self.assertFalse(behavior._active)

    def test_terminate_after_pilot_takeover_publishes_nothing(self) -> None:
        behavior = ObstacleAwareFlyToWaypoint()
        behavior._active = True
        behavior._latest_state = State(armed=True, mode="LOITER")
        behavior._controller = Mock()
        behavior._publish_velocity = Mock()

        behavior.terminate(py_trees.common.Status.INVALID)

        behavior._publish_velocity.assert_not_called()
        self.assertFalse(behavior._active)

    def test_control_cycle_after_pilot_takeover_releases_without_publish(self) -> None:
        behavior = ObstacleAwareFlyToWaypoint()
        behavior._active = True
        behavior._waypoint = object()
        behavior._timed_out = False
        behavior._latest_state = State(armed=True, mode="LOITER")
        behavior._controller = Mock()
        behavior._controller.release.return_value = NavigationDecision(
            should_publish=False,
            east_mps=0.0,
            north_mps=0.0,
            up_mps=0.0,
            planner_status="RELEASED",
            reason="PILOT_CONTROL",
            minimum_clearance_m=None,
            goal_distance_m=None,
            path_found_count=0,
            hold_count=0,
            valid_navigation_cycle=False,
            goal_reached=False,
        )
        behavior._navigation_clock = Mock()
        behavior._navigation_clock.advance.return_value = 0.0
        behavior._publish_velocity = Mock()
        behavior._publish_diagnostics = Mock()

        behavior._control_cycle()

        behavior._controller.release.assert_called_once_with("PILOT_CONTROL")
        behavior._controller.step.assert_not_called()
        behavior._publish_velocity.assert_not_called()

    def test_timed_out_cycle_cannot_resume_motion(self) -> None:
        behavior = ObstacleAwareFlyToWaypoint()
        behavior._active = True
        behavior._waypoint = object()
        behavior._timed_out = True
        behavior._latest_state = State(armed=True, mode="GUIDED")
        behavior._controller = Mock()
        behavior._latest_decision = Mock()
        behavior._publish_velocity = Mock()
        behavior._publish_diagnostics = Mock()

        behavior._control_cycle()

        behavior._controller.step.assert_not_called()
        behavior._publish_velocity.assert_called_once_with(0.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
