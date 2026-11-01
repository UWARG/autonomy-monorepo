import unittest

from qualify import control_accepted, custom_accepted


def passing_custom_summary(**overrides: object) -> dict[str, object]:
    summary: dict[str, object] = {
        "verdict": "PASS",
        "armed": True,
        "failure_reason": None,
        "worker_failure": None,
        "breached": False,
        "clearance_ok": True,
        "goal_reached_at_s": 24.0,
        "distance_sensor_rx": 1,
        "obstacle_tx_count": 1,
        "planner_path_found_count": 1,
        "planner_hold_count": 0,
        "planner_status": "PATH_FOUND",
    }
    summary.update(overrides)
    return summary


class QualificationTests(unittest.TestCase):
    def test_custom_requires_every_strict_condition(self) -> None:
        accepted, note = custom_accepted(0, passing_custom_summary())
        self.assertTrue(accepted)
        self.assertEqual(note, "ok")

        accepted, note = custom_accepted(
            0,
            passing_custom_summary(planner_hold_count=1),
        )
        self.assertFalse(accepted)
        self.assertIn("no_hold", note)

    def test_missing_custom_summary_is_failure(self) -> None:
        self.assertEqual(custom_accepted(1, None), (False, "missing summary"))

    def test_wall_guided_is_an_expected_negative_control(self) -> None:
        accepted, _ = control_accepted(
            "wall_guided",
            1,
            {"verdict": "FAIL", "breached": True},
        )
        self.assertTrue(accepted)
        accepted, _ = control_accepted(
            "wall_guided",
            0,
            {"verdict": "PASS", "breached": False},
        )
        self.assertFalse(accepted)

    def test_guided_velocity_limitation_is_not_a_control_failure(self) -> None:
        accepted, note = control_accepted(
            "wall_guided_vel",
            0,
            {
                "verdict": "PASS",
                "breached": False,
                "clearance_ok": False,
                "goal_reached_at_s": None,
            },
        )
        self.assertTrue(accepted)
        self.assertEqual(note, "stop-only limitation")


if __name__ == "__main__":
    unittest.main()
