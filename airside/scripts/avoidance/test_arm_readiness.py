import unittest

from arm_readiness import (
    ArmReadinessSnapshot,
    StableArmReadinessGate,
    missing_arm_preconditions,
)


def ready_snapshot(**overrides: object) -> ArmReadinessSnapshot:
    values: dict[str, object] = {
        "gps_fix": 3,
        "ekf_using_gps": True,
        "home_position_seen": True,
        "local_position_seen": True,
        "prearm_ok": True,
    }
    values.update(overrides)
    return ArmReadinessSnapshot(**values)  # type: ignore[arg-type]


class ArmReadinessTests(unittest.TestCase):
    def test_prearm_alone_does_not_make_vehicle_ready(self) -> None:
        snapshot = ready_snapshot(
            gps_fix=0,
            ekf_using_gps=False,
            home_position_seen=False,
            local_position_seen=False,
        )
        self.assertEqual(
            missing_arm_preconditions(snapshot),
            (
                "gps_3d_fix",
                "ekf_using_gps",
                "global_home_position",
                "local_position",
            ),
        )

    def test_gps_ekf_and_home_are_not_sufficient(self) -> None:
        snapshot = ready_snapshot(local_position_seen=False, prearm_ok=False)
        self.assertEqual(
            missing_arm_preconditions(snapshot),
            ("local_position", "prearm_check"),
        )

    def test_complete_snapshot_has_no_missing_preconditions(self) -> None:
        self.assertEqual(missing_arm_preconditions(ready_snapshot()), ())

    def test_gate_requires_complete_stable_interval(self) -> None:
        gate = StableArmReadinessGate(stable_s=5.0)
        snapshot = ready_snapshot()
        self.assertFalse(gate.update(10.0, snapshot))
        self.assertFalse(gate.update(14.9, snapshot))
        self.assertTrue(gate.update(15.0, snapshot))

    def test_missing_condition_resets_stable_interval(self) -> None:
        gate = StableArmReadinessGate(stable_s=5.0)
        self.assertFalse(gate.update(0.0, ready_snapshot()))
        self.assertFalse(gate.update(4.0, ready_snapshot()))
        self.assertFalse(
            gate.update(4.5, ready_snapshot(local_position_seen=False))
        )
        self.assertFalse(gate.update(5.0, ready_snapshot()))
        self.assertTrue(gate.update(10.0, ready_snapshot()))


if __name__ == "__main__":
    unittest.main()
