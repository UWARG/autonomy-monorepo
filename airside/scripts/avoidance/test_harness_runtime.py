import json
import tempfile
import threading
import unittest
from pathlib import Path

from harness_runtime import (
    ScenarioTimer,
    StableConditionGate,
    SummaryEmitter,
    WorkerFailureError,
    WorkerSupervisor,
    has_callable_attribute,
    is_flight_controller_heartbeat,
)


class FakeClock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def __call__(self) -> float:
        return self.now_s


class HarnessRuntimeTests(unittest.TestCase):
    def test_stable_condition_gate_resets_on_manager_disappearance(self) -> None:
        clock = FakeClock()
        gate = StableConditionGate(2.0, clock=clock)

        self.assertFalse(gate.observe(True))
        clock.now_s = 1.9
        self.assertFalse(gate.observe(True))
        self.assertFalse(gate.observe(False))
        clock.now_s = 3.0
        self.assertFalse(gate.observe(True))
        clock.now_s = 5.0
        self.assertTrue(gate.observe(True))

    def test_only_nonzero_ardupilot_heartbeat_is_accepted(self) -> None:
        self.assertFalse(is_flight_controller_heartbeat(0, 3, 3))
        self.assertFalse(is_flight_controller_heartbeat(1, 8, 3))
        self.assertTrue(is_flight_controller_heartbeat(1, 3, 3))

    def test_sender_capability_must_be_callable(self) -> None:
        class MissingSender:
            obstacle_distance_send = None

        class ReadySender:
            def obstacle_distance_send(self) -> None:
                pass

        self.assertFalse(
            has_callable_attribute(MissingSender(), "obstacle_distance_send")
        )
        self.assertTrue(
            has_callable_attribute(ReadySender(), "obstacle_distance_send")
        )

    def test_worker_exception_propagates_with_traceback(self) -> None:
        stop = threading.Event()
        supervisor = WorkerSupervisor(stop)

        def fail() -> None:
            raise ValueError("boom")

        thread = supervisor.start("sender", fail, stale_after_s=None)
        thread.join(1.0)
        with self.assertRaises(WorkerFailureError) as context:
            supervisor.check_health()
        self.assertEqual(context.exception.failure.worker, "sender")
        self.assertEqual(context.exception.failure.exception_type, "ValueError")
        self.assertIn("ValueError: boom", context.exception.failure.traceback)

    def test_unexpected_worker_return_is_failure(self) -> None:
        stop = threading.Event()
        supervisor = WorkerSupervisor(stop)
        thread = supervisor.start("rx", lambda: None, stale_after_s=None)
        thread.join(1.0)
        with self.assertRaises(WorkerFailureError) as context:
            supervisor.check_health()
        self.assertEqual(
            context.exception.failure.exception_type,
            "UnexpectedWorkerExit",
        )

    def test_stale_worker_is_failure(self) -> None:
        clock = FakeClock()
        stop = threading.Event()
        release = threading.Event()
        supervisor = WorkerSupervisor(stop, clock=clock)

        def wait_for_shutdown() -> None:
            release.wait(1.0)

        supervisor.start("planner", wait_for_shutdown, stale_after_s=2.0)
        clock.now_s = 2.01
        with self.assertRaises(WorkerFailureError) as context:
            supervisor.check_health()
        self.assertEqual(context.exception.failure.exception_type, "StaleWorker")
        release.set()

    def test_shutdown_does_not_report_expected_worker_exit(self) -> None:
        stop = threading.Event()
        supervisor = WorkerSupervisor(stop)

        def wait_for_shutdown() -> None:
            while not stop.wait(0.01):
                supervisor.mark_progress("heartbeat")

        supervisor.start("heartbeat", wait_for_shutdown, stale_after_s=1.0)
        supervisor.stop_and_join()
        self.assertIsNone(supervisor.failure())

    def test_timer_ignores_wall_clock_changes(self) -> None:
        monotonic_clock = FakeClock()
        timer = ScenarioTimer(clock=monotonic_clock)
        wall_clock_s = 1_000.0
        wall_clock_s += 90_000.0
        monotonic_clock.now_s = 4.9
        self.assertFalse(timer.expired(5.0))
        wall_clock_s -= 180_000.0
        monotonic_clock.now_s = 5.0
        self.assertTrue(timer.expired(5.0))
        self.assertNotEqual(wall_clock_s, monotonic_clock.now_s)

    def test_summary_is_written_and_emitted_exactly_once(self) -> None:
        output: list[str] = []

        def capture(value: str, *, flush: bool) -> None:
            self.assertTrue(flush)
            output.append(value)

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "summary.json"
            emitter = SummaryEmitter(str(path), print_fn=capture)
            emitter.emit({"scenario": "wall_custom_2d", "verdict": "FAIL"})
            with self.assertRaisesRegex(RuntimeError, "already emitted"):
                emitter.emit({"verdict": "PASS"})
            self.assertEqual(len(output), 1)
            self.assertEqual(json.loads(path.read_text()), {
                "scenario": "wall_custom_2d",
                "verdict": "FAIL",
            })


if __name__ == "__main__":
    unittest.main()
