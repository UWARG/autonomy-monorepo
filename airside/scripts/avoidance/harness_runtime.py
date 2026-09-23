"""Dependency-free runtime supervision for the issue-96 SITL harness."""

from __future__ import annotations

import json
import threading
import time
import traceback
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class WorkerFailure:
    """Diagnostic captured when a supervised worker cannot continue."""

    worker: str
    exception_type: str
    message: str
    traceback: str
    occurred_at_s: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class WorkerFailureError(RuntimeError):
    """Raised on the main thread for a background worker failure."""

    def __init__(self, failure: WorkerFailure) -> None:
        self.failure = failure
        super().__init__(
            f"worker {failure.worker} failed: "
            f"{failure.exception_type}: {failure.message}"
        )


class WorkerSupervisor:
    """Start workers, capture failures, and enforce progress deadlines."""

    def __init__(
        self,
        stop_event: threading.Event,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._stop_event = stop_event
        self._clock = clock
        self._lock = threading.Lock()
        self._threads: dict[str, threading.Thread] = {}
        self._last_progress_s: dict[str, float] = {}
        self._stale_after_s: dict[str, float | None] = {}
        self._failure: WorkerFailure | None = None

    def start(
        self,
        name: str,
        target: Callable[[], None],
        *,
        stale_after_s: float | None,
    ) -> threading.Thread:
        """Start one named worker and return its daemon thread."""

        if stale_after_s is not None and stale_after_s <= 0.0:
            raise ValueError("stale interval must be positive")
        with self._lock:
            if name in self._threads:
                raise ValueError(f"worker already registered: {name}")
            self._last_progress_s[name] = self._clock()
            self._stale_after_s[name] = stale_after_s

        def guarded_target() -> None:
            try:
                target()
            except Exception as exc:  # noqa: BLE001 - worker boundary
                self._record_exception(name, exc)
            else:
                if not self._stop_event.is_set():
                    self._record_message(
                        name,
                        "UnexpectedWorkerExit",
                        "worker returned before shutdown",
                        "",
                    )

        thread = threading.Thread(
            name=f"sitl-{name}",
            target=guarded_target,
            daemon=True,
        )
        with self._lock:
            self._threads[name] = thread
        thread.start()
        return thread

    def mark_progress(self, name: str) -> None:
        """Record one successful unit of work for a registered worker."""

        with self._lock:
            if name not in self._threads:
                raise KeyError(f"unknown worker: {name}")
            self._last_progress_s[name] = self._clock()

    def failure(self) -> WorkerFailure | None:
        with self._lock:
            return self._failure

    def check_health(self) -> None:
        """Raise immediately for a crash, exit, or stale worker."""

        with self._lock:
            failure = self._failure
            if failure is None and not self._stop_event.is_set():
                now_s = self._clock()
                for name, last_progress_s in self._last_progress_s.items():
                    stale_after_s = self._stale_after_s[name]
                    if (
                        stale_after_s is not None
                        and now_s - last_progress_s > stale_after_s
                    ):
                        failure = WorkerFailure(
                            worker=name,
                            exception_type="StaleWorker",
                            message=(
                                f"no progress for "
                                f"{now_s - last_progress_s:.2f}s "
                                f"(limit {stale_after_s:.2f}s)"
                            ),
                            traceback="",
                            occurred_at_s=now_s,
                        )
                        self._failure = failure
                        break
        if failure is not None:
            self._stop_event.set()
            raise WorkerFailureError(failure)

    def stop_and_join(self, timeout_s: float = 3.0) -> None:
        """Request shutdown and join every registered worker."""

        self._stop_event.set()
        with self._lock:
            threads = tuple(self._threads.values())
        deadline_s = self._clock() + timeout_s
        for thread in threads:
            thread.join(max(0.0, deadline_s - self._clock()))

    def _record_exception(self, name: str, exc: Exception) -> None:
        self._record_message(
            name,
            type(exc).__name__,
            str(exc),
            traceback.format_exc(),
        )

    def _record_message(
        self,
        name: str,
        exception_type: str,
        message: str,
        traceback_text: str,
    ) -> None:
        failure = WorkerFailure(
            worker=name,
            exception_type=exception_type,
            message=message,
            traceback=traceback_text,
            occurred_at_s=self._clock(),
        )
        with self._lock:
            if self._failure is None:
                self._failure = failure
        self._stop_event.set()


class ScenarioTimer:
    """Elapsed-time helper that never consults wall-clock time."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._started_at_s = clock()

    def elapsed_s(self) -> float:
        return max(0.0, self._clock() - self._started_at_s)

    def expired(self, duration_s: float) -> bool:
        return self.elapsed_s() >= duration_s


class SummaryEmitter:
    """Write and print exactly one machine-readable scenario summary."""

    def __init__(
        self,
        output_path: str | None,
        print_fn: Callable[..., None] = print,
    ) -> None:
        self._output_path = Path(output_path) if output_path else None
        self._print_fn = print_fn
        self._emitted = False

    def emit(self, summary: dict[str, Any]) -> None:
        if self._emitted:
            raise RuntimeError("scenario summary already emitted")
        self._emitted = True
        serialized = json.dumps(summary, sort_keys=True)
        if self._output_path is not None:
            self._output_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self._output_path.with_suffix(
                self._output_path.suffix + ".tmp"
            )
            temporary.write_text(serialized + "\n", encoding="utf-8")
            temporary.replace(self._output_path)
        self._print_fn(f"[demo] summary: {serialized}", flush=True)


def is_flight_controller_heartbeat(
    source_system: int,
    autopilot: int,
    ardupilot_autopilot_id: int,
) -> bool:
    """Accept only a non-zero ArduPilot flight-controller heartbeat."""

    return source_system > 0 and autopilot == ardupilot_autopilot_id


def has_callable_attribute(value: object, attribute: str) -> bool:
    """Return whether an object exposes the required callable capability."""

    return callable(getattr(value, attribute, None))
