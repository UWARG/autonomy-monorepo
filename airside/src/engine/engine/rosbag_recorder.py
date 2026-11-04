"""Own one rosbag subprocess and finalize it before reporting a successful stop."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4


@dataclass(frozen=True)
class RecordingResult:
    success: bool
    message: str


class RosbagRecorder:
    def __init__(
        self,
        output_directory: str | Path,
        topics: Sequence[str] = (),
        command: Sequence[str] = ("ros2", "bag", "record"),
        startup_timeout: float = 5.0,
        stop_timeout: float = 10.0,
    ):
        self.output_directory = Path(output_directory).expanduser().resolve()
        self.topics = tuple(topics)
        self.command = tuple(command)
        self.startup_timeout = startup_timeout
        self.stop_timeout = stop_timeout
        self._process: subprocess.Popen | None = None
        self._log = None
        self._log_path: Path | None = None
        self._bag_path: Path | None = None
        self._error: str | None = None

    def status(self) -> dict:
        if self._process is not None and self._process.poll() is not None:
            code = self._process.returncode
            self._release()
            self._error = f"Recorder exited unexpectedly (exit {code}). {self._log_tail()}"
        return {
            "recording": self._process is not None,
            "bag_path": str(self._bag_path) if self._bag_path else None,
            "error": self._error,
        }

    def set_recording(self, active: bool) -> RecordingResult:
        self.status()
        if active:
            return self._start()
        return self._stop()

    def _start(self) -> RecordingResult:
        if self._process is not None:
            return RecordingResult(True, "Recording is already active.")
        self._error = None
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        name = f"bag_{stamp}_{uuid4().hex[:8]}"
        self._bag_path = self.output_directory / name
        self._log_path = self.output_directory / f"{name}.log"
        try:
            self.output_directory.mkdir(parents=True, exist_ok=True)
            self._log = self._log_path.open("wb")
            arguments = [
                *self.command, "--output", str(self._bag_path), "--storage", "sqlite3",
                *(self.topics or ("--all",)),
            ]
            self._process = subprocess.Popen(
                arguments, stdin=subprocess.DEVNULL, stdout=self._log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
        except OSError as error:
            self._release()
            self._error = f"Could not start recording: {error}"
            return RecordingResult(False, self._error)

        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            if not self.status()["recording"]:
                return RecordingResult(False, self._error or "Recorder failed to start.")
            # sqlite storage creates a database once the writer has opened the bag.
            if any(self._bag_path.glob("*.db3")):
                return RecordingResult(True, f"Recording to {self._bag_path}")
            time.sleep(.02)

        self.close()
        self._error = f"Recorder did not open a bag before the startup timeout. {self._log_tail()}"
        return RecordingResult(False, self._error)

    def _stop(self) -> RecordingResult:
        if self._process is None:
            return RecordingResult(True, "Recording is already stopped.")
        try:
            os.killpg(self._process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            code = self._process.wait(timeout=self.stop_timeout)
        except subprocess.TimeoutExpired:
            self._error = "Stopping timed out; recorder is still active. Retry Stop Recording."
            return RecordingResult(False, self._error)
        self._release()
        finalized = self._bag_path is not None and (self._bag_path / "metadata.yaml").is_file()
        if code not in (0, -signal.SIGINT, 128 + signal.SIGINT) or not finalized:
            self._error = f"Recorder stopped, but the bag may be incomplete (exit {code}). {self._log_tail()}"
            return RecordingResult(False, self._error)
        self._error = None
        return RecordingResult(True, f"Saved recording to {self._bag_path}")

    def close(self) -> None:
        """Gracefully stop on node shutdown; forcibly reap an unresponsive child."""
        self.status()
        if self._process is None:
            return
        result = self._stop()
        if self._process is None:
            return
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(self._process.pid, sig)
            except ProcessLookupError:
                pass
            try:
                self._process.wait(timeout=self.stop_timeout)
                break
            except subprocess.TimeoutExpired:
                continue
        self._release()
        self._error = f"Recorder was forcibly stopped during shutdown; bag may be incomplete. {result.message}"

    def _release(self) -> None:
        self._process = None
        if self._log is not None:
            self._log.close()
            self._log = None

    def _log_tail(self) -> str:
        if self._log_path is None:
            return ""
        try:
            with self._log_path.open("rb") as log:
                log.seek(0, os.SEEK_END)
                log.seek(max(0, log.tell() - 2000))
                return log.read().decode("utf-8", errors="replace").strip()
        except OSError:
            return ""
