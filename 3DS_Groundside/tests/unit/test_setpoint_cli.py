"""Unit tests for src.setpoint_tester.cli."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any, Callable

import pytest

from src.setpoint_tester.cli import (
    HELP,
    HELP_TEXT,
    QUIT,
    UsageError,
    main,
    parse_line,
    run,
)
from src.setpoint_tester.client import LinkError
from src.setpoint_tester.protocol import Reply, go_home, setpoint

OK = Reply(ok=True, message="published")


@pytest.mark.parametrize(
    "text, expected",
    [
        ("goto 43.4339 -80.5776 10", setpoint(43.4339, -80.5776, 10.0)),
        ("goto 43.4339, -80.5776, 10", setpoint(43.4339, -80.5776, 10.0)),
        ("goto 43.4339,-80.5776,10", setpoint(43.4339, -80.5776, 10.0)),
        ("  GOTO 43 -80 10  ", setpoint(43.0, -80.0, 10.0)),
        ("setpoint 43 -80 10", setpoint(43.0, -80.0, 10.0)),
        ("home", go_home()),
        ("go_home", go_home()),
        ("help", HELP),
        ("?", HELP),
        ("quit", QUIT),
        ("exit", QUIT),
        ("", None),
        ("   ", None),
    ],
)
def test_parse_line(text: str, expected: Any) -> None:
    assert parse_line(text) == expected


@pytest.mark.parametrize(
    "text",
    [
        "goto",
        "goto 43.4 -80.5",
        "goto 43.4 -80.5 10 5",
        "goto north -80.5 10",
        "goto 43.4 -80.5 nan",
        "goto 91 -80.5 10",
        "home now",
        "takeoff",
    ],
)
def test_parse_line_rejects(text: str) -> None:
    with pytest.raises(UsageError):
        parse_line(text)


class FakeClient:
    address = "drone:7070"

    def __init__(self, replies: Iterable[Reply | Exception] = (), connect_error: bool = False):
        self.sent: list[dict[str, Any]] = []
        self._replies = list(replies)
        self._connect_error = connect_error

    def connect(self) -> None:
        if self._connect_error:
            raise LinkError("cannot connect to drone:7070: refused")

    def send(self, command: dict[str, Any]) -> Reply:
        self.sent.append(command)
        reply = self._replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return reply


def scripted(lines: Iterable[str]) -> Callable[[str], str]:
    """An input() replacement that types `lines`, then Ctrl-D."""
    remaining = iter(lines)

    def read_line(prompt: str) -> str:
        try:
            return next(remaining)
        except StopIteration:
            raise EOFError from None

    return read_line


def test_run_sends_commands_until_quit() -> None:
    client = FakeClient(replies=[OK, Reply(ok=False, message="rejected: too low")])
    output: list[str] = []

    run(
        client,  # type: ignore[arg-type]
        scripted(["goto 43.4, -80.5, 10", "", "bogus", "help", "home", "quit", "home"]),
        output.append,
    )

    assert client.sent == [setpoint(43.4, -80.5, 10.0), go_home()]
    assert "Connected." in output
    assert "-> setpoint lat=43.4 lon=-80.5 alt=10.0m" in output
    assert "OK: published" in output
    assert "error: unknown command 'bogus'; type 'help'" in output
    assert HELP_TEXT in output
    assert "-> go home" in output
    assert "REJECTED: rejected: too low" in output


def test_run_reports_link_errors_and_continues() -> None:
    client = FakeClient(replies=[LinkError("no reply within 3s"), OK], connect_error=True)
    output: list[str] = []

    run(client, scripted(["home", "home"]), output.append)  # type: ignore[arg-type]

    assert output[1].startswith("Not connected: cannot connect")
    assert "FAILED: no reply within 3s" in output
    assert output[-2:] == ["OK: published", ""]
    assert len(client.sent) == 2


@pytest.mark.parametrize("argv", [["--port", "0"], ["--port", "70000"], ["--timeout", "0"]])
def test_main_rejects_bad_arguments(argv: list[str]) -> None:
    with pytest.raises(SystemExit):
        main(argv)
