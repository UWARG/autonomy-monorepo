"""Interactive prompt for commanding a drone's setpoints during flight tests.

uv run python -m src.setpoint_tester --host <drone-ip>
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from typing import Any, Callable, Union

from src.setpoint_tester.client import LinkError, SetpointClient
from src.setpoint_tester.protocol import DEFAULT_PORT, go_home, setpoint

HELP_TEXT = """\
Commands:
  goto LAT LON ALT   fly to a point and photograph it (ALT: meters above launch)
  home               return to the launch point and land (ends the mission)
  help               show this help
  quit               exit (also Ctrl-D)
Commas are ignored, so coordinates can be pasted from a map:
  goto 43.4339, -80.5776, 10"""

# Non-command results of parse_line.
HELP = "help"
QUIT = "quit"

Action = Union[dict[str, Any], str, None]


class UsageError(ValueError):
    """The operator typed something that is not a valid command."""


def parse_line(text: str) -> Action:
    """Turn one typed line into a command to send, HELP, QUIT, or None if blank.

    Raises UsageError with a message for the operator.
    """
    words = text.replace(",", " ").split()
    if not words:
        return None
    name, args = words[0].lower(), words[1:]

    if name in ("goto", "setpoint"):
        if len(args) != 3:
            raise UsageError("usage: goto LAT LON ALT")
        try:
            lat, lon, alt = (float(arg) for arg in args)
        except ValueError:
            raise UsageError(f"not a number in: {' '.join(args)}") from None
        try:
            return setpoint(lat, lon, alt)
        except ValueError as error:
            raise UsageError(str(error)) from None
    if name in ("home", "go_home"):
        if args:
            raise UsageError("usage: home")
        return go_home()
    if name in ("help", "?"):
        return HELP
    if name in ("quit", "exit"):
        return QUIT
    raise UsageError(f"unknown command {name!r}; type 'help'")


def describe(command: dict[str, Any]) -> str:
    if command["type"] == "setpoint":
        return (
            f"setpoint lat={command['lat']} lon={command['lon']} "
            f"alt={command['alt']}m"
        )
    return "go home"


def run(
    client: SetpointClient,
    read_line: Callable[[str], str] = input,
    write: Callable[[str], None] = print,
) -> None:
    """Prompt for commands until quit or end of input."""
    write(f"Setpoint tester - drone at {client.address}. Type 'help' for commands.")
    try:
        client.connect()
        write("Connected.")
    except LinkError as error:
        write(f"Not connected: {error}. Will retry on the next command.")

    while True:
        try:
            text = read_line("> ")
        except EOFError:
            write("")
            return
        try:
            action = parse_line(text)
        except UsageError as error:
            write(f"error: {error}")
            continue

        if action is None:
            continue
        if action == QUIT:
            return
        if action == HELP:
            write(HELP_TEXT)
            continue

        assert isinstance(action, dict)
        write(f"-> {describe(action)}")
        try:
            reply = client.send(action)
        except LinkError as error:
            write(f"FAILED: {error}")
            continue
        write(f"{'OK' if reply.ok else 'REJECTED'}: {reply.message}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m src.setpoint_tester",
        description="Send setpoint and go-home commands to a 3DS drone over TCP.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="drone IP address (default: %(default)s, i.e. SITL on this machine)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help="airside setpoint_link port (default: %(default)s)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="seconds to wait to connect and for each reply (default: %(default)s)",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error(f"--port must be 1-65535, got {args.port}")
    if not args.timeout > 0:
        parser.error(f"--timeout must be positive, got {args.timeout}")

    try:
        import readline  # noqa: F401  (arrow-key history for input() where available)
    except ImportError:
        pass

    client = SetpointClient(args.host, args.port, args.timeout)
    try:
        run(client)
    except KeyboardInterrupt:
        print()
    finally:
        client.close()
    return 0
