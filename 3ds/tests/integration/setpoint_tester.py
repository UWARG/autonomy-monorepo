# /// script
# requires-python = ">=3.9"
# dependencies = ["roslibpy~=2.1"]
# ///
"""
Flight-test tool: command the 3DS mission from a laptop over rosbridge.

Connects to the rosbridge server that 3ds.launch.py runs on the drone (port
9090, the same link the IMS dashboard uses) and publishes typed commands on
/mission/setpoint and /mission/go_home, the topics the engine subscribes to.
Status text the engine sends to the ground is printed as it arrives.

    uv run 3ds/tests/integration/setpoint_tester.py --host <drone-ip>
"""

from __future__ import annotations

import argparse
import math
from typing import Any

import roslibpy
from roslibpy.core import RosTimeoutError, ServiceException

SETPOINT_TOPIC = "/mission/setpoint"
SETPOINT_TYPE = "airside_interfaces/msg/Coordinate"
GO_HOME_TOPIC = "/mission/go_home"
GO_HOME_TYPE = "std_msgs/msg/Empty"
STATUS_TOPIC = "/mavros/statustext/send"
STATUS_TYPE = "mavros_msgs/msg/StatusText"

# If a change to min alt occurs in constants.py, change it here too.
MIN_ALTITUDE_M = 2.0

HELP_TEXT = """\
Commands:
  goto LAT LON ALT   fly to a point and photograph it (ALT: meters above launch)
  home               return to the launch point and land (ends the mission)
  help               show this help
  quit               exit (also Ctrl-D)
Commas are ignored, so coordinates can be pasted from a map:
  goto 43.4339, -80.5776, 10
Lines starting with [drone] are status messages from the engine."""


def parse_command(text: str) -> tuple[str, dict[str, Any]] | None:
    """
    Turns one typed line into ``(kind, payload)``, where kind is "setpoint",
    "go_home", "help" or "quit". Returns None for a blank line and raises
    ValueError with a message for the operator if the line is invalid.
    """

    words = text.replace(",", " ").split()
    if not words:
        return None
    name, args = words[0].lower(), words[1:]

    if name == "goto":
        if len(args) != 3:
            raise ValueError("usage: goto LAT LON ALT")
        try:
            lat, lon, alt = (float(arg) for arg in args)
        except ValueError:
            raise ValueError(f"not a number in: {' '.join(args)}") from None
        if not all(math.isfinite(value) for value in (lat, lon, alt)):
            raise ValueError("coordinates must be finite numbers")
        if not -90.0 <= lat <= 90.0:
            raise ValueError(f"lat {lat} is outside [-90, 90]")
        if not -180.0 <= lon <= 180.0:
            raise ValueError(f"lon {lon} is outside [-180, 180]")
        if alt < MIN_ALTITUDE_M:
            raise ValueError(
                f"alt {alt}m is below the engine's {MIN_ALTITUDE_M}m minimum"
            )
        return "setpoint", {"lat": lat, "lon": lon, "alt": alt}
    if name == "home":
        if args:
            raise ValueError("usage: home")
        return "go_home", {}
    if name in ("help", "?"):
        return "help", {}
    if name in ("quit", "exit"):
        return "quit", {}
    raise ValueError(f"unknown command {name!r}; type 'help'")


def is_subscribed(ros: roslibpy.Ros, topic: str) -> bool | None:
    """Whether any node (i.e. the engine) subscribes to ``topic``, asked via
    rosapi. None if rosapi did not answer."""

    service = roslibpy.Service(ros, "/rosapi/subscribers", "rosapi/Subscribers")
    try:
        response = service.call(roslibpy.ServiceRequest({"topic": topic}), timeout=2.0)
    except (RosTimeoutError, ServiceException):
        return None
    return bool(response["subscribers"])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Send setpoint and go-home commands to a 3DS drone over rosbridge."
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="drone IP address (default: %(default)s, i.e. SITL on this machine)",
    )
    parser.add_argument(
        "--port", type=int, default=9090, help="rosbridge port (default: %(default)s)"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="seconds to wait for the connection (default: %(default)s)",
    )
    args = parser.parse_args(argv)
    url = f"ws://{args.host}:{args.port}"

    ros = roslibpy.Ros(host=args.host, port=args.port)
    try:
        ros.run(timeout=args.timeout)
    except RosTimeoutError:
        print(f"Cannot reach rosbridge at {url}. Is the 3DS stack running there?")
        ros.terminate()
        return 1

    # Advertise up front so the first command does not wait for it.
    topics = {
        "setpoint": roslibpy.Topic(ros, SETPOINT_TOPIC, SETPOINT_TYPE),
        "go_home": roslibpy.Topic(ros, GO_HOME_TOPIC, GO_HOME_TYPE),
    }
    for topic in topics.values():
        topic.advertise()
    # roslibpy re-subscribes once per dropped connection attempt, so after a
    # reconnect rosbridge can deliver each message twice. The engine stamps
    # every message, so skip exact repeats.
    last_status: tuple[int, int, str] | None = None

    def on_status(message: dict[str, Any]) -> None:
        nonlocal last_status
        stamp = message["header"]["stamp"]
        key = (stamp["sec"], stamp["nanosec"], message["text"])
        if key != last_status:
            last_status = key
            print(f"\n[drone] {message['text']}")

    status = roslibpy.Topic(ros, STATUS_TOPIC, STATUS_TYPE)
    status.subscribe(on_status)

    # roslibpy reconnects, re-advertises and re-subscribes by itself. Track the
    # websocket ourselves: ros.is_connected only reflects the TCP socket, which
    # is up before rosbridge's websocket is, and roslibpy holds anything
    # published in that gap until the next successful connection.
    link_up = True
    quitting = False

    def on_close(_proto: object) -> None:
        nonlocal link_up
        if link_up and not quitting:
            print("\n[link] lost connection to rosbridge, reconnecting...")
        link_up = False

    def on_ready(_proto: object) -> None:
        nonlocal link_up
        link_up = True
        print("\n[link] reconnected to rosbridge")

    ros.on("close", on_close)
    ros.on("ready", on_ready)

    print(f"Connected to rosbridge at {url}. Type 'help' for commands.")
    subscribed = is_subscribed(ros, SETPOINT_TOPIC)
    if subscribed is False:
        print(
            f"WARNING: nothing subscribes to {SETPOINT_TOPIC} - is the engine running?"
        )
    elif subscribed is None:
        print("(could not ask rosapi whether the engine is running)")

    try:
        while True:
            try:
                text = input("> ")
            except EOFError:
                print()
                break
            try:
                command = parse_command(text)
            except ValueError as error:
                print(f"error: {error}")
                continue

            if command is None:
                continue
            kind, payload = command
            if kind == "quit":
                break
            if kind == "help":
                print(HELP_TEXT)
                continue

            # roslibpy would queue the message and send it after reconnecting,
            # possibly much later, so refuse instead.
            if not link_up:
                print("NOT SENT: no connection to rosbridge (reconnecting); try again")
                continue

            topic = topics[kind]
            topic.publish(roslibpy.Message(payload))
            if kind == "setpoint":
                print(
                    f"sent setpoint lat={payload['lat']} lon={payload['lon']} "
                    f"alt={payload['alt']}m on {topic.name}"
                )
            else:
                print(f"sent go home on {topic.name}")
            if is_subscribed(ros, topic.name) is False:
                print(
                    f"WARNING: nothing subscribes to {topic.name} - is the engine running?"
                )
    except KeyboardInterrupt:
        print()
    finally:
        quitting = True
        ros.terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
