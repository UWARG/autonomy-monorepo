"""Operator CLI for the 3DS rosbridge link."""

from __future__ import annotations

import argparse
import asyncio
import os
import sys

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from .client import CommandDeliveryUnknown, LinkError, RosbridgeSession, Setpoint

DEFAULT_URL = "ws://10.42.3.2:9090"
LINK_ERRORS = (LinkError, WebSocketException, OSError, asyncio.TimeoutError)
RECONNECT_DELAY = 1.0


def _connect(url: str):
    return connect(url, open_timeout=5, close_timeout=1)


async def _status(url: str, watch: bool) -> None:
    last_error = None
    while True:
        try:
            async with _connect(url) as websocket:
                session = RosbridgeSession(websocket)
                await session.subscribe()
                await session.wait_for_heartbeat()
                print("Online: receiving airside heartbeat.", flush=True)
                if not watch:
                    return
                last_error = None
                while True:
                    await session.wait_for_heartbeat()
        except LINK_ERRORS as error:
            if not watch:
                raise
            if str(error) != last_error:
                print(f"Offline: {error}", file=sys.stderr, flush=True)
                last_error = str(error)
            await asyncio.sleep(RECONNECT_DELAY)


async def _command(url: str, setpoint: Setpoint | None) -> None:
    async with _connect(url) as websocket:
        session = RosbridgeSession(websocket)
        if setpoint is None:
            await session.advertise_go_home()
        else:
            await session.advertise_setpoint()
        await session.subscribe()
        await session.wait_for_heartbeat()
        # Give the newly advertised ROS publisher time for DDS discovery.
        await session.wait_for_heartbeat()
        if setpoint is None:
            await session.go_home()
        else:
            await session.send_setpoint(setpoint)
        print("Sent over WebSocket. Airside delivery unconfirmed.")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url", default=os.environ.get("THREEDS_ROSBRIDGE_URL", DEFAULT_URL)
    )
    commands = parser.add_subparsers(dest="command", required=True)
    status = commands.add_parser("status", help="Check for airside heartbeats")
    status.add_argument(
        "--watch", action="store_true", help="Monitor the link and reconnect"
    )
    setpoint_parser = commands.add_parser(
        "send-setpoint", help="Send one mapping setpoint"
    )
    setpoint_parser.add_argument("lat", type=float)
    setpoint_parser.add_argument("lon", type=float)
    setpoint_parser.add_argument("alt", type=float, help="Relative altitude in metres")
    commands.add_parser("go-home", help="Send the airside return-home command")
    args = parser.parse_args(argv)

    setpoint = None
    if args.command == "send-setpoint":
        try:
            setpoint = Setpoint(args.lat, args.lon, args.alt)
        except ValueError as error:
            parser.error(str(error))

    try:
        if args.command == "status":
            asyncio.run(_status(args.url, args.watch))
        else:
            asyncio.run(_command(args.url, setpoint))
    except CommandDeliveryUnknown as error:
        print(error, file=sys.stderr)
        return 3
    except LINK_ERRORS as error:
        print(f"3DS link: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
