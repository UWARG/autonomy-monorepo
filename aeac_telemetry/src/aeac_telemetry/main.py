"""Reads MAVLink, sends AEAC telemetry at 1 Hz."""

from __future__ import annotations

import json
import logging
import sys
import time
from urllib.parse import quote

from pymavlink import mavutil
from websockets.exceptions import ConnectionClosed, InvalidHandshake
from websockets.sync.client import connect

from aeac_telemetry.config import SEND_HZ, Config
from aeac_telemetry.vehicle import VehicleState

log = logging.getLogger("aeac_telemetry")

PERIOD_S = 1.0 / SEND_HZ
# The server penalizes packet gaps > 1.1 s and < 0.4 s.
MIN_GAP_S = 0.9
STREAM_RATE_HZ = 4


def _handle_event(message: dict) -> None:
    event = message.get("event")
    if event == "telemetry_ack":
        if message.get("insideBoundary") is False:
            log.warning("server: outside the flight boundary")
        if message.get("tooCloseToTraffic"):
            log.warning("server: too close to traffic")
    elif event == "infraction":
        log.warning("infraction: %s", message.get("last_infraction"))
    elif event == "error":
        log.error("server error: %s", message.get("message"))
    # "traffic" also arrives at 1 Hz; airside and the IMS use it.


def run(config: Config) -> None:
    mav = mavutil.mavlink_connection(config.mavlink_url, source_system=254)
    vehicle = VehicleState()
    streams_requested = False
    url = f"{config.websocket_url}?Authorization={quote(config.token, safe='')}"

    while True:
        try:
            with connect(url, open_timeout=10) as ws:
                log.info("connected to AEAC as '%s'", config.uav_id)
                last_send = -float("inf")
                next_tick = time.monotonic()
                while True:
                    # Read MAVLink and server events until the next send.
                    while time.monotonic() < max(next_tick, last_send + MIN_GAP_S):
                        msg = mav.recv_match(blocking=True, timeout=0.05)
                        if msg is not None:
                            vehicle.handle(msg, time.monotonic())
                            if not streams_requested and msg.get_type() == "HEARTBEAT":
                                mav.mav.request_data_stream_send(
                                    msg.get_srcSystem(),
                                    msg.get_srcComponent(),
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                    STREAM_RATE_HZ,
                                    1,
                                )
                                streams_requested = True
                        try:
                            while True:
                                _handle_event(json.loads(ws.recv(timeout=0)))
                        except TimeoutError:
                            pass
                        except ValueError:
                            log.warning("unparseable server message")

                    now = time.monotonic()
                    packet = vehicle.packet(config.uav_id, now)
                    if packet is None:
                        log.info("waiting for a heartbeat on %s", config.mavlink_url)
                    else:
                        ws.send(json.dumps({"action": "telemetry", "data": packet}))
                        last_send = now
                        if not vehicle.has_fix(now):
                            log.warning("no GPS fix (%s)", packet["mode"])
                        log.debug("sent %s", packet)
                    # Never catch up with a burst after a stall.
                    next_tick = max(next_tick + PERIOD_S, now + MIN_GAP_S)
        except InvalidHandshake as error:
            sys.exit(f"Server rejected the connection ({error}) - check the token")
        except (ConnectionClosed, OSError) as error:
            log.warning("no server connection (%s); retrying", error)
            time.sleep(1.0)


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        config = Config.from_env()
    except ValueError as error:
        log.error("%s", error)
        return 1
    try:
        run(config)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
