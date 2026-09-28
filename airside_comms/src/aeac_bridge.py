"""
Bridges the AEAC competition server to IMS: AEAC traffic becomes a NearbyDronesMessage.

Run from the monorepo root:
    AEAC_CONNECTION_TOKEN=... python -m airside_comms.src.aeac_bridge [--simulate]

Without --simulate the bridge only listens; with it, it also streams a simulated
circular flight to AEAC and forwards that position to IMS.
"""

import argparse
import math
import os
import time

from utils.src.messages import NearbyDronePayload, PositionPayload

from .aeac_client import AeacClient
from .comms import AirsideComms

DEFAULT_AEAC_URL = "wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod"
DEFAULT_IMS_URL = "ws://localhost:8765/airside"

SIM_UAV_ID = "TEAM-UAV-01"
SIM_CENTER_LAT = 50.098074883470005
SIM_CENTER_LON = -110.73566404793513
SIM_RADIUS_M = 30.0
SIM_ALTITUDE_M = 45.0
SIM_DEGREES_PER_TICK = 15
METRES_PER_DEGREE_LAT = 111_320


def traffic_to_drones(payload: dict) -> list[NearbyDronePayload]:
    """Convert an AEAC `traffic` event payload into IMS drone payloads."""
    drones = []
    for aircraft in payload["traffic"]:
        position = aircraft["position"]
        drones.append(
            NearbyDronePayload(
                id=int(aircraft["aircraftIndex"]),
                name=aircraft["name"],
                lat=float(position["lat"]),
                lon=float(position["lon"]),
                alt=float(position["altitude"]),
                speed=float(position["speed"]),
                direction=float(position["direction"]),
                horizontal_keep_away=float(aircraft["horizontalKeepAway"]),
                vertical_keep_away=float(aircraft["verticalKeepAway"]),
            )
        )
    return drones


def simulated_position(tick: int) -> PositionPayload:
    """A point on a circle around the simulated site centre, advancing each tick."""
    angle = math.radians(tick * SIM_DEGREES_PER_TICK)
    metres_per_degree_lon = METRES_PER_DEGREE_LAT * math.cos(math.radians(SIM_CENTER_LAT))
    return PositionPayload(
        lat=SIM_CENTER_LAT + (SIM_RADIUS_M * math.cos(angle)) / METRES_PER_DEGREE_LAT,
        lon=SIM_CENTER_LON + (SIM_RADIUS_M * math.sin(angle)) / metres_per_degree_lon,
        alt=SIM_ALTITUDE_M,
    )


def simulated_telemetry(tick: int, unix_time: float) -> dict:
    position = simulated_position(tick)
    return {
        "uavId": SIM_UAV_ID,
        "unixTime": unix_time,
        "latitude": position.lat,
        "longitude": position.lon,
        "altitudeAGL": position.alt,
        "horizontalPositionAccuracy": 1.2,
        "verticalPositionAccuracy": 1.5,
        "batteryPercentage": max(0, 90 - tick * 0.1),
        "mode": "armed-automatic",
        "telemetryLinkStatus": 1,
        "rcLinkStatus": 1,
    }


def run(simulate: bool) -> None:
    token = os.environ.get("AEAC_CONNECTION_TOKEN", "").strip()
    if not token:
        raise SystemExit("Set AEAC_CONNECTION_TOKEN to the token from the AEAC connect page.")
    aeac_url = os.environ.get("AEAC_WEBSOCKET_URL", DEFAULT_AEAC_URL)
    ims_url = os.environ.get("IMS_URL", DEFAULT_IMS_URL)

    ims = AirsideComms(ims_url)
    ims.connect()
    aeac = AeacClient(aeac_url, token)

    aeac.on_traffic = lambda payload: ims.send_nearby_drones(traffic_to_drones(payload))
    aeac.on_telemetry_ack = lambda msg: print("[telemetry_ack]", msg)
    aeac.on_infraction = lambda msg: print("[infraction]", msg)
    aeac.on_error = lambda msg: print("[error]", msg.get("message"))

    tick = 0
    try:
        while True:
            if simulate:
                aeac.send_telemetry(simulated_telemetry(tick, time.time()))
                ims.send_position(simulated_position(tick))
                tick += 1

            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                aeac.poll()
    finally:
        aeac.close()
        ims.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="stream simulated telemetry to AEAC and forward the simulated position to IMS",
    )
    run(parser.parse_args().simulate)
