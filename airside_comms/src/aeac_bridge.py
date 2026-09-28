"""
Bridges the AEAC competition server to IMS: AEAC traffic becomes a NearbyDronesMessage.

Listen-only: nothing is sent to AEAC.

Run from the monorepo root:
    AEAC_CONNECTION_TOKEN=... python -m airside_comms.src.aeac_bridge
"""

import os

from utils.src.messages import NearbyDronePayload

from .aeac_client import AeacClient
from .comms import AirsideComms

DEFAULT_AEAC_URL = "wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod"
DEFAULT_IMS_URL = "ws://localhost:8765/airside"

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


def run() -> None:
    token = os.environ.get("AEAC_CONNECTION_TOKEN", "").strip()
    if not token:
        raise SystemExit("Set AEAC_CONNECTION_TOKEN to the token from the AEAC connect page.")
    aeac_url = os.environ.get("AEAC_WEBSOCKET_URL", DEFAULT_AEAC_URL)
    ims_url = os.environ.get("IMS_URL", DEFAULT_IMS_URL)

    ims = AirsideComms(ims_url)
    ims.connect()
    aeac = AeacClient(aeac_url, token)

    aeac.on_traffic = lambda payload: ims.send_nearby_drones(traffic_to_drones(payload))
    aeac.on_infraction = lambda msg: print("[infraction]", msg)
    aeac.on_error = lambda msg: print("[error]", msg.get("message"))

    try:
        while True:
            aeac.poll()
    finally:
        aeac.close()
        ims.disconnect()


if __name__ == "__main__":
    run()
