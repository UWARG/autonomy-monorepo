"""
AirsideComms — Streams airside data to IMS.
"""

from websockets.sync.client import connect as ws_connect

from utils.src.message_encoder import (
    encode_nearby_drones,
    encode_status,
)


class AirsideComms:
    """Manages the WebSocket connection to the IMS ground station."""

    def __init__(self, url: str) -> None:
        self._url = url
        self._ws = None

    def connect(self) -> None:
        """Open the WebSocket connection to IMS."""
        self._ws = ws_connect(self._url)

    def disconnect(self) -> None:
        """Close the WebSocket connection."""
        if self._ws is not None:
            self._ws.close()
            self._ws = None

    def send_nearby_drones(self, drones) -> None:
        """Encode and send a NearbyDronesMessage (full traffic snapshot) to IMS."""
        self._ws.send(encode_nearby_drones(drones))

    def send_status(self, task: str, state: str, text: str) -> None:
        """Encode and send a StatusMessage to IMS."""
        self._ws.send(encode_status(task, state, text))
