"""
AirsideComms — Streams airside data to IMS.
"""

import threading

from websockets.sync.client import connect as ws_connect

from utils.src.message_encoder import (
    encode_attitude,
    encode_camera,
    encode_health,
    encode_log,
    encode_position,
    encode_status,
)


class AirsideComms:
    """Manages the WebSocket connection to the IMS ground station."""

    def __init__(self, url: str) -> None:
        self._url = url
        self._ws = None
        self._receiver_thread = None
        self._on_message = None

    def connect(self) -> None:
        """Open the WebSocket connection to IMS."""
        self._ws = ws_connect(self._url)

    def disconnect(self) -> None:
        """Close the WebSocket connection."""
        if self._ws is not None:
            self._ws.close()
            self._ws = None

    def send_attitude(self, attitude) -> None:
        """Encode and send an AttitudeMessage to IMS."""

    def send_position(self, position) -> None:
        """Encode and send a PositionMessage to IMS."""

    def send_camera(self, width: int, height: int, encoding: str) -> None:
        """Encode and send camera frame metadata to IMS."""

    def send_health(self, healthy: bool) -> None:
        """Encode and send a health status to IMS."""

    def send_log(self, message: str) -> None:
        """Encode and send a log message to IMS."""

    def send_status(self, task: str, state: str, text: str) -> None:
        """Encode and send a StatusMessage to IMS."""
        # socket.js parses every incoming message as JSON text, so this must go out as a
        # text frame - passing the raw encoder bytes straight to send() would send a binary
        # frame instead, which the browser can't JSON.parse.
        self._ws.send(encode_status(task, state, text).decode())

    def send_json(self, message: str) -> None:
        """Send a raw JSON string to IMS."""

    def start_receiving(self) -> None:
        """Start a background thread that calls for each incoming message."""

    def receive_loop(self) -> None:
        """Loop to continuously receive messages"""
