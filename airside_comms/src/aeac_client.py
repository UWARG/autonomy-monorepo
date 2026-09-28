"""
Client for the AEAC competition WebSocket API.
"""

import json
from urllib.parse import quote

import websocket
from websocket import WebSocketTimeoutException


class AeacClient:
    """Connects to AEAC and dispatches incoming events to the on_* callbacks."""

    def __init__(self, url: str, token: str) -> None:
        separator = "&" if "?" in url else "?"
        self._ws = websocket.create_connection(
            f"{url}{separator}Authorization={quote(token, safe='')}"
        )
        self._ws.settimeout(0.2)
        self.on_telemetry_ack = None
        self.on_traffic = None
        self.on_infraction = None
        self.on_error = None

    def send_telemetry(self, telemetry: dict) -> None:
        self._send("telemetry", telemetry)

    def request_telemetry(self, uav_id: str) -> None:
        self._send("telemetry_request", {"uavId": uav_id})

    def request_infraction(self, uav_id: str) -> None:
        self._send("infraction_request", {"uavId": uav_id})

    def _send(self, action: str, data: dict) -> None:
        self._ws.send(json.dumps({"action": action, "data": data}))

    def poll(self) -> None:
        """Receive one event if available (waits up to 0.2 s). Call frequently."""
        try:
            raw = self._ws.recv()
        except WebSocketTimeoutException:
            return
        if not raw:
            return
        message = json.loads(raw)
        event = message.get("event")
        if event == "telemetry_ack" and self.on_telemetry_ack:
            self.on_telemetry_ack(message)
        elif event == "traffic" and self.on_traffic:
            self.on_traffic(message["payload"])
        elif event == "infraction" and self.on_infraction:
            self.on_infraction(message)
        elif event == "error" and self.on_error:
            self.on_error(message)
        else:
            print(f"[{event}]", message)

    def close(self) -> None:
        self._ws.close()
