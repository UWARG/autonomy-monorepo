#!/usr/bin/env python3
"""Small local WebSocket AEAC fixture for SITL; never use for live flight."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import select
import socketserver
import struct
import threading
import time
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_EARTH_RADIUS_M = 6_371_000.0


class FixtureState:
    def __init__(self, *, mode: str, summary: Path) -> None:
        self.mode = mode
        self.summary = summary
        self.lock = threading.Lock()
        self.connections = 0
        self.telemetry: list[tuple[float, dict]] = []
        self.traffic_sent = 0
        self.origin: tuple[float, float] | None = None
        self.blackout_started_s: float | None = None

    def record_telemetry(self, payload: dict) -> None:
        with self.lock:
            now_s = time.monotonic()
            self.telemetry.append((now_s, payload))
            if self.origin is None:
                self.origin = (payload["latitude"], payload["longitude"])
            intervals = [
                b[0] - a[0] for a, b in zip(self.telemetry, self.telemetry[1:])
            ]
            armed = [
                item
                for item in self.telemetry
                if item[1].get("mode") in {"armed-pilot", "armed-automatic"}
            ]
            armed_intervals = [b[0] - a[0] for a, b in zip(armed, armed[1:])]
            self.summary.write_text(
                json.dumps(
                    {
                        "connections": self.connections,
                        "telemetry_count": len(self.telemetry),
                        "interval_min_s": min(intervals) if intervals else None,
                        "interval_max_s": max(intervals) if intervals else None,
                        "armed_telemetry_count": len(armed),
                        "armed_interval_min_s": (
                            min(armed_intervals) if armed_intervals else None
                        ),
                        "armed_interval_max_s": (
                            max(armed_intervals) if armed_intervals else None
                        ),
                        "armed_intervals_outside_0_4_to_1_1": sum(
                            not 0.4 <= interval <= 1.1
                            for interval in armed_intervals
                        ),
                        "traffic_sent": self.traffic_sent,
                        "last_payload": payload,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )

    def traffic_event(self) -> str | None:
        with self.lock:
            entries: list[dict] = []
            if self.telemetry and self.origin is not None:
                latest = self.telemetry[-1][1]
                if self.mode == "dropout":
                    north_m = math.radians(
                        latest["latitude"] - self.origin[0]
                    ) * _EARTH_RADIUS_M
                    if north_m >= 5.0 and self.blackout_started_s is None:
                        self.blackout_started_s = time.monotonic()
                    if (
                        self.blackout_started_s is not None
                        and time.monotonic() - self.blackout_started_s < 5.5
                    ):
                        return None
                entries.append(
                    {
                        "aircraftIndex": 1,
                        "name": "SITL-OWN",
                        "horizontalKeepAway": 5.0,
                        "verticalKeepAway": 5.0,
                        "position": {
                            "lat": latest["latitude"],
                            "lon": latest["longitude"],
                            "altitude": latest["altitudeAGL"],
                            "speed": 0.0,
                            "direction": 0.0,
                        },
                    }
                )
                if self.mode in {"static", "dropout"}:
                    lat = self.origin[0] + math.degrees(20.0 / _EARTH_RADIUS_M)
                    entries.append(
                        {
                            "aircraftIndex": 2,
                            "name": "SITL-STATIC",
                            "horizontalKeepAway": 5.0,
                            "verticalKeepAway": 5.0,
                            "position": {
                                "lat": lat,
                                "lon": self.origin[1],
                                "altitude": 15.0,
                                "speed": 0.0,
                                "direction": 0.0,
                            },
                        }
                    )
            self.traffic_sent += 1
        return json.dumps({"event": "traffic", "payload": {"traffic": entries}})


def _exact(sock, count: int) -> bytes:
    data = b""
    while len(data) < count:
        block = sock.recv(count - len(data))
        if not block:
            raise ConnectionError("socket closed")
        data += block
    return data


def _send_frame(sock, data: str, opcode: int = 1) -> None:
    payload = data.encode("utf-8")
    if len(payload) < 126:
        header = bytes((0x80 | opcode, len(payload)))
    elif len(payload) < 65536:
        header = bytes((0x80 | opcode, 126)) + struct.pack("!H", len(payload))
    else:
        header = bytes((0x80 | opcode, 127)) + struct.pack("!Q", len(payload))
    sock.sendall(header + payload)


def _read_frame(sock) -> tuple[int, str]:
    first, second = _exact(sock, 2)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", _exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", _exact(sock, 8))[0]
    if length > 1_000_000:
        raise ValueError("frame too large")
    mask = _exact(sock, 4) if second & 0x80 else None
    raw = _exact(sock, length)
    if mask:
        raw = bytes(byte ^ mask[i % 4] for i, byte in enumerate(raw))
    return first & 0x0F, raw.decode("utf-8")


class Handler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        self.request.settimeout(5.0)
        request = b""
        while b"\r\n\r\n" not in request and len(request) < 8192:
            request += self.request.recv(4096)
        lines = request.decode("latin1").split("\r\n")
        path = lines[0].split(" ")[1]
        token = parse_qs(urlsplit(path).query).get("Authorization", [""])[0]
        headers = {
            name.strip().lower(): value.strip()
            for line in lines[1:]
            if ":" in line
            for name, value in [line.split(":", 1)]
        }
        if token != self.server.token or "sec-websocket-key" not in headers:
            self.request.sendall(b"HTTP/1.1 401 Unauthorized\r\n\r\n")
            return
        accept = base64.b64encode(
            hashlib.sha1((headers["sec-websocket-key"] + _MAGIC).encode()).digest()
        ).decode()
        self.request.sendall(
            (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
            ).encode()
        )
        with self.server.fixture.lock:
            self.server.fixture.connections += 1
        self.request.settimeout(None)
        next_traffic_s = time.monotonic()
        try:
            while True:
                timeout_s = max(0.0, next_traffic_s - time.monotonic())
                readable, _, _ = select.select([self.request], [], [], timeout_s)
                if readable:
                    opcode, body = _read_frame(self.request)
                    if opcode == 8:
                        return
                    if opcode == 9:
                        _send_frame(self.request, body, opcode=10)
                    elif opcode == 1:
                        message = json.loads(body)
                        if message.get("action") == "telemetry":
                            self.server.fixture.record_telemetry(message["data"])
                            _send_frame(self.request, '{"event":"telemetry_ack"}')
                if time.monotonic() >= next_traffic_s:
                    event = self.server.fixture.traffic_event()
                    if event is not None:
                        _send_frame(self.request, event)
                    next_traffic_s += 1.0
        except (ConnectionError, OSError, ValueError, json.JSONDecodeError):
            return


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("clear", "static", "dropout"), required=True)
    parser.add_argument("--token", required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    fixture = FixtureState(mode=args.mode, summary=args.summary)
    with Server(("127.0.0.1", args.port), Handler) as server:
        server.token = args.token
        server.fixture = fixture
        server.serve_forever()


if __name__ == "__main__":
    main()
