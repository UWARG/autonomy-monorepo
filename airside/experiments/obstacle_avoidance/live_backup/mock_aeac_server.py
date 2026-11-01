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


def _steady_time_s() -> float:
    raw_clock = getattr(time, "CLOCK_MONOTONIC_RAW", None)
    return time.clock_gettime(raw_clock) if raw_clock is not None else time.monotonic()


class FixtureState:
    def __init__(self, *, mode: str, summary: Path, truth_jsonl: Path | None = None) -> None:
        self.mode = mode
        self.summary = summary
        self.truth_jsonl = truth_jsonl
        self.lock = threading.Lock()
        self.connections = 0
        self.telemetry: list[tuple[float, dict]] = []
        self.traffic_sent = 0
        self.traffic_sent_at_s: dict[int, list[float]] = {}
        self.origin: tuple[float, float] | None = None
        self.blackout_started_s: float | None = None
        self.movement_started_s: float | None = None

    def record_telemetry(self, payload: dict) -> None:
        with self.lock:
            now_s = _steady_time_s()
            self.telemetry.append((now_s, payload))
            if self.origin is None:
                self.origin = (payload["latitude"], payload["longitude"])
            if (
                self.mode in {"moving", "multi", "turn", "delay"}
                and self.movement_started_s is None
            ):
                north_m = math.radians(payload["latitude"] - self.origin[0]) * _EARTH_RADIUS_M
                if payload.get("mode") == "armed-automatic" and north_m >= 2.0:
                    self.movement_started_s = now_s
            intervals = [
                b[0] - a[0] for a, b in zip(self.telemetry, self.telemetry[1:])
            ]
            armed = [
                item
                for item in self.telemetry
                if item[1].get("mode") in {"armed-pilot", "armed-automatic"}
            ]
            armed_intervals = [b[0] - a[0] for a, b in zip(armed, armed[1:])]
            traffic_intervals = [
                later - earlier
                for times in self.traffic_sent_at_s.values()
                for earlier, later in zip(times, times[1:])
            ]
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
                        "traffic_interval_min_s": (
                            min(traffic_intervals) if traffic_intervals else None
                        ),
                        "traffic_interval_max_s": (
                            max(traffic_intervals) if traffic_intervals else None
                        ),
                        "traffic_intervals_outside_0_8_to_1_2": sum(
                            not 0.8 <= interval <= 1.2 for interval in traffic_intervals
                        ),
                        "last_payload": payload,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )

    def _other_aircraft(self, now_s: float) -> list[dict]:
        if self.origin is None:
            return []

        def entry(index: int, name: str, east_m: float, north_m: float,
                  speed_mps: float, heading_deg: float) -> dict:
            latitude = self.origin[0] + math.degrees(north_m / _EARTH_RADIUS_M)
            longitude = self.origin[1] + math.degrees(
                east_m / (_EARTH_RADIUS_M * math.cos(math.radians(self.origin[0])))
            )
            return {
                "aircraftIndex": index,
                "name": name,
                "horizontalKeepAway": 5.0,
                "verticalKeepAway": 5.0,
                "position": {
                    "lat": latitude,
                    "lon": longitude,
                    "altitude": (
                        30.0 if self.mode in {"moving", "multi", "turn", "delay"}
                        else 15.0
                    ),
                    "speed": speed_mps,
                    "direction": heading_deg,
                },
            }

        if self.mode in {"static", "dropout"}:
            return [entry(2, "SITL-STATIC", 0.0, 20.0, 0.0, 0.0)]
        if self.mode == "blocked":
            return [entry(2, "SITL-BLOCKED-GOAL", 0.0, 40.0, 0.0, 0.0)]
        if self.mode not in {"moving", "multi", "turn", "delay"}:
            return []

        elapsed_s = max(0.0, now_s - self.movement_started_s) if self.movement_started_s else 0.0
        if self.mode == "turn" and elapsed_s >= 6.0:
            first_east_m = -12.0 + 1.2 * 6.0
            first = entry(2, "SITL-TURN", first_east_m, 20.0 + 1.2 * (elapsed_s - 6.0), 1.2, 0.0)
        else:
            first = entry(2, "SITL-MOVING", -12.0 + 1.2 * elapsed_s, 20.0, 1.2, 90.0)
        if self.mode == "multi":
            second = entry(3, "SITL-SECOND", 12.0 - 1.2 * elapsed_s, 27.0, 1.2, 270.0)
            return [first, second]
        return [first]

    def record_truth(self) -> None:
        if self.truth_jsonl is None:
            return
        with self.lock:
            now_s = _steady_time_s()
            if self.origin is None:
                return
            sample = {"monotonic_s": now_s, "aircraft": self._other_aircraft(now_s)}
        with self.truth_jsonl.open("a", encoding="utf-8") as output:
            output.write(json.dumps(sample, separators=(",", ":")) + "\n")

    def traffic_event(self, connection_number: int) -> str | None:
        with self.lock:
            now_s = _steady_time_s()
            entries: list[dict] = []
            if self.telemetry and self.origin is not None:
                latest = self.telemetry[-1][1]
                if self.mode == "dropout":
                    north_m = math.radians(
                        latest["latitude"] - self.origin[0]
                    ) * _EARTH_RADIUS_M
                    if north_m >= 5.0 and self.blackout_started_s is None:
                        self.blackout_started_s = now_s
                    if (
                        self.blackout_started_s is not None
                        and now_s - self.blackout_started_s < 5.5
                    ):
                        return None
                if self.mode == "delay" and self.movement_started_s is not None:
                    elapsed_s = now_s - self.movement_started_s
                    if 8.0 <= elapsed_s < 10.2:
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
                entries.extend(self._other_aircraft(now_s))
            self.traffic_sent += 1
            self.traffic_sent_at_s.setdefault(connection_number, []).append(now_s)
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
            connection_number = self.server.fixture.connections
        self.request.settimeout(None)
        next_traffic_s = _steady_time_s()
        try:
            while True:
                timeout_s = max(0.0, next_traffic_s - _steady_time_s())
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
                if _steady_time_s() >= next_traffic_s:
                    event = self.server.fixture.traffic_event(connection_number)
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
    parser.add_argument(
        "--mode",
        choices=("clear", "static", "dropout", "moving", "multi", "turn", "delay", "blocked"),
        required=True,
    )
    parser.add_argument("--token", required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--truth-jsonl", type=Path)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    fixture = FixtureState(mode=args.mode, summary=args.summary, truth_jsonl=args.truth_jsonl)
    if args.truth_jsonl is not None:
        def sample_truth() -> None:
            while True:
                fixture.record_truth()
                time.sleep(0.1)
        threading.Thread(target=sample_truth, daemon=True).start()
    with Server(("127.0.0.1", args.port), Handler) as server:
        server.token = args.token
        server.fixture = fixture
        server.serve_forever()


if __name__ == "__main__":
    main()
