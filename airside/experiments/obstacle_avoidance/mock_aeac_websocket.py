"""Small receive-only AEAC traffic server for the SITL integration test."""

from __future__ import annotations

import base64
import hashlib
import json
import socketserver
import threading
from collections.abc import Callable


_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class _Handler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        self.request.settimeout(3.0)
        headers = bytearray()
        while b"\r\n\r\n" not in headers and len(headers) < 8192:
            chunk = self.request.recv(4096)
            if not chunk:
                return
            headers.extend(chunk)
        lines = headers.decode("ascii", errors="replace").split("\r\n")
        key = next(
            (
                line.split(":", 1)[1].strip()
                for line in lines
                if line.lower().startswith("sec-websocket-key:")
            ),
            None,
        )
        if key is None or "Authorization=local-test-token" not in lines[0]:
            return
        accept = base64.b64encode(hashlib.sha1((key + _GUID).encode()).digest())
        self.request.sendall(
            b"HTTP/1.1 101 Switching Protocols\r\n"
            b"Upgrade: websocket\r\nConnection: Upgrade\r\n"
            + b"Sec-WebSocket-Accept: "
            + accept
            + b"\r\n\r\n"
        )
        server = self.server
        while not server.stopping.is_set():
            event = server.event()
            if event is not None:
                payload = json.dumps(event, separators=(",", ":")).encode()
                length = len(payload)
                if length < 126:
                    prefix = bytes((0x81, length))
                elif length < 65536:
                    prefix = bytes((0x81, 126)) + length.to_bytes(2, "big")
                else:
                    prefix = bytes((0x81, 127)) + length.to_bytes(8, "big")
                try:
                    self.request.sendall(prefix + payload)
                except OSError:
                    return
            server.stopping.wait(1.0)


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, event: Callable[[], dict | None], port: int) -> None:
        self.event = event
        self.stopping = threading.Event()
        super().__init__(("127.0.0.1", port), _Handler)


class MockAeacWebSocket:
    """Serve scenario truth at approximately 1 Hz without external traffic."""

    def __init__(self, event: Callable[[], dict | None], port: int = 18765) -> None:
        self._server = _Server(event, port)
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="mock_aeac", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._server.stopping.set()
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
