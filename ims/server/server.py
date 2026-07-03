"""
Opens a port on RPi and accepts connections from the browser.
It maintains a list of connected clients to send data towards.
"""

from __future__ import annotations

import threading

from flask import Flask
from flask_sock import ConnectionClosed, Sock

app = Flask(__name__)
sock = Sock(app)

_clients: set = set()
_lock = threading.Lock()


@sock.route("/ws")
def ws_route(ws) -> None:
    with _lock:
        _clients.add(ws)

    try:
        while ws.connected:
            data = ws.receive()
            # Browsers are receive-only and never send anything; airside connections
            # send pre-encoded messages here, which get relayed to every other client.
            if data is not None:
                broadcast(data, exclude=ws)
    except ConnectionClosed:
        pass
    finally:
        with _lock:
            _clients.discard(ws)


def broadcast(data: bytes, exclude=None) -> None:
    """Send raw encoded message bytes to every connected client except `exclude`."""

    with _lock:
        clients = [client for client in _clients if client is not exclude]

    for client in clients:
        try:
            client.send(data)
        except ConnectionClosed:
            with _lock:
                _clients.discard(client)


def serve_forever(host: str = "0.0.0.0", port: int = 8765) -> None:
    app.run(host=host, port=port, threaded=True)
