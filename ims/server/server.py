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
_clients_lock = threading.Lock()


@sock.route("/ws")
def handle_client(ws) -> None:
    """Register a browser connection and hold it open until it disconnects."""
    with _clients_lock:
        _clients.add(ws)

    try:
        while True:
            ws.receive()
    except ConnectionClosed:
        pass
    finally:
        with _clients_lock:
            _clients.discard(ws)


def broadcast(message: bytes) -> None:
    """Send an already-encoded message to every connected browser client."""
    with _clients_lock:
        clients = list(_clients)

    for client in clients:
        try:
            client.send(message)
        except ConnectionClosed:
            with _clients_lock:
                _clients.discard(client)


def serve_forever(host: str = "0.0.0.0", port: int = 8765) -> None:
    app.run(host=host, port=port, threaded=True)


if __name__ == "__main__":
    serve_forever()
