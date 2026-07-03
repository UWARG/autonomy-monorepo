"""
Always checks for new messages, state of drone, and
the camera using mav_comms and encodes it using message_encoder.py.

Then calls the server to broadcast it to clients (socket.js).
"""

from __future__ import annotations

from collections.abc import Iterable

import server


def stream_forever(source: Iterable[bytes]) -> None:
    """Forward every already-encoded message from `source` to connected browser clients.

    `source` is expected to be an iterable of pre-encoded message bytes (see
    utils/src/message_encoder.py) - e.g. produced by polling mav_comms/camera state,
    or relayed from an incoming AirsideComms connection. Wiring up that real source is
    a separate piece of work; this is a generic pass-through.
    """

    for message in source:
        server.broadcast(message)
