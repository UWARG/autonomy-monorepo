"""
Always checks for new messages, state of drone, and
the camera using telemetry.py (direct MAVROS subscriptions) and encodes it
using message_encoder.py.

Then calls the server to broadcast it to clients (socket.js). Every
broadcast message is also logged to a local .txt file (via the standard
`logging` module), so telemetry is captured even when no ground-station is
connected (e.g. no LTE in the field).
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path

from camera.src import SimCamera
from telemetry import Telemetry
from utils.src.message_encoder import (
    encode_attitude,
    encode_camera,
    encode_health,
    encode_log,
    encode_position,
)
from utils.src.types import AttitudeMessage, PositionMessage

import server

POLL_INTERVAL_S = 0.1
LOG_DIR = Path(__file__).parent / "logs"


def setup_telemetry_logger() -> logging.Logger:
    """Create a fresh timestamped .txt log for this run under logs/."""
    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.txt"

    handler = logging.FileHandler(path, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(created)f\t%(message)s"))

    logger = logging.getLogger("telemetry")
    logger.setLevel(logging.INFO)
    logger.addHandler(handler)
    logger.propagate = False
    return logger


def emit(logger: logging.Logger, data: bytes) -> None:
    """Broadcast a message to connected clients and log it to disk."""
    server.broadcast(data)
    logger.info(data.decode())


def poll_once(telemetry: Telemetry, camera: SimCamera, logger: logging.Logger) -> None:
    """Check every data source once and broadcast/log whatever came back."""
    attitude = AttitudeMessage(
        roll=0.0, pitch=0.0, yaw=0.0, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0
    )
    if telemetry.receive_attitude(attitude):
        emit(logger, encode_attitude(attitude))

    position = PositionMessage(lat=0.0, lon=0.0, alt=0.0)
    if telemetry.receive_position(position):
        emit(logger, encode_position(position))

    text = telemetry.receive_message()
    if text:
        emit(logger, encode_log(text))

    emit(logger, encode_health(telemetry.is_connected()))

    if camera.capture_frame() is not None:
        emit(logger, encode_camera())


def stream_forever(telemetry: Telemetry, camera: SimCamera, logger: logging.Logger) -> None:
    """Poll drone + camera state on a fixed interval, forever."""
    while True:
        poll_once(telemetry, camera, logger)
        time.sleep(POLL_INTERVAL_S)


def main() -> None:
    threading.Thread(target=server.serve_forever, daemon=True).start()

    telemetry = Telemetry()
    telemetry.connect()

    camera = SimCamera()
    camera.initialize_camera()

    logger = setup_telemetry_logger()
    stream_forever(telemetry, camera, logger)


if __name__ == "__main__":
    main()
