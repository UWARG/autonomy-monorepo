from __future__ import annotations

import logging
import threading
import time
from dataclasses import asdict
from pathlib import Path

from telemetry import Telemetry
from utils.src.message_encoder import (
    encode_attitude,
    encode_health,
    encode_log,
    encode_position,
)
from utils.src.types import Attitude, PositionMessage

import server

POLL_INTERVAL_S = 0.1
LOG_DIR = Path(__file__).parent / "logs"


def setup_telemetry_logger() -> logging.Logger:
    """Create a fresh timestamped .txt log for this run under logs/."""
    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.txt"

    handler = logging.FileHandler(path, encoding="utf-8")
    handler.setFormatter(
        logging.Formatter("%(asctime)s.%(msecs)03d | %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
    )

    logger = logging.getLogger("telemetry")
    logger.setLevel(logging.INFO)
    logger.addHandler(handler)
    logger.propagate = False
    return logger


def _format_value(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.3f}"
    if isinstance(value, str):
        return f'"{value}"'
    return str(value)


def emit(logger: logging.Logger, msg_type: str, payload: dict, data: bytes) -> None:
    """Broadcast a message to connected clients and log it to disk."""
    server.broadcast(data)
    fields = " ".join(f"{key}={_format_value(value)}" for key, value in payload.items())
    logger.info(f"{msg_type.upper():<9} {fields}")


def poll_once(telemetry: Telemetry, logger: logging.Logger) -> None:
    """Check every data source once and broadcast/log whatever came back."""
    attitude = Attitude(
        roll=0.0, pitch=0.0, yaw=0.0, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0
    )
    if telemetry.receive_attitude(attitude):
        emit(logger, "attitude", asdict(attitude), encode_attitude(attitude))

    position = PositionMessage(lat=0.0, lon=0.0, alt=0.0)
    if telemetry.receive_position(position):
        emit(logger, "position", asdict(position), encode_position(position))

    text = telemetry.receive_message()
    if text:
        emit(logger, "log", {"message": text}, encode_log(text))

    # Broadcast only, not logged - health fires every tick and would swamp
    # the flight log with a low-signal line; the frontend still gets it live.
    server.broadcast(encode_health(telemetry.is_connected()))


def stream_forever(telemetry: Telemetry, logger: logging.Logger) -> None:
    """Poll drone state on a fixed interval, forever."""
    while True:
        poll_once(telemetry, logger)
        time.sleep(POLL_INTERVAL_S)


def main() -> None:
    threading.Thread(target=server.serve_forever, daemon=True).start()

    telemetry = Telemetry()
    telemetry.connect()

    logger = setup_telemetry_logger()
    stream_forever(telemetry, logger)


if __name__ == "__main__":
    main()
