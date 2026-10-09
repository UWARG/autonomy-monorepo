"""JSON message protocol between mock drones and the groundside drone state manager."""

from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

from src.geometry import Vector3

logger = logging.getLogger(__name__)


class MessageType(str, Enum):
    """The `type` field of every message in the protocol."""

    REGISTER = "REGISTER"
    LOCATION_COMMAND = "LOCATION_COMMAND"
    COMMAND_ACK = "COMMAND_ACK"
    DRONE_STATE = "DRONE_STATE"


class MissionState(str, Enum):
    """Mission states in DRONE_STATE. Only IDLE drones take new tasks."""

    IDLE = "IDLE"
    MOVING = "MOVING"
    ARRIVED = "ARRIVED"


@dataclass(frozen=True)
class LocationCommand:
    """A validated LOCATION_COMMAND payload."""

    task_id: str
    target: Vector3
    yaw: float


def make_message(
    msg_type: MessageType, drone_id: str, payload: dict[str, Any]
) -> dict[str, Any]:
    """Wrap `payload` in the message envelope with the current timestamp."""
    return {
        "type": msg_type.value,
        "drone_id": drone_id,
        "timestamp": time.time(),
        "payload": payload,
    }


def register(drone_id: str, vehicle_type: str = "QUADCOPTER") -> dict[str, Any]:
    """Build the REGISTER message a drone sends when it connects."""
    return make_message(MessageType.REGISTER, drone_id, {"vehicle_type": vehicle_type})


def command_ack(drone_id: str, task_id: str, accepted: bool) -> dict[str, Any]:
    """Build the COMMAND_ACK reply to a LOCATION_COMMAND."""
    return make_message(
        MessageType.COMMAND_ACK, drone_id, {"task_id": task_id, "accepted": accepted}
    )


def drone_state(
    drone_id: str,
    task_id: str | None,
    mission_state: MissionState,
    position: Vector3,
    orientation: Vector3,
) -> dict[str, Any]:
    """Build DRONE_STATE. Position (x, y, z) in m; orientation in rad."""
    x, y, z = position
    roll, pitch, yaw = orientation
    return make_message(
        MessageType.DRONE_STATE,
        drone_id,
        {
            "task_id": task_id,
            "mission_state": mission_state.value,
            "position": {"x": float(x), "y": float(y), "z": float(z)},
            "orientation": {
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
            },
        },
    )


def encode(message: dict[str, Any]) -> str:
    """Serialize to JSON; raises ValueError on NaN or infinity."""
    return json.dumps(message, allow_nan=False)


def _finite(value: Any) -> float:
    """`value` as a float, or raise unless it is a finite number (bools aren't)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"not a number: {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"not finite: {value!r}")
    return float(value)


def parse_location_command(raw: str | bytes, drone_id: str) -> LocationCommand | None:
    """Parse a LOCATION_COMMAND for `drone_id`, or return None if it is invalid."""
    try:
        message = json.loads(raw)
        if message["type"] != MessageType.LOCATION_COMMAND:
            raise ValueError(f"not a LOCATION_COMMAND: {message['type']!r}")
        if message["drone_id"] != drone_id:
            raise ValueError(f"addressed to {message['drone_id']!r}")
        payload = message["payload"]
        task_id = payload["task_id"]
        if not isinstance(task_id, str) or not task_id:
            raise ValueError(f"invalid task_id: {task_id!r}")
        target = payload["target"]
        position = (_finite(target["x"]), _finite(target["y"]), _finite(target["z"]))
        yaw = _finite(payload["orientation"]["yaw"])
    except (ValueError, KeyError, TypeError) as error:
        # json and Unicode decode errors are ValueErrors; KeyError/TypeError cover
        # missing fields and values of the wrong JSON type.
        logger.debug("%s: ignoring message (%r): %r", drone_id, error, raw)
        return None
    return LocationCommand(task_id=task_id, target=position, yaw=yaw)
