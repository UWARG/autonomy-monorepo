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


def _as_finite_float(value: Any) -> float | None:
    """`value` as a float if it is a finite number (not a bool), else None."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(value):
        return None
    return float(value)


def parse_location_command(raw: str | bytes, drone_id: str) -> LocationCommand | None:
    """Parse a LOCATION_COMMAND for `drone_id`, or return None if it is invalid."""
    try:
        message = json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError):
        logger.debug("%s: ignoring message that is not valid JSON", drone_id)
        return None

    if not isinstance(message, dict):
        logger.debug("%s: ignoring message that is not a JSON object", drone_id)
        return None
    if message.get("type") != MessageType.LOCATION_COMMAND:
        logger.debug("%s: ignoring message of type %r", drone_id, message.get("type"))
        return None
    if message.get("drone_id") != drone_id:
        logger.debug(
            "%s: ignoring command addressed to %r", drone_id, message.get("drone_id")
        )
        return None

    payload = message.get("payload")
    if not isinstance(payload, dict):
        logger.debug("%s: ignoring command with missing payload", drone_id)
        return None

    task_id = payload.get("task_id")
    target = payload.get("target")
    orientation = payload.get("orientation")
    if not isinstance(task_id, str) or not task_id:
        logger.debug("%s: ignoring command with invalid task_id %r", drone_id, task_id)
        return None
    if not isinstance(target, dict) or not isinstance(orientation, dict):
        logger.debug(
            "%s: ignoring command %s with missing target/orientation", drone_id, task_id
        )
        return None

    x = _as_finite_float(target.get("x"))
    y = _as_finite_float(target.get("y"))
    z = _as_finite_float(target.get("z"))
    yaw = _as_finite_float(orientation.get("yaw"))
    if x is None or y is None or z is None or yaw is None:
        logger.debug(
            "%s: ignoring command %s with invalid target %r or orientation %r",
            drone_id,
            task_id,
            target,
            orientation,
        )
        return None

    return LocationCommand(task_id=task_id, target=(x, y, z), yaw=yaw)
