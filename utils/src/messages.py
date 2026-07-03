"""
msgspec Struct definitions for messages that cross the WebSocket boundary.
These are the wire format — used by airside_comms to encode and ims/server to decode.

Separate from types.py (plain dataclasses) which are used internally on the RPi.
"""

from typing import Union

import msgspec


class AttitudePayload(msgspec.Struct):
    roll: float
    pitch: float
    yaw: float
    rollspeed: float
    pitchspeed: float
    yawspeed: float


class PositionPayload(msgspec.Struct):
    lat: float
    lon: float
    alt: float


class CameraPayload(msgspec.Struct):
    """Payload for camera data - Currently unscoped"""


class HealthPayload(msgspec.Struct):
    healthy: bool


class LogPayload(msgspec.Struct):
    message: str


class StatusPayload(msgspec.Struct):
    """task/state reported as plain strings; state mirrors py_trees.common.Status
    (RUNNING/SUCCESS/FAILURE/INVALID) since that's what the engine's behaviors produce."""

    task: str
    state: str
    text: str


class AttitudeMessage(msgspec.Struct, tag_field="type", tag="attitude"):
    payload: AttitudePayload


class PositionMessage(msgspec.Struct, tag_field="type", tag="position"):
    payload: PositionPayload


class CameraMessage(msgspec.Struct, tag_field="type", tag="camera"):
    payload: CameraPayload


class HealthMessage(msgspec.Struct, tag_field="type", tag="health"):
    payload: HealthPayload


class LogMessage(msgspec.Struct, tag_field="type", tag="log"):
    payload: LogPayload


class StatusMessage(msgspec.Struct, tag_field="type", tag="status"):
    payload: StatusPayload

AirsideMessage = Union[
    AttitudeMessage,
    PositionMessage,
    CameraMessage,
    HealthMessage,
    LogMessage,
    StatusMessage,
]
