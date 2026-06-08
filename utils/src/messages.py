"""
msgspec Struct definitions for messages that cross the WebSocket boundary.
These are the wire format — used by airside_comms to encode and ims/server to decode.

Separate from types.py (plain dataclasses) which are used internally on the RPi.
"""

from typing import Optional, Union

import msgspec


class AttitudePayload(msgspec.Struct):
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    rollspeed: Optional[float] = None
    pitchspeed: Optional[float] = None
    yawspeed: Optional[float] = None


class PositionPayload(msgspec.Struct):
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None


class CameraPayload(msgspec.Struct):
    """Payload for camera data - Currently unscoped"""


class HealthPayload(msgspec.Struct):
    healthy: bool


class LogPayload(msgspec.Struct):
    message: str


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

AirsideMessage = Union[
    AttitudeMessage,
    PositionMessage,
    CameraMessage,
    HealthMessage,
    LogMessage,
]
