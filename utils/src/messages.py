"""
DEPRECATED — orphaned by the rosbridge migration; nothing consumes this contract.

msgspec Struct definitions for messages that were to cross the WebSocket boundary
between airside_comms (encode) and ims/server (decode). Separate from types.py
(plain dataclasses) which are used internally on the RPi.

Why deprecated:
  * The consumer side was deleted in f70a50a (rosbridge support, #124, 2026-07-24):
    ims/frontend/src/socket.js, ims/server/server.py and ims/server/streamer.py are
    gone, and the dashboard widgets read ROS topics directly instead.
  * The producer side never worked: airside_comms is a README-declared skeleton whose
    send_* methods are empty stubs, and encode_camera() has no return statement.
  * No test covers the encoder or the decoder. ims/frontend/src/types.ts is
    mislabelled as the mirror of this module — treat that file as deprecated too.

Live telemetry now arrives over rosbridge (ws://<host>:9090) via roslib, reading the
ROS topics directly (e.g. /mavros/local_position/pose, /mavros/imu/data).
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
