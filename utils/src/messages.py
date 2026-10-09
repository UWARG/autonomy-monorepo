"""
msgspec Struct definitions for messages that cross the WebSocket boundary.
These are the wire format — used by airside_comms to encode and ims/server to decode.

Separate from types.py (plain dataclasses) which are used internally on the RPi.
"""

from typing import Any, Optional, Union

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


class TargetPayload(msgspec.Struct):
    lat: float
    lon: float
    label: Optional[str] = None
    tracking: Optional[bool] = None
    cluster: Optional[int] = None


class NearbyDronePayload(msgspec.Struct):
    id: int
    name: str
    lat: float
    lon: float
    alt: float
    speed: float
    direction: float
    horizontal_keep_away: float
    vertical_keep_away: float


class NearbyDronesPayload(msgspec.Struct):
    drones: list[NearbyDronePayload]


class TelemetrySentPayload(msgspec.Struct):
    # The exact packet sent to AEAC (its camelCase schema), echoed for display.
    packet: dict[str, Any]


class AeacAckPayload(msgspec.Struct):
    # AEAC's checks on the packet it acknowledged.
    unix_time: float
    inside_boundary: bool
    too_close_to_traffic: bool


class AeacInfractionPayload(msgspec.Struct):
    last_infraction: str
    # Cumulative per-type counts for this UAV, across sessions (e.g. slow_telem: 3).
    counts: dict[str, int]
    armed_seconds: float
    # Unix time the relay received it; replayed to late joiners, so arrival time isn't enough.
    received_at: float


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


class TargetMessage(msgspec.Struct, tag_field="type", tag="target"):
    payload: TargetPayload


class NearbyDronesMessage(msgspec.Struct, tag_field="type", tag="nearby_drones"):
    payload: NearbyDronesPayload


class TelemetrySentMessage(msgspec.Struct, tag_field="type", tag="telemetry_sent"):
    payload: TelemetrySentPayload


class AeacAckMessage(msgspec.Struct, tag_field="type", tag="aeac_ack"):
    payload: AeacAckPayload


class AeacInfractionMessage(msgspec.Struct, tag_field="type", tag="aeac_infraction"):
    payload: AeacInfractionPayload

AirsideMessage = Union[
    AttitudeMessage,
    PositionMessage,
    CameraMessage,
    HealthMessage,
    LogMessage,
    StatusMessage,
    TargetMessage,
    NearbyDronesMessage,
    TelemetrySentMessage,
    AeacAckMessage,
    AeacInfractionMessage,
]
