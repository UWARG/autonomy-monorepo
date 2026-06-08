"""
Encodes utils dataclasses into JSON bytes for transmission to IMS.
Uses msgspec for fast serialization and schema enforcement.
All messages use the envelope: {"type": "...", "payload": {...}}
"""

import msgspec

from utils.messages import (
    AttitudeMessage,
    AttitudePayload,
    HealthMessage,
    HealthPayload,
    LogMessage,
    LogPayload,
    PositionMessage,
    PositionPayload,
)

_encoder = msgspec.json.Encoder()

def encode_attitude(attitude) -> bytes:
    return _encoder.encode(
        AttitudeMessage(
            payload=AttitudePayload(
                roll=float(attitude.roll),
                pitch=float(attitude.pitch),
                yaw=float(attitude.yaw),
                rollspeed=float(attitude.rollspeed),
                pitchspeed=float(attitude.pitchspeed),
                yawspeed=float(attitude.yawspeed),
            )
        )
    )


def encode_position(position) -> bytes:
    return _encoder.encode(
        PositionMessage(
            payload=PositionPayload(
                lat=float(position.lat),
                lon=float(position.lon),
                alt=float(position.alt),
            )
        )
    )


def encode_camera() -> bytes:
    """Encodes camera data into CameraMessage format. Currently unscoped."""

def encode_health(healthy: bool) -> bytes:
    return _encoder.encode(
        HealthMessage(payload=HealthPayload(healthy=healthy))
    )


def encode_log(message: str) -> bytes:
    return _encoder.encode(
        LogMessage(payload=LogPayload(message=message))
    )
