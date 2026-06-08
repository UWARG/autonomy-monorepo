"""
Encodes utils dataclasses into JSON bytes for transmission to IMS.
Uses msgspec for fast serialization and schema enforcement.
All messages use the envelope: {"type": "...", "payload": {...}}
"""

import math

import msgspec

from utils.messages import (
    AttitudeMessage,
    AttitudePayload,
    CameraMessage,
    CameraPayload,
    HealthMessage,
    HealthPayload,
    LogMessage,
    LogPayload,
    PositionMessage,
    PositionPayload,
)

_encoder = msgspec.json.Encoder()


def _safe_float(value: float):
    return None if math.isnan(value) else value


def encode_attitude(attitude) -> bytes:
    return _encoder.encode(
        AttitudeMessage(
            payload=AttitudePayload(
                roll=_safe_float(attitude.roll),
                pitch=_safe_float(attitude.pitch),
                yaw=_safe_float(attitude.yaw),
                rollspeed=_safe_float(attitude.rollspeed),
                pitchspeed=_safe_float(attitude.pitchspeed),
                yawspeed=_safe_float(attitude.yawspeed),
            )
        )
    )


def encode_position(position) -> bytes:
    return _encoder.encode(
        PositionMessage(
            payload=PositionPayload(
                lat=_safe_float(position.lat),
                lon=_safe_float(position.lon),
                alt=_safe_float(position.alt),
            )
        )
    )


def encode_camera(width: int, height: int, encoding: str) -> bytes:
    return _encoder.encode(
        CameraMessage(
            payload=CameraPayload(
                width=width,
                height=height,
                encoding=encoding,
            )
        )
    )


def encode_health(healthy: bool) -> bytes:
    return _encoder.encode(
        HealthMessage(payload=HealthPayload(healthy=healthy))
    )


def encode_log(message: str) -> bytes:
    return _encoder.encode(
        LogMessage(payload=LogPayload(message=message))
    )
