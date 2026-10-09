"""
Encodes utils dataclasses into JSON bytes for transmission to IMS.
All messages use the envelope: {"type": "...", "payload": {...}}
"""

import msgspec

from utils.src.messages import (
    AeacAckMessage,
    AeacAckPayload,
    AeacInfractionMessage,
    AeacInfractionPayload,
    AttitudeMessage,
    AttitudePayload,
    HealthMessage,
    HealthPayload,
    LogMessage,
    LogPayload,
    NearbyDronePayload,
    NearbyDronesMessage,
    NearbyDronesPayload,
    PositionMessage,
    PositionPayload,
    StatusMessage,
    StatusPayload,
    TelemetrySentMessage,
    TelemetrySentPayload,
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
    """Encodes camera data into CameraMessage format"""

def encode_health(healthy: bool) -> bytes:
    return _encoder.encode(
        HealthMessage(payload=HealthPayload(healthy=healthy))
    )


def encode_log(message: str) -> bytes:
    return _encoder.encode(
        LogMessage(payload=LogPayload(message=message))
    )


def encode_status(task: str, state: str, text: str) -> bytes:
    return _encoder.encode(
        StatusMessage(payload=StatusPayload(task=task, state=state, text=text))
    )


def encode_nearby_drones(drones: list[NearbyDronePayload]) -> bytes:
    return _encoder.encode(
        NearbyDronesMessage(payload=NearbyDronesPayload(drones=drones))
    )


def encode_telemetry_sent(packet: dict) -> bytes:
    return _encoder.encode(TelemetrySentMessage(payload=TelemetrySentPayload(packet=packet)))


def encode_aeac_ack(payload: AeacAckPayload) -> bytes:
    return _encoder.encode(AeacAckMessage(payload=payload))


def encode_aeac_infraction(payload: AeacInfractionPayload) -> bytes:
    return _encoder.encode(AeacInfractionMessage(payload=payload))
