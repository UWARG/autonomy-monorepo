"""Settings, read from the environment."""

from __future__ import annotations

import dataclasses
import os

DEFAULT_MAVLINK_URL = "udpin:0.0.0.0:14551"
DEFAULT_WEBSOCKET_URL = "wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod"

# AEAC telemetry compliance requires one update per second.
SEND_HZ = 1.0


@dataclasses.dataclass(frozen=True)
class Config:
    mavlink_url: str
    websocket_url: str
    token: str
    uav_id: str

    @classmethod
    def from_env(cls) -> Config:
        """Raises ``ValueError`` if a required variable is missing."""

        token = os.environ.get("AEAC_CONNECTION_TOKEN", "").strip()
        uav_id = os.environ.get("AEAC_UAV_ID", "").strip()
        missing = [
            name
            for name, value in (
                ("AEAC_CONNECTION_TOKEN", token),
                ("AEAC_UAV_ID", uav_id),
            )
            if not value
        ]
        if missing:
            raise ValueError(f"missing environment variables: {', '.join(missing)}")
        return cls(
            mavlink_url=os.environ.get("MAVLINK_URL", "").strip()
            or DEFAULT_MAVLINK_URL,
            websocket_url=os.environ.get("AEAC_WEBSOCKET_URL", "").strip()
            or DEFAULT_WEBSOCKET_URL,
            token=token,
            uav_id=uav_id,
        )
