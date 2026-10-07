"""
Async rosbridge client keeping the drone's latest MAVROS readings on the ground.

The relay builds AEAC telemetry from these, so it can keep reporting (as
link-lost) when the drone's link drops instead of AEAC seeing silence.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time

from websockets.asyncio.client import connect as ws_connect
from websockets.exceptions import WebSocketException

log = logging.getLogger("ims.drone")

RECONNECT_DELAY_S = 3

# rosbridge-side rate limit per topic: 1 Hz telemetry doesn't need MAVROS's
# full rates crossing the LTE link.
THROTTLE_MS = 200

GLOBAL_POSITION_TOPIC = "/mavros/global_position/global"
REL_ALT_TOPIC = "/mavros/global_position/rel_alt"
STATE_TOPIC = "/mavros/state"
BATTERY_TOPIC = "/mavros/battery"

_TOPIC_TYPES = {
    GLOBAL_POSITION_TOPIC: "sensor_msgs/msg/NavSatFix",
    REL_ALT_TOPIC: "std_msgs/msg/Float64",
    STATE_TOPIC: "mavros_msgs/msg/State",
    BATTERY_TOPIC: "sensor_msgs/msg/BatteryState",
}


class DroneState:
    """Latest message per topic, plus when anything last arrived from the drone."""

    def __init__(self) -> None:
        self.messages: dict[str, dict] = {}
        self._last_message_at: float | None = None

    def update(self, topic: str, message: dict) -> None:
        self.messages[topic] = message
        self._last_message_at = time.monotonic()

    def age_s(self) -> float | None:
        """Seconds since the last message, or None if nothing has arrived yet."""
        if self._last_message_at is None:
            return None
        return time.monotonic() - self._last_message_at


async def run_drone_feed(url: str, state: DroneState) -> None:
    """Subscribes to the MAVROS topics over rosbridge. Reconnects forever."""
    while True:
        try:
            async with ws_connect(url, max_size=None) as ws:
                log.info("connected to rosbridge at %s", url)
                for topic, msg_type in _TOPIC_TYPES.items():
                    await ws.send(
                        json.dumps(
                            {
                                "op": "subscribe",
                                "topic": topic,
                                "type": msg_type,
                                "throttle_rate": THROTTLE_MS,
                            }
                        )
                    )
                async for raw in ws:
                    message = json.loads(raw)
                    if message.get("op") == "publish":
                        state.update(message["topic"], message["msg"])
        except (WebSocketException, OSError) as error:
            log.warning("rosbridge connection lost (%s); reconnecting in %ss", error, RECONNECT_DELAY_S)
        await asyncio.sleep(RECONNECT_DELAY_S)
