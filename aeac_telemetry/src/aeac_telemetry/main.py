"""Reads MAVLink, sends AEAC telemetry at 1 Hz."""

from __future__ import annotations

import logging
import sys

from aeac_telemetry.config import Config

log = logging.getLogger("aeac_telemetry")


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        config = Config.from_env()
    except ValueError as error:
        log.error("%s", error)
        return 1

    log.info("MAVLink %s -> AEAC as UAV '%s'", config.mavlink_url, config.uav_id)
    # TODO: open the MAVLink stream with pymavlink (mavutil.mavlink_connection)
    #   and keep the latest GLOBAL_POSITION_INT, GPS_RAW_INT, SYS_STATUS and
    #   HEARTBEAT.
    # TODO: connect to f"{websocket_url}?Authorization=<token>" and send one
    #   telemetry packet every 1 / SEND_HZ seconds, reconnecting on loss.
    log.error("not implemented yet")
    return 1


if __name__ == "__main__":
    sys.exit(main())
