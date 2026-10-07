# ims

Intelligent Monitoring System — WebSocket relay and React dashboard for the ground station.

## Setup

The relay (`server/relay.py`) connects directly to AEAC and needs a connection token
(from https://aeac.mylonics.com/#/connect). Put it in `ims/.env` (gitignored) so it
doesn't have to be set by hand every run:

```
AEAC_CONNECTION_TOKEN=your-token-here
# The drone's rosbridge (e.g. its VPN address over LTE). Default ws://127.0.0.1:9090
ROSBRIDGE_URL=ws://<drone-ip>:9090
# UAV identifier sent to AEAC. Default WARG-01
AEAC_UAV_ID=WARG-01
```

## Architecture

The relay is the ground station's single connection to AEAC, in both directions:

- `server/relay.py` — `Relay` class. Runs the AEAC feed and the drone feed as background
  tasks, fans out `nearby_drones` updates to browser dashboards connected on `/client`
  (replaying the latest snapshot to clients that connect later), and sends our telemetry
  to AEAC at 1 Hz, echoing each sent packet to dashboards as `telemetry_sent`.
- `server/aeac.py` — async AEAC competition WebSocket client (traffic in, infractions logged).
- `server/drone.py` — async rosbridge client keeping the drone's latest MAVROS readings.
- `server/telemetry.py` — builds the AEAC telemetry packet. If nothing arrives from the
  drone for 2 s, the last known position keeps going out with mode `link-lost` and
  `telemetryLinkStatus: 0`, so AEAC sees a reported link loss rather than silence.
  `unixTime` is always the ground station's clock (unconfirmed with AEAC for the
  link-lost case — see the comment in `telemetry.py`).
