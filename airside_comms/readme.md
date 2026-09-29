# airside_comms

> [!WARNING]
> This is a skeletal project and has no functionality yet.

 WebSocket client that streams airside telemetry to the IMS ground station.

## Setup

`aeac_bridge.py` needs an AEAC connection token (from https://aeac.mylonics.com/#/connect).
Put it in `airside_comms/.env` (gitignored) so it doesn't have to be set by hand every run:

```
AEAC_CONNECTION_TOKEN=your-token-here
```

## Architecture

- `comms.py` — `AirsideComms` class. Manages the WebSocket connection and exposes `send_nearby_drones()`, `send_status()`.
