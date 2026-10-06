# ims

Intelligent Monitoring System — WebSocket relay and React dashboard for the ground station.

## Setup

The relay (`server/relay.py`) connects directly to AEAC and needs a connection token
(from https://aeac.mylonics.com/#/connect). Put it in `ims/.env` (gitignored) so it
doesn't have to be set by hand every run:

```
AEAC_CONNECTION_TOKEN=your-token-here
```

## Architecture

- `server/relay.py` — `Relay` class. Runs the AEAC feed (`server/aeac.py`) as a background
  task and fans out `nearby_drones` updates to browser dashboards connected on `/client`,
  replaying the latest snapshot to clients that connect later.
- `server/aeac.py` — async AEAC competition WebSocket client.
