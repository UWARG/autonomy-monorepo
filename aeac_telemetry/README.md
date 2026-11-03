# aeac_telemetry

Sends our drone's telemetry to the AEAC competition server. It reads the
MAVLink stream from Mission planner via pymavlink and pushes one telemetry
packet per second over the server's WebSocket.

## Running

```bash
cp .env.example .env   # fill in the token and UAV ID
warg run aeac_telemetry setup
set -a; source .env; set +a
warg run aeac_telemetry run
```

| Environment variable | Default | Description |
|---|---|---|
| `AEAC_CONNECTION_TOKEN` | required | AEAC team token |
| `AEAC_UAV_ID` | required | UAV ID |
| `MAVLINK_URL` | `udpin:0.0.0.0:14551` | pymavlink connection string for the telemetry stream |
| `AEAC_WEBSOCKET_URL` | AEAC production URL | Override for other servers |

## What it sends

One packet per second (never less than 0.9 s apart):

| Field | Source |
|---|---|
| `unixTime` | `SYSTEM_TIME` (FC GPS time), advanced to the send time; system clock until the FC has one |
| `latitude`, `longitude`, `altitudeAGL` | `GLOBAL_POSITION_INT` (`relative_alt`) |
| `horizontal/verticalPositionAccuracy` | `GPS_RAW_INT` `h_acc`/`v_acc`, else `eph`/`epv` as metres |
| `batteryPercentage` | `SYS_STATUS.battery_remaining` |
| `mode` | `HEARTBEAT`: `link-lost` (no heartbeat for 2 s), `failsafe` (CRITICAL/EMERGENCY), `idle` (disarmed), `armed-automatic` (AUTO, GUIDED, RTL, LAND, ...), else `armed-pilot` |
| `telemetryLinkStatus` | 1 while heartbeats arrive, else 0 |
| `rcLinkStatus` | `RC_CHANNELS.rssi` / 254 (1 if the receiver doesn't report it, 0 if channels stop) |
