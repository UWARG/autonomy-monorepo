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
