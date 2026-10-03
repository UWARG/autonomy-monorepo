# airside_comms

> [!WARNING]
> **Deprecated.** Orphaned by the rosbridge migration (f70a50a, #124, 2026-07-24):
> the subscription path (`ims/frontend/src/socket.js`) and the `ims/server` that
> decoded these messages were deleted, and the dashboard now reads ROS topics
> through rosbridge instead — nothing connects to this client. The wire contract
> lives in `utils/src/messages.py`, which is deprecated for the same reason.
>
> It is also still skeletal and has never worked: `send_attitude()`,
> `send_position()`, `send_camera()`, `send_health()` and `send_log()` are empty
> stubs, and only `send_status()` actually sends.

WebSocket client that was to stream airside telemetry to the IMS ground station.

## Architecture

- `comms.py` — `AirsideComms` class. Was to manage the WebSocket connection and expose `send_attitude()`, `send_position()`, `send_camera()` etc.
