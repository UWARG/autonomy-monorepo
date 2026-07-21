# airside_comms

> [!WARNING]
> This is a skeletal project and has no functionality yet.

 WebSocket client that streams airside telemetry to the IMS ground station.

## Architecture

- `comms.py` — `AirsideComms` class. Manages the WebSocket connection and exposes `send_attitude()`, `send_position()`, `send_camera()` etc.
