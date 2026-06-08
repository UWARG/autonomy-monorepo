# airside_comms

> [!WARNING]
> This is a skeletal project and has no functionality yet.

Standalone WebSocket client that streams airside telemetry to the IMS ground station. Has no ROS2 dependency — it is wrapped by a thin ROS2 node inside the `airside` workspace.

## Architecture

- `comms.py` — `AirsideComms` class. Manages the WebSocket connection and exposes `send_attitude()`, `send_position()`, `send_camera()` etc.
- `message_encoder.py` — encodes utils dataclasses into JSON strings in `{"type": ..., "payload": {...}}` format.

