/**
 * DEPRECATED (status, not a per-symbol compatibility marker) — orphaned by the
 * rosbridge migration (f70a50a, #124, 2026-07-24).
 *
 * These were to mirror the WebSocket payloads in utils/src/messages.py, which
 * carries the same status. The subscription path (socket.js) and the ims/server
 * that decoded them were deleted in #124, so no message of any shape below can
 * reach the frontend: App.tsx mounts these widgets without props and they render
 * NO DATA.
 *
 * Live telemetry arrives over rosbridge (ws://<host>:9090) via roslib instead —
 * AttitudeWidget (/mavros/local_position/pose) and LogWidget (/heartbeat) are the
 * only widgets wired to it.
 *
 * This file was never an accurate mirror of utils/src/messages.py:
 *   - ConnectionMessage and TargetMessage have no Python counterpart at all.
 *   - Python has HealthMessage (`healthy`), which has no interface here.
 *   - CameraPayload is an empty struct and encode_camera() has no return statement.
 *   - AttitudePayload requires rollspeed/pitchspeed/yawspeed; here they are optional.
 *   - these interfaces place payload fields at the top level, not under `payload`.
 *
 * Kept only because removing the contract spans utils/, airside_comms/ and the
 * widgets; see the deprecation note in utils/src/messages.py.
 */

export type ConnectionStatus = 'active' | 'degraded' | 'lost';

/**
 * payload for { "type": "connection" }
 *
 * Deprecated — no producer: the Python contract has no connection message.
 */
export interface ConnectionMessage {
  status: ConnectionStatus;
  transport: string;
  heartbeatHz: number; //Hz
}

/**
 * payload for { "type": "attitude" }
 * Was to mirror AttitudePayload in utils/src/messages.py (the wire contract).
 *
 * Deprecated — use a ROS topic (e.g. /mavros/imu/data) through rosbridge.
 */
export interface AttitudeMessage {
  /** radians;  */
  roll: number;
  pitch: number;
  yaw: number;
  rollspeed?: number;
  pitchspeed?: number;
  yawspeed?: number; 
}

/**
 * payload for { "type": "camera" }
 *
 * Deprecated — the Python CameraPayload is empty and encode_camera() is a stub.
 */
export interface CameraMessage {
  src?: string;
  width?: number; //px
  height?: number; //px
  latencyMs?: number; 
}

/**
 * payload for { "type": "position" } — the drone's global position.
 * Was to mirror PositionPayload in utils/src/messages.py.
 *
 * Deprecated — use a ROS topic (e.g. /mavros/global_position/global) through rosbridge.
 */
export interface PositionMessage {
  /** degrees */
  lat: number;
  lon: number;
  alt: number; // meter
}

/**
 * payload for { "type": "target" }
 *
 * PROVISIONAL 
 *
 * Deprecated — no producer: the Python contract has no target message.
 */
export interface TargetMessage {
  /** degrees */
  lat: number; 
  lon: number;
  label?: string; 
  tracking?: boolean;
}

/**
 * payload for { "type": "status" } — mission/script state.
 *
 * Deprecated — use /mavros/state through rosbridge (mavros_msgs/State).
 */
export interface StatusMessage {
  task: string;
  state: string; // "RUNNING" | "PAUSED" | "IDLE" | "ABORTED"
  text: string;
}

/**
 * payload for { "type": "log" }
 *
 * Deprecated — use a ROS topic (e.g. /heartbeat, /rosout) through rosbridge.
 */
export interface LogMessage {
  message: string;
}
