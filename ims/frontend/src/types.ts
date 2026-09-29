/**
 * Wire payloads mirror utils/src/messages.py. Envelope: { "type": "<tag>", "payload": {...} }.
 */

export type ConnectionStatus = 'active' | 'degraded' | 'lost';

/**
 * payload for { "type": "connection" }
 */
export interface ConnectionMessage {
  status: ConnectionStatus;
  transport: string;
  heartbeatHz: number; //Hz
}

/**
 * payload for { "type": "attitude" }
 * Mirrors AttitudePayload in utils/src/messages.py (the wire contract).
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
 */
export interface CameraMessage {
  src?: string;
  width?: number; //px
  height?: number; //px
  latencyMs?: number; 
}

/**
 * payload for { "type": "position" } — the drone's global position.
 * Mirrors PositionPayload in utils/src/messages.py.
 */
export interface PositionMessage {
  /** degrees */
  lat: number;
  lon: number;
  alt: number; // meter
}

/**
 * payload for { "type": "target" }
 * Mirrors TargetPayload in utils/src/messages.py.
 */
export interface TargetMessage {
  /** degrees */
  lat: number;
  lon: number;
  label?: string;
  tracking?: boolean;
  cluster?: number;
}

/**
 * One aircraft in { "type": "nearby_drones" }.
 * Mirrors NearbyDronePayload in utils/src/messages.py.
 */
export interface NearbyDrone {
  id: number;
  name: string;
  lat: number; // degrees
  lon: number; // degrees
  alt: number; // meter AGL
  speed: number; // m/s
  /** degrees clockwise from true north */
  direction: number;
  horizontal_keep_away: number; // meter
  vertical_keep_away: number; // meter
}

/**
 * payload for { "type": "nearby_drones" } — snapshot of all traffic, replaces the previous one.
 * Mirrors NearbyDronesPayload in utils/src/messages.py.
 */
export interface NearbyDronesMessage {
  drones: NearbyDrone[];
}

/**
 * payload for { "type": "status" } — mission/script state.
 *
 */
export interface StatusMessage {
  task: string;
  state: string; // "RUNNING" | "PAUSED" | "IDLE" | "ABORTED"
  text: string;
}

/**
 * payload for { "type": "log" }
 *
 */
export interface LogMessage {
  message: string;
}