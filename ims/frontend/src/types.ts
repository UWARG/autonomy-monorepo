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

/** AEAC flight modes (CONOPS 5.2.3 item 3.b.vii). */
export type AeacFlightMode =
  | 'off'
  | 'idle'
  | 'link-lost'
  | 'failsafe'
  | 'armed-pilot'
  | 'armed-automatic';

/**
 * The packet the relay sent to AEAC, in AEAC's own camelCase schema.
 * Built in ims/server/telemetry.py.
 */
export interface AeacTelemetryPacket {
  uavId: string;
  unixTime: number; // seconds, ground station clock
  latitude: number;
  longitude: number;
  altitudeAGL: number; // meter
  horizontalPositionAccuracy: number;
  verticalPositionAccuracy: number;
  batteryPercentage: number | null;
  mode: AeacFlightMode;
  telemetryLinkStatus: number; // 0-1
  rcLinkStatus: number; // 0-1
}

/**
 * payload for { "type": "telemetry_sent" } — one per packet actually sent to AEAC.
 * Mirrors TelemetrySentPayload in utils/src/messages.py.
 */
export interface TelemetrySentMessage {
  packet: AeacTelemetryPacket;
}

/**
 * payload for { "type": "aeac_ack" } — AEAC's checks on a packet it acknowledged.
 * Mirrors AeacAckPayload in utils/src/messages.py.
 */
export interface AeacAckMessage {
  unix_time: number; // the acknowledged packet's unixTime
  inside_boundary: boolean;
  too_close_to_traffic: boolean;
}

/**
 * payload for { "type": "aeac_infraction" } — sent by AEAC on each new infraction.
 * Mirrors AeacInfractionPayload in utils/src/messages.py.
 */
export interface AeacInfractionMessage {
  last_infraction: string;
  /** Cumulative per-type counts for this UAV, across sessions. */
  counts: Record<string, number>;
  armed_seconds: number;
  received_at: number; // unix seconds, relay clock
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