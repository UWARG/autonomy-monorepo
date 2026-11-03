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
 *
 * PROVISIONAL
 */
export interface TargetMessage {
  /** degrees */
  lat: number;
  lon: number;
  label?: string;
  tracking?: boolean;
}

/** One aircraft from an AEAC `traffic` event. */
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

/** One complete AEAC traffic snapshot. */
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

/** rosbridge shape of the airside_interfaces/srv/CaptureImage response. */
export interface CaptureImageResponse {
  success: boolean;
  message: string;
  header: { stamp: { sec: number; nanosec: number }; frame_id: string };
  /** sensor_msgs/CompressedImage; `data` is base64 JPEG, as rosbridge encodes uint8[]. */
  image: { format: string; data: string };
  location: { lat: number; lon: number; alt: number };
  imu: { orientation: { x: number; y: number; z: number; w: number } };
}

/** One triggered photo, kept for the crew to review while counting deer. */
export interface Capture {
  id: string;
  time: string;
  imageUrl: string;
}
