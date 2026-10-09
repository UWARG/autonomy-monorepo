/**
 * Payload shapes the widgets consume. The envelope form described the deleted
 * WebSocket contract (utils/src/messages.py); live data now arrives as ROS
 * messages over rosbridge and the widgets map those into these shapes.
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
 * Attitude in the widgets' own terms; AttitudeWidget derives it from
 * geometry_msgs/PoseStamped.
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
 * Global position; TargetWidget reads sensor_msgs/NavSatFix instead.
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