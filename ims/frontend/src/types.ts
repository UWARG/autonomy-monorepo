/**
 * Payload shapes the widgets consume. The envelope form described the deleted
 * WebSocket contract (utils/src/messages.py); live data now arrives as ROS
 * messages over rosbridge and the widgets map those into these shapes.
 */

export type ConnectionStatus = 'active' | 'degraded' | 'lost';

/**
 * Link state as the Connection panel and header strip present it.
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
 * Camera frame metadata for the Camera panel (no pixels).
 */
export interface CameraMessage {
  src?: string;
  width?: number; //px
  height?: number; //px
  latencyMs?: number; 
}

/**
 * Global position, currently supplied to TargetWidget as a prop; #191 wires the
 * widget to sensor_msgs/NavSatFix, which does not carry this shape.
 */
export interface PositionMessage {
  /** degrees */
  lat: number;
  lon: number;
  alt: number; // meter
}

/**
 * A detected target for the Position / Target panel.
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
 * Mission/script state for the Mission Script panel.
 */
export interface StatusMessage {
  task: string;
  state: string; // "RUNNING" | "PAUSED" | "IDLE" | "ABORTED"
  text: string;
}