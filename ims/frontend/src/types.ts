/**
 * Payload shapes broadcast over the websocket (see socket.js).
 * Message envelope: { "type": "<key>", "payload": { ... } }
 */


export type ConnectionStatus = 'active' | 'degraded' | 'lost';

/** payload for { "type": "connection" } */
export interface ConnectionMessage {
  status: ConnectionStatus;
  protocol: string;
  transport: string;
  heartbeatHz: number; //Hz
  latencyMs: number; // ms
  packetLossPct: number; //0-100
  /** inbound messages per second */
  msgRate: number;
}

/**
 * payload for { "type": "attitude" }
 * Angles are radians, but converted to deg for display
 */
export interface AttitudeMessage {
  roll: number;
  pitch: number;
  yaw: number;
}