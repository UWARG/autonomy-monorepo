/** Messages IMS reads from airside over rosbridge, and its own derived state. */

/** Ground link health, derived from rosbridge, airside's heartbeat and MAVROS. */
export type ConnectionStatus = 'active' | 'degraded' | 'lost';

/** A global position; `alt` is AMSL from GPS, or relative to home for setpoints and waypoints. */
export interface PositionMessage {
  /** degrees */
  lat: number;
  lon: number;
  alt: number; // meter
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

/** airside_interfaces/MissionStatus on engine/status: the engine's mission progress. */
export interface MissionStatus {
  /** STARTING, PAUSED, RUNNING, ABORTED or COMPLETE. */
  state: string;
  /** Mission step running, e.g. "Takeoff", "Lapping". */
  phase: string;
  /** Deepest running behaviour, e.g. "FlyToWaypoint". */
  behaviour: string;
  /** 1-based; 0 outside lapping. */
  lap: number;
  lap_count: number;
  /** 1-based; 0 when none. */
  waypoint: number;
  waypoint_count: number;
  last_message: string;
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
