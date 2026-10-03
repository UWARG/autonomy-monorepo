export const ROS_URL = `ws://${window.location.hostname}:9090`;
export const RECONNECT_DELAY_MS = 3000;
export const MAX_RECONNECT_ATTEMPTS = 10;

/**
 * Scalar telemetry recorded by default. Image topics (`camera/image_raw` at
 * 50 Hz) are deliberately excluded — they are megabytes per second and do not
 * survive JSONL serialization cleanly.
 */
export const RECORDED_TOPICS: string[] = [
  '/heartbeat',
  '/mavros/state',
  '/mavros/local_position/pose',
  '/mavros/global_position/global',
  '/mavros/global_position/rel_alt',
  '/mavros/imu/data',
];

/**
 * Replay must never publish to a topic matching these: they command the
 * vehicle, so replaying a recorded one could move a live aircraft. Blocked
 * frames are counted and surfaced, not silently dropped.
 */
export const COMMAND_TOPIC_PATTERNS: RegExp[] = [
  /(^|\/)setpoint/i, // mavros/setpoint_raw/*, mavros/setpoint_position/*
  /(^|\/)cmd/i, // /cmd_vel, mavros/cmd/*
  /(^|\/)command/i,
  /(^|\/)rc\/in/i, // injecting RC channels
  /statustext\/send/i,
  /trigger_post_processing/i,
];

export const isCommandTopic = (name: string): boolean =>
  COMMAND_TOPIC_PATTERNS.some((re) => re.test(name));
