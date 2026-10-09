export const ROS_URL = `ws://${window.location.hostname}:9090`;
export const RECONNECT_DELAY_MS = 3000;
export const MAX_RECONNECT_ATTEMPTS = 10;

/**
 * Scalar telemetry recorded by default, and the only topics replay may publish
 * (see {@link isReplayableTopic}). Image topics (`camera/image_raw` at 50 Hz)
 * are deliberately excluded — they are megabytes per second and do not survive
 * JSONL serialization cleanly. Anything else can still be recorded, but replay
 * refuses it.
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
 * Second line of defence behind {@link isReplayableTopic}: these command the
 * vehicle, so a recorded frame on one must never be published. Blocked frames
 * are counted and surfaced, not silently dropped.
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

/**
 * Replay publishes an allowlist, not a blocklist: a session file is JSON from
 * disk and may name any topic, so a deny-list can only ever be a guess (it
 * already missed `/mavros/actuator_control`, which MAVROS turns into
 * SET_ACTUATOR_CONTROL_TARGET). Only the telemetry the dashboard itself tracks
 * is republished; every other topic is counted as blocked.
 */
export const isReplayableTopic = (name: string): boolean =>
  RECORDED_TOPICS.includes(name) && !isCommandTopic(name);
