export const ROS_URL = `ws://${window.location.hostname}:9090`;
export const STALE_AFTER_MS = 5000;
export const RECONNECT_DELAY_MS = 3000;

// IMS subscribes to AEAC traffic on its own receive-only connection. Put the
// team token in ims/frontend/.env.local (gitignored) as VITE_AEAC_CONNECTION_TOKEN.
export const AEAC_WEBSOCKET_URL: string =
  import.meta.env.VITE_AEAC_WEBSOCKET_URL || 'wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod';
export const AEAC_TOKEN: string = import.meta.env.VITE_AEAC_CONNECTION_TOKEN ?? '';

// Default telemetry topics for session recording
export const RECORDED_TOPICS: string[] = [
  '/heartbeat',
  '/mavros/state',
  '/mavros/local_position/pose',
  '/mavros/global_position/global',
  '/mavros/global_position/rel_alt',
  '/mavros/imu/data',
  '/capture/target_location',
];

// Only replay approved telemetry to avoid interfering with live flight systems
export const REPLAY_TOPICS: string[] = ['/mavros/local_position/pose'];

// Block topics that can issue vehicle control commands
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

// Replay only allowlisted topics that are not vehicle commands
export const isReplayableTopic = (name: string): boolean =>
  REPLAY_TOPICS.includes(name) && !isCommandTopic(name);
