export const ROS_URL = `ws://${window.location.hostname}:9090`;
export const RECONNECT_DELAY_MS = 3000;
export const MAX_RECONNECT_ATTEMPTS = 10;

/**
 * Scalar telemetry recorded by default. Image topics (`camera/image_raw` at
 * 50 Hz) are deliberately excluded — they are megabytes per second and do not
 * survive JSONL serialization cleanly. Anything else may still be recorded by
 * ticking it in the picker; recording is passive.
 *
 * These are the same names `TOPICS` lists in `topics.ts` (#191), which the
 * widgets subscribe to; the two are independent today, so keep them in step (or
 * derive this from `TOPICS` once both have landed).
 */
export const RECORDED_TOPICS: string[] = [
  '/heartbeat',
  '/mavros/state',
  '/mavros/local_position/pose',
  '/mavros/global_position/global',
  '/mavros/global_position/rel_alt',
  '/mavros/imu/data',
  '/capture/target_location',
];

/**
 * The only topics replay may publish — deliberately much narrower than
 * {@link RECORDED_TOPICS}, because publishing is not the same risk as
 * recording.
 *
 * airside subscribes to most of the recorded set as inputs to mission
 * decisions: `TouchingGround` gates `ReleasePayload` on `/mavros/state.armed`
 * (see `airside/.../subtrees/dropping.py`), `FlyToWaypoint` and
 * `DescendToLandingPad` derive guided setpoints from
 * `/mavros/global_position/{global,rel_alt}`, the fusion node consumes
 * `/mavros/imu/data`. A session holds pre-arm and pre-takeoff frames, so
 * replaying those names into a live graph can release a payload in the air or
 * fly the aircraft to a stale setpoint. They stay recordable — a session is
 * evidence — but are never republished.
 *
 * `/heartbeat` is excluded for a different reason: it is the liveness beacon,
 * so replaying it would make the dashboard report a link that is not there.
 * `/capture/target_location` is excluded because `map_manager_node` logs it,
 * and replay would write invented targets into that log.
 */
export const REPLAY_TOPICS: string[] = ['/mavros/local_position/pose'];

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
 * SET_ACTUATOR_CONTROL_TARGET). Only {@link REPLAY_TOPICS} is republished;
 * every other topic is counted as blocked.
 */
export const isReplayableTopic = (name: string): boolean =>
  REPLAY_TOPICS.includes(name) && !isCommandTopic(name);
