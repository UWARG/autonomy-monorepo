/**
 * Ground-station link health, shared by the Header and Connection widgets.
 *
 * Both panels must agree on whether the aircraft is being heard, so this module
 * owns the single subscription to each of /mavros/state and /heartbeat, and
 * derives the status once. Subscriptions start on the first listener and stop
 * with the last, so StrictMode's double-effect cannot leave a duplicate behind.
 */
import ROSLIB from 'roslib';
import { ros } from './ros.js';
import { TOPICS } from './topics.ts';

export type LinkStatus = 'active' | 'degraded' | 'lost';

export interface LinkSnapshot {
  /** null until anything arrives, so panels can tell "no data" from "dropped". */
  status: LinkStatus | null;
  /** MAVROS ↔ flight-controller link, from /mavros/state. */
  connected: boolean | null;
  armed: boolean | null;
  /** Flight-controller mode, e.g. "GUIDED". */
  mode: string | null;
  /** Measured heartbeat rate; null until two beats land in the window. */
  heartbeatHz: number | null;
}

interface MavrosState {
  connected: boolean;
  armed: boolean;
  mode: string;
}

/** heartbeat_node publishes at 1 Hz, so a few seconds of silence is a dead link. */
const SILENCE_MS = 3000;
/** Window the heartbeat rate is averaged over. */
const RATE_WINDOW_MS = 5000;
/** Below this the link counts as degraded. */
const DEGRADED_HZ = 0.7;
/** Re-evaluated on a timer so silence is noticed without new messages. */
const TICK_MS = 500;

const EMPTY: LinkSnapshot = {
  status: null,
  connected: null,
  armed: null,
  mode: null,
  heartbeatHz: null,
};

let link: LinkSnapshot = EMPTY;
const listeners = new Set<() => void>();
let beats: number[] = [];
let state: MavrosState | null = null;
let lastStateAt = 0;
let started: { unsubscribe(): void }[] = [];
let ticker: number | null = null;

function sample(): LinkSnapshot {
  const now = Date.now();
  beats = beats.filter((at) => now - at <= RATE_WINDOW_MS);
  const newest = Math.max(beats.length ? beats[beats.length - 1] : 0, lastStateAt);
  if (newest === 0) return EMPTY;

  const span = beats.length > 1 ? beats[beats.length - 1] - beats[0] : 0;
  const heartbeatHz = span > 0 ? ((beats.length - 1) * 1000) / span : null;
  const silent = now - newest > SILENCE_MS;
  const degraded = state?.connected === false || (heartbeatHz != null && heartbeatHz < DEGRADED_HZ);

  return {
    status: silent ? 'lost' : degraded ? 'degraded' : 'active',
    connected: state?.connected ?? null,
    armed: state?.armed ?? null,
    mode: state?.mode ?? null,
    heartbeatHz,
  };
}

function refresh(): void {
  const next = sample();
  const unchanged =
    next.status === link.status &&
    next.connected === link.connected &&
    next.armed === link.armed &&
    next.mode === link.mode &&
    next.heartbeatHz === link.heartbeatHz;
  if (unchanged) return;
  link = next;
  for (const listener of listeners) listener();
}

function start(): void {
  const stateTopic = new ROSLIB.Topic<MavrosState>({
    ros,
    name: TOPICS.mavrosState.name,
    messageType: TOPICS.mavrosState.type,
  });
  const heartbeatTopic = new ROSLIB.Topic({
    ros,
    name: TOPICS.heartbeat.name,
    messageType: TOPICS.heartbeat.type,
  });

  stateTopic.subscribe((message) => {
    state = message;
    lastStateAt = Date.now();
    refresh();
  });
  heartbeatTopic.subscribe(() => {
    beats.push(Date.now());
    refresh();
  });

  started = [stateTopic, heartbeatTopic];
  ticker = window.setInterval(refresh, TICK_MS);
}

function stop(): void {
  if (ticker != null) {
    window.clearInterval(ticker);
    ticker = null;
  }
  for (const topic of started) topic.unsubscribe();
  started = [];
  beats = [];
  state = null;
  lastStateAt = 0;
  link = EMPTY;
}

export function subscribeLink(listener: () => void): () => void {
  listeners.add(listener);
  if (listeners.size === 1) start();
  return () => {
    listeners.delete(listener);
    if (listeners.size === 0) stop();
  };
}

export function getLinkSnapshot(): LinkSnapshot {
  return link;
}
