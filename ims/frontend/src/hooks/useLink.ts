import { useEffect, useState, useSyncExternalStore } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';
import type { ConnectionStatus } from '../types';

/** airside's heartbeat_node, 1 Hz. */
const HEARTBEAT_TOPIC = '/heartbeat';
/** MAVROS's view of the flight controller, ~1-2 Hz (from FCU HEARTBEAT). */
const STATE_TOPIC = 'mavros/state';

/** Heartbeats are averaged over this window for the rate readout. */
const RATE_WINDOW_MS = 5000;
/** No airside heartbeat for this long and the link is lost. */
const HEARTBEAT_LOST_MS = 3000;
/** No MAVROS state for this long and the FCU counts as unknown. */
const FCU_STALE_MS = 3000;

interface MavrosState {
  connected: boolean;
  armed: boolean;
  guided: boolean;
  mode: string;
  system_status: number;
}

export interface LinkState {
  status: ConnectionStatus;
  /** One-line explanation of `status`. */
  summary: string;
  /** IMS's WebSocket to rosbridge on airside is open. */
  rosbridge: boolean;
  /** Airside heartbeats per second over the last few seconds, if any arrived. */
  heartbeatHz: number | null;
  /** ms since the last airside heartbeat, if one ever arrived. */
  heartbeatAgeMs: number | null;
  /** MAVROS reports a live connection to the flight controller. */
  fcuConnected: boolean | null;
  armed: boolean | null;
  mode: string | null;
}

/** rosbridge open/closed, read straight from roslib so it is right even if it connected before mount. */
function subscribeRos(onChange: () => void): () => void {
  ros.on('connection', onChange);
  ros.on('close', onChange);
  return () => {
    ros.off('connection', onChange);
    ros.off('close', onChange);
  };
}

/** Health of the ground ↔ airside ↔ flight controller chain. */
export default function useLink(): LinkState {
  const rosbridge = useSyncExternalStore(subscribeRos, () => ros.isConnected);
  const [beats, setBeats] = useState<number[]>([]);
  const [fcu, setFcu] = useState<{ value: MavrosState; at: number }>();
  const [now, setNow] = useState(() => Date.now());

  useEffect(() => {
    const heartbeatTopic = new ROSLIB.Topic({ ros, name: HEARTBEAT_TOPIC, messageType: 'std_msgs/Empty' });
    const stateTopic = new ROSLIB.Topic<MavrosState>({ ros, name: STATE_TOPIC, messageType: 'mavros_msgs/State' });

    const onBeat = () => {
      const t = Date.now();
      setBeats((prev) => [...prev.filter((b) => t - b <= RATE_WINDOW_MS), t]);
    };
    const onState = (msg: MavrosState) => setFcu({ value: msg, at: Date.now() });

    heartbeatTopic.subscribe(onBeat);
    stateTopic.subscribe(onState);
    const tick = setInterval(() => setNow(Date.now()), 500);

    return () => {
      clearInterval(tick);
      heartbeatTopic.unsubscribe(onBeat);
      stateTopic.unsubscribe(onState);
    };
  }, []);

  const lastBeat = beats.length ? beats[beats.length - 1] : null;
  const heartbeatAgeMs = lastBeat != null ? now - lastBeat : null;
  const recent = beats.filter((b) => now - b <= RATE_WINDOW_MS);
  // Rate over the span actually covered, so it reads right within seconds of connecting.
  const heartbeatHz =
    recent.length >= 2 ? (recent.length - 1) / ((recent[recent.length - 1] - recent[0]) / 1000) : null;

  const fcuFresh = !!fcu && now - fcu.at <= FCU_STALE_MS;
  const fcuConnected = fcuFresh ? fcu.value.connected : null;

  let status: ConnectionStatus;
  let summary: string;
  if (!rosbridge) {
    status = 'lost';
    summary = 'No rosbridge connection to airside';
  } else if (heartbeatAgeMs == null || heartbeatAgeMs > HEARTBEAT_LOST_MS) {
    status = 'lost';
    summary = heartbeatAgeMs == null ? 'rosbridge up, waiting for airside heartbeat' : 'Airside heartbeat lost';
  } else if (!fcuConnected) {
    status = 'degraded';
    summary = fcuFresh ? 'Airside up, MAVROS has no FCU link' : 'Airside up, no MAVROS state';
  } else {
    status = 'active';
    summary = 'Airside and flight controller linked';
  }

  return {
    status,
    summary,
    rosbridge,
    heartbeatHz,
    heartbeatAgeMs,
    fcuConnected,
    armed: fcuFresh ? fcu.value.armed : null,
    mode: fcuFresh ? fcu.value.mode : null,
  };
}
