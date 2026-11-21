import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';
import type { MissionStatus } from '../types';

/** Latched by the engine, at most every 0.5 s. */
const STATUS_TOPIC = 'engine/status';
const STATUS_STALE_MS = 3000;

export interface MissionStatusState {
  status?: MissionStatus;
  /** True until a status arrives, and again if the engine stops ticking. */
  stale: boolean;
}

/** The engine's mission progress, from airside over rosbridge. */
export default function useMissionStatus(): MissionStatusState {
  const [status, setStatus] = useState<{ value: MissionStatus; at: number }>();
  const [now, setNow] = useState(() => Date.now());

  useEffect(() => {
    const topic = new ROSLIB.Topic<MissionStatus>({
      ros,
      name: STATUS_TOPIC,
      messageType: 'airside_interfaces/MissionStatus',
    });
    const onStatus = (msg: MissionStatus) => setStatus({ value: msg, at: Date.now() });
    topic.subscribe(onStatus);
    const tick = setInterval(() => setNow(Date.now()), 1000);
    return () => {
      clearInterval(tick);
      topic.unsubscribe(onStatus);
    };
  }, []);

  return { status: status?.value, stale: !status || now - status.at > STATUS_STALE_MS };
}
