import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';

/** Engine progress messages on their way to the FCU (and any GCS) as STATUSTEXT. */
const ENGINE_TEXT_TOPIC = 'mavros/statustext/send';
/** STATUSTEXT from the flight controller: arming checks, EKF, failsafes, mode changes. */
const FCU_TEXT_TOPIC = 'mavros/statustext/recv';

const LOG_MAX = 300;

interface StatusText {
  severity: number;
  text: string;
}

export type LogLevel = 'ERR' | 'WRN' | 'INF' | 'DBG';

export interface LogEntry {
  id: number;
  /** Receipt time, ms epoch. */
  t: number;
  level: LogLevel;
  /** Who said it: ENG (airside engine), FCU (ArduPilot) or IMS (this page). */
  source: 'ENG' | 'FCU' | 'IMS';
  text: string;
}

/** MAVLink MAV_SEVERITY: 0-3 emergency..error, 4 warning, 5-6 notice/info, 7 debug. */
function levelOf(severity: number): LogLevel {
  if (severity <= 3) return 'ERR';
  if (severity === 4) return 'WRN';
  if (severity <= 6) return 'INF';
  return 'DBG';
}

let nextId = 0;

const CONNECTED_TEXT = 'Connected to airside rosbridge';
const LOST_TEXT = 'rosbridge connection lost, retrying';

/** Engine and flight-controller messages plus IMS's own link events, oldest first. */
export default function useGroundLog(): LogEntry[] {
  const [entries, setEntries] = useState<LogEntry[]>([]);

  useEffect(() => {
    const add = (entry: Omit<LogEntry, 'id' | 't'>) =>
      setEntries((prev) => [...prev, { ...entry, id: nextId++, t: Date.now() }].slice(-LOG_MAX));

    const engineTopic = new ROSLIB.Topic<StatusText>({
      ros,
      name: ENGINE_TEXT_TOPIC,
      messageType: 'mavros_msgs/StatusText',
    });
    const fcuTopic = new ROSLIB.Topic<StatusText>({
      ros,
      name: FCU_TEXT_TOPIC,
      messageType: 'mavros_msgs/StatusText',
    });
    const onEngine = (msg: StatusText) =>
      add({ source: 'ENG', level: levelOf(msg.severity), text: msg.text.replace(/^ENG:\s*/, '') });
    const onFcu = (msg: StatusText) => add({ source: 'FCU', level: levelOf(msg.severity), text: msg.text });
    // Link lines are logged only when they change the link state: every failed
    // retry also closes, and rosbridge may already be open when this mounts.
    const addLink = (level: LogLevel, text: string) =>
      setEntries((prev) => {
        const last = prev.findLast((e) => e.source === 'IMS');
        if ((last?.text ?? LOST_TEXT) === text) return prev;
        return [...prev, { id: nextId++, t: Date.now(), source: 'IMS' as const, level, text }].slice(-LOG_MAX);
      });
    const onOpen = () => addLink('INF', CONNECTED_TEXT);
    const onClose = () => addLink('WRN', LOST_TEXT);

    if (ros.isConnected) onOpen();
    engineTopic.subscribe(onEngine);
    fcuTopic.subscribe(onFcu);
    ros.on('connection', onOpen);
    ros.on('close', onClose);
    return () => {
      engineTopic.unsubscribe(onEngine);
      fcuTopic.unsubscribe(onFcu);
      ros.off('connection', onOpen);
      ros.off('close', onClose);
    };
  }, []);

  return entries;
}
