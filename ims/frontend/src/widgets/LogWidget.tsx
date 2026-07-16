import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';

export interface LogEntry {
  message: string;
  t: number;
}

const LOG_MAX = 200;
const DEG = 180 / Math.PI;

interface PoseStamped {
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}

function quaternionToEulerDeg(q: { w: number; x: number; y: number; z: number }): {
  roll: number;
  pitch: number;
  yaw: number;
} {
  const sinrCosp = 2 * (q.w * q.x + q.y * q.z);
  const cosrCosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = Math.max(-1, Math.min(1, 2 * (q.w * q.y - q.z * q.x)));
  const pitch = Math.asin(sinp);

  const sinyCosp = 2 * (q.w * q.z + q.x * q.y);
  const cosyCosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  return { roll: roll * DEG, pitch: pitch * DEG, yaw: yaw * DEG };
}

type Severity = 'INF' | 'WRN' | 'ERR';

function parseLine(message: string, t: number): {
  time: string;
  severity: Severity;
  body: string;
} {
  let rest = message.trim();

  const timeMatch = rest.match(/^(\d{2}:\d{2}:\d{2})\s+/);
  let time: string;
  if (timeMatch) {
    time = timeMatch[1];
    rest = rest.slice(timeMatch[0].length);
  } else {
    time = new Date(t).toLocaleTimeString('en-GB', { hour12: false });
  }

  const sevMatch = rest.match(/^(INF|WRN|ERR)\s+/i);
  let severity: Severity = 'INF';
  if (sevMatch) {
    severity = sevMatch[1].toUpperCase() as Severity;
    rest = rest.slice(sevMatch[0].length);
  }

  return { time, severity, body: rest };
}

const SEV_TONE: Record<Severity, string> = {
  INF: 'text-ink-3',
  WRN: 'text-warn',
  ERR: 'text-bad',
};

export default function LogWidget() {
  const [entries, setEntries] = useState<LogEntry[]>([]);

  useEffect(() => {
    const poseTopic = new ROSLIB.Topic<PoseStamped>({
      ros,
      name: '/mavros/local_position/pose',
      messageType: 'geometry_msgs/PoseStamped',
    });

    const onPose = (message: PoseStamped) => {
      const { x, y, z } = message.pose.position;
      const { roll, pitch, yaw } = quaternionToEulerDeg(message.pose.orientation);
      const line =
        `pos x=${x.toFixed(2)} y=${y.toFixed(2)} z=${z.toFixed(2)} | ` +
        `att roll=${roll.toFixed(1)} pitch=${pitch.toFixed(1)} yaw=${yaw.toFixed(1)}`;
      setEntries((prev) => [...prev, { message: line, t: Date.now() }].slice(-LOG_MAX));
    };

    poseTopic.subscribe(onPose);
    return () => poseTopic.unsubscribe(onPose);
  }, []);

  const rows = [...entries].reverse(); 

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-hidden p-4">
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">Telemetry Log</h2>
        <span className="text-[11px] text-ink-3">live · newest first</span>
      </header>

      <div className="mt-3 flex-1 overflow-y-auto font-mono text-[12px] leading-relaxed">
        {rows.length === 0 ? (
          <p className="text-[13px] text-ink-3">Awaiting log messages</p>
        ) : (
          rows.map((e, i) => {
            const { time, severity, body } = parseLine(e.message, e.t);
            return (
              <div key={`${e.t}-${i}`} className="flex gap-2">
                <span className="text-ink-3">{time}</span>
                <span className={`${SEV_TONE[severity]} font-semibold`}>{severity}</span>
                <span className="text-ink-2">{body}</span>
              </div>
            );
          })
        )}
      </div>
    </section>
  );
}
