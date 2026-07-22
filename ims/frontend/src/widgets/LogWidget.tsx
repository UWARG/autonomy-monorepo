import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';

export interface LogEntry {
  raw: unknown;
  t: number;
}

const LOG_MAX = 200;

interface PoseStamped {
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}

type Severity = 'INF' | 'WRN' | 'ERR';

const SEV_TONE: Record<Severity, string> = {
  INF: 'text-accent',
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
      setEntries((prev) => [...prev, { raw: message, t: Date.now() }].slice(-LOG_MAX));
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
            const severity: Severity = 'INF';
            return (
              <div key={`${e.t}-${i}`} className="flex gap-2">
                <span className="shrink-0 text-ink-3">
                  {new Date(e.t).toLocaleTimeString('en-GB', { hour12: false })}
                </span>
                <span className={`shrink-0 font-semibold ${SEV_TONE[severity]}`}>{severity}</span>
                <span className="break-all text-ink-2">{JSON.stringify(e.raw)}</span>
              </div>
            );
          })
        )}
      </div>
    </section>
  );
}
