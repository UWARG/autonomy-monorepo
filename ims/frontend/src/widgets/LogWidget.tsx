import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';

type LogEntry = {
  topic: string;
  raw: unknown;
  t: number;
};

const LOG_MAX = 200;

export default function LogWidget() {
  const [entries, setEntries] = useState<LogEntry[]>([]);

  useEffect(() => {
    const topics: ROSLIB.Topic[] = [];

    const onMessage = (name: string) => (message: unknown) => {
      setEntries((prev) => [...prev, { topic: name, raw: message, t: Date.now() }].slice(-LOG_MAX));
    };

    const TOPIC_NAMES = ['/heartbeat'];
    ros.getTopics(
      ({ topics: allTopics, types }) => {
        for (const name of TOPIC_NAMES) {
          const i = allTopics.indexOf(name);
          if (i === -1) {
            console.warn(`rosapi: ${name} topic not found on the graph`);
            continue;
          }
          const topic = new ROSLIB.Topic({ ros, name, messageType: types[i] });
          topic.subscribe(onMessage(name));
          topics.push(topic);
        }
      },
      (error) => console.error('rosapi getTopics failed:', error),
    );

    return () => {
      topics.forEach((t) => t.unsubscribe());
    };
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
          rows.map((e, i) => (
            <div key={`${e.t}-${i}`} className="flex gap-2">
              <span className="shrink-0 text-ink-3">
                {new Date(e.t).toLocaleTimeString('en-GB', { hour12: false })}
              </span>
              <span className="shrink-0 font-semibold text-accent">INF</span>
              <span className="break-all text-ink-2">
                [{e.topic}] {JSON.stringify(e.raw)}
              </span>
            </div>
          ))
        )}
      </div>
    </section>
  );
}
