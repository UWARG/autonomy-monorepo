import useGroundLog, { type LogEntry, type LogLevel } from '../hooks/useGroundLog';

const LEVEL_TONE: Record<LogLevel, string> = {
  ERR: 'text-bad',
  WRN: 'text-warn',
  INF: 'text-accent',
  DBG: 'text-ink-3',
};

const SOURCE_TONE: Record<LogEntry['source'], string> = {
  ENG: 'text-ok',
  FCU: 'text-ink-2',
  IMS: 'text-ink-3',
};

export default function LogWidget() {
  const entries = useGroundLog();
  const rows = [...entries].reverse();

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-hidden p-4">
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">Log</h2>
        <span className="text-[11px] text-ink-3">engine · flight controller · newest first</span>
      </header>

      <div className="mt-3 flex-1 overflow-y-auto font-mono text-[12px] leading-relaxed">
        {rows.length === 0 ? (
          <p className="text-[13px] text-ink-3">Awaiting messages</p>
        ) : (
          rows.map((e) => (
            <div key={e.id} className="flex gap-2">
              <span className="shrink-0 text-ink-3">
                {new Date(e.t).toLocaleTimeString('en-GB', { hour12: false })}
              </span>
              <span className={`w-7 shrink-0 font-semibold ${LEVEL_TONE[e.level]}`}>{e.level}</span>
              <span className={`w-7 shrink-0 ${SOURCE_TONE[e.source]}`}>{e.source}</span>
              <span className="break-words text-ink-2">{e.text}</span>
            </div>
          ))
        )}
      </div>
    </section>
  );
}
