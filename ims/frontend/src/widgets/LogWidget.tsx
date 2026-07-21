
export interface LogEntry {
  message: string;
  t: number;
}

type Severity = 'INF' | 'WRN' | 'ERR';

/**
 * Pull an optional leading "HH:MM:SS" and a severity token (INF/WRN/ERR) out of
 * the raw message. Airside may format lines like "04:19:35 WRN Wind gust...".
 * Anything not present falls back: no embedded time -> client receipt time,
 * no severity -> INF.
 */
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

export default function LogWidget({
  entries = [],
}: {
  entries?: LogEntry[];
}) {
  const rows = [...entries].reverse(); // newest first

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