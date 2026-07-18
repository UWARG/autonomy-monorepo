import type { StatusMessage } from '../types';

/**
 * Extract waypoint progress from the free-text status line. Matches "WP <n> / <total>"
 * leniently (any whitespace around the slash, case-insensitive). Returns null if
 * the format isn't present or is nonsensical (total <= 0, current > total) 
 */
function parseWaypoints(text?: string): { current: number; total: number } | null {
  if (!text) return null;
  const m = text.match(/WP\s+(\d+)\s*\/\s*(\d+)/i);
  if (!m) return null;
  const current = Number(m[1]);
  const total = Number(m[2]);
  if (!Number.isFinite(current) || !Number.isFinite(total)) return null;
  if (total <= 0 || current < 0 || current > total) return null;
  return { current, total };
}

function statePill(state?: string): { className: string; label: string } {
  const s = (state ?? '').toUpperCase();
  if (!state) return { className: 'pill bg-edge text-ink-3', label: 'NO DATA' };
  if (s.includes('RUN')) return { className: 'pill-ok', label: state };
  if (s.includes('PAUSE')) return { className: 'pill-warn', label: state };
  if (s.includes('ABORT') || s.includes('FAIL')) return { className: 'pill-bad', label: state };
  return { className: 'pill bg-edge text-ink-3', label: state };
}

/**
 * Mission command buttons. There is no command/send channel yet (socket.js) is
 * receive-only and airside has no command handler  so these are disabled. When
 * a send path exists, wire onClick and drop `disabled`.
 */
function CommandButton({
  label,
  tone,
}: {
  label: string;
  tone: 'warn' | 'ok' | 'bad';
}) {
  const toneClass =
    tone === 'warn'
      ? 'text-warn'
      : tone === 'bad'
        ? 'text-bad'
        : 'text-ink-2';
  return (
    <button
      type="button"
      disabled
      title="Mission commands aren't wired yet"
      className={`flex-1 rounded-md border border-edge bg-card/60 py-2 text-[13px] font-semibold ${toneClass} opacity-40 cursor-not-allowed`}
    >
      {label}
    </button>
  );
}

export default function ScriptWidget({
  status,
}: {
  status?: StatusMessage;
}) {
  const pill = statePill(status?.state);
  const wp = parseWaypoints(status?.text);
  const pct = wp ? (wp.current / wp.total) * 100 : null;

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <div className="flex items-baseline gap-2">
          <h2 className="widget-label">Mission Script</h2>
          <span className="font-mono text-[13px] text-ink">
            {status?.task ?? '\u2014'}
          </span>
        </div>
        <span className={pill.className}>{pill.label}</span>
      </header>

      <p className={`mt-3 text-[13px] ${status ? 'text-ink-2' : 'text-ink-3'}`}>
        {status?.text ?? 'Awaiting mission status'}
      </p>

      {pct != null && wp && (
        <div className="mt-3">
          <div className="mb-1 flex justify-between text-[11px] text-ink-3">
            <span>WP {wp.current} / {wp.total}</span>
            <span className="font-mono">{Math.round(pct)}%</span>
          </div>
          <div className="h-2 overflow-hidden rounded-full bg-edge">
            <div
              className="h-full rounded-full bg-ok transition-[width]"
              style={{ width: `${Math.max(0, Math.min(100, pct))}%` }}
            />
          </div>
        </div>
      )}

      {/* mission commands — disabled until a send channel exists */}
      <div className="mt-auto pt-4">
        <div className="flex gap-3">
          <CommandButton label="Pause" tone="warn" />
          <CommandButton label="Resume" tone="ok" />
          <CommandButton label="Abort" tone="bad" />
        </div>
        <p className="mt-2 text-[11px] text-ink-3">
          Commands unavailable — no uplink channel yet
        </p>
      </div>
    </section>
  );
}