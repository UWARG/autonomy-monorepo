import type { ConnectionMessage, StatusMessage } from '../types';

/**
 * Header telemetry strip. Only renders the fields the wire contract actually
 * supports today:
 *   LINK    <- connection.status  (active/degraded/lost)
 *   MISSION <- parsed from status.text ("WP n / total")
 *
 * BATTERY and GPS from the mock are intentionally omitted: utils/messages.py
 * has no battery or GPS message, so they'd be invented/empty. Add them here
 * once airside defines those messages.
 */

const DASH = '\u2014';

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

const LINK_TONE: Record<string, string> = {
  active: 'text-ok',
  degraded: 'text-warn',
  lost: 'text-bad',
};

const LINK_LABEL: Record<string, string> = {
  active: 'NOMINAL',
  degraded: 'DEGRADED',
  lost: 'LOST',
};

function Stat({
  label,
  value,
  tone = 'text-ink',
}: {
  label: string;
  value: string;
  tone?: string;
}) {
  return (
    <div className="flex flex-col items-end leading-tight">
      <span className="text-[10px] font-semibold uppercase tracking-wider text-ink-3">
        {label}
      </span>
      <span className={`font-mono text-[13px] font-semibold tabular-nums ${tone}`}>
        {value}
      </span>
    </div>
  );
}

export default function HeaderStatus({
  connection,
  status,
}: {
  connection?: ConnectionMessage;
  status?: StatusMessage;
}) {
  const linkTone = connection ? (LINK_TONE[connection.status] ?? 'text-ink') : 'text-ink-3';
  const linkLabel = connection ? (LINK_LABEL[connection.status] ?? connection.status.toUpperCase()) : DASH;

  const wp = parseWaypoints(status?.text);
  const mission = wp ? `WP ${wp.current}/${wp.total}` : DASH;

  return (
    <div className="flex items-center gap-6">
      <Stat label="Link" value={linkLabel} tone={linkTone} />
      <Stat label="Mission" value={mission} />
    </div>
  );
}