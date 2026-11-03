import type { LinkState } from '../hooks/useLink';
import type { ConnectionStatus } from '../types';

const DASH = '—';

const STATUS_PILL: Record<ConnectionStatus, { className: string; label: string }> = {
  active: { className: 'pill-ok', label: 'ACTIVE' },
  degraded: { className: 'pill-warn', label: 'DEGRADED' },
  lost: { className: 'pill-bad', label: 'LOST' },
};

function Row({
  label,
  value,
  tone = 'text-ink',
}: {
  label: string;
  value: string;
  tone?: string;
}) {
  return (
    <div className="flex items-baseline justify-between gap-4">
      <dt className="text-[13px] text-ink-3">{label}</dt>
      <dd className={`font-mono text-[13px] tabular-nums ${tone}`}>{value}</dd>
    </div>
  );
}

function heartbeatText(link: LinkState): { value: string; tone: string } {
  if (link.heartbeatAgeMs == null) return { value: DASH, tone: 'text-ink-3' };
  if (link.status === 'lost' && link.rosbridge) {
    return { value: `lost ${Math.round(link.heartbeatAgeMs / 1000)} s`, tone: 'text-bad' };
  }
  return { value: link.heartbeatHz != null ? `${link.heartbeatHz.toFixed(1)} Hz` : 'ok', tone: 'text-ink' };
}

export default function ConnectionWidget({ link }: { link: LinkState }) {
  const pill = STATUS_PILL[link.status];
  const heartbeat = heartbeatText(link);

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">Connection</h2>
        <span className={pill.className}>
          <span className="status-dot bg-current" aria-hidden="true" />
          {pill.label}
        </span>
      </header>

      <p className="mt-2 text-[13px] text-ink-2">{link.summary}</p>

      <dl className="mt-3 flex flex-col gap-1.5">
        <Row
          label="rosbridge"
          value={link.rosbridge ? 'connected' : 'disconnected'}
          tone={link.rosbridge ? 'text-ok' : 'text-bad'}
        />
        <Row label="Airside heartbeat" value={heartbeat.value} tone={heartbeat.tone} />
        <Row
          label="Flight controller"
          value={link.fcuConnected == null ? DASH : link.fcuConnected ? 'connected' : 'no link'}
          tone={link.fcuConnected == null ? 'text-ink-3' : link.fcuConnected ? 'text-ok' : 'text-bad'}
        />
        <Row label="Mode" value={link.mode ?? DASH} tone={link.mode === 'GUIDED' ? 'text-accent' : 'text-ink'} />
        <Row
          label="Armed"
          value={link.armed == null ? DASH : link.armed ? 'ARMED' : 'disarmed'}
          tone={link.armed ? 'text-warn' : 'text-ink'}
        />
      </dl>
    </section>
  );
}
