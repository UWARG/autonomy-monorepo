import type { ConnectionMessage, ConnectionStatus } from '../types';

const DASH = '—';

const STATUS_PILL: Record<ConnectionStatus, { className: string; label: string }> = {
  active: { className: 'pill-ok', label: 'ACTIVE' },
  degraded: { className: 'pill-warn', label: 'DEGRADED' },
  lost: { className: 'pill-bad', label: 'LOST' },
};

const STATUS_SUMMARY: Record<ConnectionStatus, string> = {
  active: 'MAVLink heartbeat nominal',
  degraded: 'Heartbeat irregular \u2014 check link quality',
  lost: 'No heartbeat \u2014 drone unreachable',
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

export default function ConnectionWidget({
  connection,
}: {
  connection?: ConnectionMessage;
}) {
  const pill = connection
    ? STATUS_PILL[connection.status]
    : { className: 'pill', label: 'NO DATA' };

  const summary = connection
    ? STATUS_SUMMARY[connection.status]
    : 'Awaiting first heartbeat';

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">Connection</h2>
        <span
          className={`${pill.className} ${connection ? '' : 'bg-edge text-ink-3'}`}
        >
          <span
            className={`status-dot ${connection ? 'bg-current' : 'bg-ink-3'}`}
            aria-hidden="true"
          />
          {pill.label}
        </span>
      </header>

      <p className={`mt-2 text-[13px] ${connection ? 'text-ink-2' : 'text-ink-3'}`}>
        {summary}
      </p>

      <dl className="mt-3 flex flex-col gap-1.5">
        <Row label="Transport" value={connection?.transport ?? DASH} />
        <Row
          label="Heartbeat"
          value={connection ? `${connection.heartbeatHz.toFixed(1)} Hz` : DASH}
        />
      </dl>
    </section>
  );
}