import { useSyncExternalStore } from 'react';
import type { LinkStatus } from '../link';
import { getLinkSnapshot, subscribeLink } from '../link';

const DASH = '\u2014';

const STATUS_PILL: Record<LinkStatus, { className: string; label: string }> = {
  active: { className: 'pill-ok', label: 'ACTIVE' },
  degraded: { className: 'pill-warn', label: 'DEGRADED' },
  lost: { className: 'pill-bad', label: 'LOST' },
};

const STATUS_SUMMARY: Record<LinkStatus, string> = {
  active: 'MAVLink heartbeat nominal',
  degraded: 'Heartbeat irregular — check link quality',
  lost: 'No heartbeat — drone unreachable',
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

export default function ConnectionWidget() {
  const { status, connected, armed, mode, heartbeatHz } = useSyncExternalStore(
    subscribeLink,
    getLinkSnapshot,
  );

  const pill = status
    ? STATUS_PILL[status]
    : { className: 'pill bg-edge text-ink-3', label: 'NO DATA' };

  const summary = status
    ? STATUS_SUMMARY[status]
    : 'Awaiting first heartbeat';

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">Connection</h2>
        <span className={`${pill.className} ${status ? '' : 'bg-edge text-ink-3'}`}>
          <span
            className={`status-dot ${status ? 'bg-current' : 'bg-ink-3'}`}
            aria-hidden="true"
          />
          {pill.label}
        </span>
      </header>

      <p className={`mt-2 text-[13px] ${status ? 'text-ink-2' : 'text-ink-3'}`}>
        {summary}
      </p>

      <dl className="mt-3 flex flex-col gap-1.5">
        <Row label="Mode" value={mode ?? DASH} />
        <Row
          label="Armed"
          value={armed == null ? DASH : armed ? 'ARMED' : 'DISARMED'}
          tone={armed ? 'text-ok' : 'text-ink'}
        />
        <Row
          label="MAVROS"
          value={connected == null ? DASH : connected ? 'connected' : 'disconnected'}
          tone={connected === false ? 'text-warn' : 'text-ink'}
        />
        <Row
          label="Heartbeat"
          value={heartbeatHz != null ? `${heartbeatHz.toFixed(1)} Hz` : DASH}
        />
      </dl>
    </section>
  );
}
