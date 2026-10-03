import { useSyncExternalStore } from 'react';
import type { LinkStatus } from '../link';
import { getLinkSnapshot, subscribeLink } from '../link';

/**
 * Header telemetry strip, fed by the same link store as the Connection panel:
 *   LINK <- /heartbeat + /mavros/state
 *   MODE <- /mavros/state.mode
 *
 * The previous MISSION readout ("WP n / total") is gone: nothing in airside
 * publishes mission or waypoint progress, so it could only ever show a dash.
 */

const DASH = '\u2014';

const LINK_TONE: Record<LinkStatus, string> = {
  active: 'text-ok',
  degraded: 'text-warn',
  lost: 'text-bad',
};

const LINK_LABEL: Record<LinkStatus, string> = {
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

export default function HeaderStatus() {
  const { status, mode, armed } = useSyncExternalStore(subscribeLink, getLinkSnapshot);

  return (
    <div className="flex items-center gap-6">
      <Stat
        label="Link"
        value={status ? LINK_LABEL[status] : DASH}
        tone={status ? LINK_TONE[status] : 'text-ink-3'}
      />
      <Stat label="Mode" value={mode ?? DASH} />
      <Stat
        label="Armed"
        value={armed == null ? DASH : armed ? 'YES' : 'NO'}
        tone={armed ? 'text-ok' : 'text-ink-3'}
      />
    </div>
  );
}
