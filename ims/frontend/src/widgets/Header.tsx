import type { LinkState } from '../hooks/useLink';
import type { MissionStatusState } from '../hooks/useMissionStatus';
import { missionSummary } from '../mission';

/** Header telemetry strip: link health, flight controller mode, mission progress. */

const DASH = '—';

const LINK_TONE: Record<LinkState['status'], string> = {
  active: 'text-ok',
  degraded: 'text-warn',
  lost: 'text-bad',
};

const LINK_LABEL: Record<LinkState['status'], string> = {
  active: 'NOMINAL',
  degraded: 'DEGRADED',
  lost: 'LOST',
};

const MISSION_TONE: Record<string, string> = {
  PAUSED: 'text-warn',
  ABORTED: 'text-bad',
  COMPLETE: 'text-accent',
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

export default function HeaderStatus({ link, mission }: { link: LinkState; mission: MissionStatusState }) {
  const fcu = link.mode ? `${link.mode}${link.armed ? ' · ARMED' : ''}` : DASH;
  const summary = missionSummary(mission.status);
  const missionValue = mission.status?.state === 'PAUSED' ? 'PILOT' : (summary ?? DASH);
  const missionTone = !summary || mission.stale ? 'text-ink-3' : (MISSION_TONE[mission.status!.state] ?? 'text-ink');

  return (
    <div className="flex items-center gap-6">
      <Stat label="Link" value={LINK_LABEL[link.status]} tone={LINK_TONE[link.status]} />
      <Stat label="FCU" value={fcu} tone={link.armed ? 'text-warn' : 'text-ink'} />
      <Stat label="Mission" value={missionValue} tone={missionTone} />
    </div>
  );
}
