import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';
import { MISSION_STEPS, humanize, stepIndex } from '../mission';
import type { MissionStatusState } from '../hooks/useMissionStatus';

/** engine/behaviors/safety_supervisor.py CheckAbort: any message lands the drone where it is. */
const ABORT_TOPIC = '/commander/abort';
/** How long the confirm prompt waits for the second click. */
const CONFIRM_TIMEOUT_MS = 4000;

const abortTopic = new ROSLIB.Topic({ ros, name: ABORT_TOPIC, messageType: 'std_msgs/Empty' });

function statePill(mission: MissionStatusState): { className: string; label: string } {
  const { status, stale } = mission;
  if (!status) return { className: 'pill bg-edge text-ink-3', label: 'NO DATA' };
  if (stale) return { className: 'pill-warn', label: 'STALE' };
  switch (status.state) {
    case 'RUNNING':
      return { className: 'pill-ok', label: 'RUNNING' };
    case 'PAUSED':
      return { className: 'pill-warn', label: 'PILOT CONTROL' };
    case 'ABORTED':
      return { className: 'pill-bad', label: 'ABORTED' };
    case 'COMPLETE':
      return { className: 'pill-accent', label: 'COMPLETE' };
    default:
      return { className: 'pill bg-edge text-ink-3', label: status.state || 'UNKNOWN' };
  }
}

function Progress({ label, value, total }: { label: string; value: number; total: number }) {
  const pct = total > 0 ? Math.max(0, Math.min(100, (value / total) * 100)) : 0;
  return (
    <div>
      <div className="mb-1 flex justify-between text-[11px] text-ink-3">
        <span>{label}</span>
        <span className="font-mono">
          {value}/{total}
        </span>
      </div>
      <div className="h-1.5 overflow-hidden rounded-full bg-edge">
        <div className="h-full rounded-full bg-ok transition-[width]" style={{ width: `${pct}%` }} />
      </div>
    </div>
  );
}

/** Two-step: the first click asks, the second (within a few seconds) publishes the abort. */
function AbortButton({ disabled }: { disabled: boolean }) {
  const [confirming, setConfirming] = useState(false);

  useEffect(() => {
    if (!confirming) return;
    const timer = setTimeout(() => setConfirming(false), CONFIRM_TIMEOUT_MS);
    return () => clearTimeout(timer);
  }, [confirming]);

  const onClick = () => {
    if (!confirming) {
      setConfirming(true);
      return;
    }
    abortTopic.publish({});
    setConfirming(false);
  };

  return (
    <button
      type="button"
      onClick={onClick}
      disabled={disabled}
      title="Engine lands immediately where it is and stops the mission"
      className={`w-full rounded-md border py-2 text-[13px] font-semibold disabled:cursor-not-allowed disabled:opacity-40 ${
        confirming ? 'border-bad bg-bad text-app' : 'border-bad/60 bg-bad-dim text-bad hover:border-bad'
      }`}
    >
      {confirming ? 'Click again to abort and land' : 'Abort mission'}
    </button>
  );
}

export default function MissionWidget({ mission, linked }: { mission: MissionStatusState; linked: boolean }) {
  const { status, stale } = mission;
  const pill = statePill(mission);
  const current = stepIndex(status);
  const finished = status?.state === 'COMPLETE' || status?.state === 'ABORTED';

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">Mission</h2>
        <span className={pill.className}>{pill.label}</span>
      </header>

      <ol className="mt-3 grid grid-cols-6 gap-1">
        {MISSION_STEPS.map((step, i) => {
          const done = current > i || (status?.state === 'COMPLETE' && i === current);
          const active = i === current && !done;
          return (
            <li key={step} className="flex flex-col gap-1">
              <span
                className={`h-1 rounded-full ${
                  done ? 'bg-ok' : active ? (status?.state === 'PAUSED' ? 'bg-warn' : 'bg-accent') : 'bg-edge'
                }`}
              />
              <span
                className={`text-[10px] font-semibold uppercase tracking-wide ${
                  active ? 'text-ink' : done ? 'text-ink-2' : 'text-ink-3'
                }`}
              >
                {step}
              </span>
            </li>
          );
        })}
      </ol>

      <div className={`mt-3 ${stale ? 'opacity-50' : ''}`}>
        <p className="text-[14px] font-semibold text-ink">
          {status ? humanize(status.phase || status.state) : 'Waiting for engine'}
        </p>
        {status?.behaviour && (
          <p className="font-mono text-[11px] text-ink-3">{status.behaviour}</p>
        )}
        {status?.state === 'PAUSED' && (
          <p className="mt-1 text-[12px] text-warn">Pilot has control. Select GUIDED to resume.</p>
        )}
        {status?.last_message && (
          <p className="mt-2 text-[12px] text-ink-2">{status.last_message.replace(/^ENG:\s*/, '')}</p>
        )}
      </div>

      {status?.phase === 'Lapping' && status.lap_count > 0 && (
        <div className="mt-3 flex flex-col gap-2">
          <Progress label="Lap" value={status.lap} total={status.lap_count} />
          {status.waypoint_count > 0 && (
            <Progress label="Waypoint" value={status.waypoint} total={status.waypoint_count} />
          )}
        </div>
      )}

      <div className="mt-auto pt-4">
        <AbortButton disabled={!linked || !status || finished} />
      </div>
    </section>
  );
}
