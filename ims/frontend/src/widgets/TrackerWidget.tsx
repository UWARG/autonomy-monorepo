import useOwnTrack from '../hooks/useOwnTrack';
import { bearingDeg, haversineM } from '../geo';
import type { TrackerState } from '../hooks/useTracker';

const DASH = '—';

function Row({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-baseline justify-between gap-4">
      <dt className="text-[13px] text-ink-3">{label}</dt>
      <dd className="font-mono text-[13px] tabular-nums text-ink">{value}</dd>
    </div>
  );
}

export default function TrackerWidget({ tracker }: { tracker: TrackerState }) {
  const { sessions, ids, selected, setPicked, fixes, latest, ageMs, stale, mqttUp, dir, pickDir, canPickDir, saveError, download, reset } = tracker;
  const { position } = useOwnTrack();

  const dist = position && latest ? haversineM(position.lat, position.lon, latest.lat, latest.lon) : null;
  const brg = position && latest ? bearingDeg(position.lat, position.lon, latest.lat, latest.lon) : null;

  const pill = !latest
    ? { className: 'pill bg-edge text-ink-3', label: 'NO DATA' }
    : stale
      ? { className: 'pill-warn', label: 'STALE' }
      : { className: 'pill-ok', label: 'LIVE' };

  return (
    <section className="widget flex h-full flex-col p-4">
      <header className="flex items-center justify-between gap-2">
        <h2 className="widget-label">Tracker</h2>
        <select
          aria-label="Tracker session"
          value={selected ?? ''}
          onChange={(e) => setPicked(e.target.value)}
          className="max-w-[140px] truncate rounded bg-feed p-1 font-mono text-[11px] text-ink-2"
        >
          {ids.length === 0 && <option value="">no sessions</option>}
          {ids.map((s) => <option key={s} value={s}>{s.slice(0, 8)} ({sessions[s].length})</option>)}
        </select>
        <span className={pill.className}>{pill.label}</span>
      </header>

      <dl className="mt-3 flex flex-col gap-1.5">
        <Row label="Position" value={latest ? `${latest.lat.toFixed(6)}, ${latest.lon.toFixed(6)}` : DASH} />
        <Row label="Last fix" value={ageMs != null ? `${Math.max(0, Math.round(ageMs / 1000))} s ago` : DASH} />
        <Row
          label="From drone"
          value={dist == null ? DASH : `${dist < 100 ? dist.toFixed(1) : Math.round(dist)} m  ${String(Math.round(brg!) % 360).padStart(3, '0')}°`}
        />
        <Row label="Fixes" value={String(fixes.length)} />
      </dl>

      <footer className="mt-3 flex flex-wrap items-center gap-2 text-[11px]">
        <span className="flex items-center gap-1.5"><span className={`status-dot ${mqttUp ? 'bg-ok' : 'bg-bad'}`} />MQTT</span>
        <button onClick={download} disabled={!selected} className="ml-auto rounded bg-edge px-2 py-1 text-ink-2">
          Download CSV
        </button>
        <button
          onClick={pickDir}
          disabled={!canPickDir}
          title={canPickDir ? 'Keep a CSV per session in a folder' : 'Needs Chrome or Edge'}
          className="rounded bg-edge px-2 py-1 text-ink-2"
        >
          {dir ? `Saving → ${dir.name}` : 'Auto-save folder'}
        </button>
        <button onClick={reset} disabled={!selected} className="rounded bg-edge px-2 py-1 text-bad">
          Reset
        </button>
        {saveError && <p role="alert" className="w-full text-bad">{saveError}</p>}
      </footer>
    </section>
  );
}
