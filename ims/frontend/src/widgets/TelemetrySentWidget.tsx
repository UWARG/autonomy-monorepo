import { useEffect, useState } from 'react';
import type { AeacAckMessage, AeacInfractionMessage, TelemetrySentMessage } from '../types';

const DASH = '—';

/** AEAC penalizes gaps between telemetry packets longer than this. */
const SLOW_TELEM_S = 1.1;
/** Packets are going out but AEAC hasn't acknowledged one for this long. */
const NO_ACK_S = 3;
/** Highlight an infraction for this long after it happens. */
const RECENT_INFRACTION_S = 15;
const CLOCK_TICK_MS = 250;

function Row({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-baseline justify-between gap-4">
      <dt className="text-[13px] text-ink-3">{label}</dt>
      <dd className="font-mono text-[13px] tabular-nums text-ink">{value}</dd>
    </div>
  );
}

function Flag({ ok, okLabel, badLabel }: { ok: boolean; okLabel: string; badLabel: string }) {
  return <span className={ok ? 'pill-ok' : 'pill-bad'}>{ok ? okLabel : badLabel}</span>;
}

/** Latest packet the IMS relay sent to AEAC, when it was sent, and AEAC's verdict on it. */
export default function TelemetrySentWidget({
  telemetrySent,
  receivedAtMs,
  ack,
  ackAtMs,
  infraction,
  connected,
}: {
  telemetrySent?: TelemetrySentMessage;
  receivedAtMs?: number;
  ack?: AeacAckMessage;
  ackAtMs?: number;
  infraction?: AeacInfractionMessage;
  connected: boolean;
}) {
  const [nowMs, setNowMs] = useState(() => Date.now());

  useEffect(() => {
    const id = window.setInterval(() => setNowMs(Date.now()), CLOCK_TICK_MS);
    return () => window.clearInterval(id);
  }, []);

  const p = telemetrySent?.packet;
  // Measured from arrival here; the relay is local, so this tracks the send time.
  const ageS = receivedAtMs !== undefined ? Math.max(0, (nowMs - receivedAtMs) / 1000) : null;
  const late = ageS !== null && ageS > SLOW_TELEM_S;
  const linkLost = p?.mode === 'link-lost';
  const ackAgeS = ackAtMs !== undefined ? Math.max(0, (nowMs - ackAtMs) / 1000) : null;
  // Only an issue while we're actually sending.
  const noAck = !late && p !== undefined && (ackAgeS === null || ackAgeS > NO_ACK_S);
  const infractionAgeS = infraction ? Math.max(0, nowMs / 1000 - infraction.received_at) : null;
  const recentInfraction = infractionAgeS !== null && infractionAgeS <= RECENT_INFRACTION_S;
  const infractionTotal = infraction
    ? Object.values(infraction.counts).reduce((sum, n) => sum + n, 0)
    : 0;

  const pill = !connected
    ? { className: 'pill-bad', label: 'RELAY OFFLINE' }
    : !p
      ? { className: 'pill bg-edge text-ink-3', label: 'NO DATA' }
      : late
        ? { className: 'pill-bad', label: 'LATE' }
        : noAck
          ? { className: 'pill-bad', label: 'NO ACK' }
          : linkLost
            ? { className: 'pill-bad', label: 'LINK LOST' }
            : { className: 'pill-ok', label: 'SENDING' };

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">AEAC Telemetry{p ? ` · ${p.uavId}` : ''}</h2>
        <span className={pill.className}>{pill.label}</span>
      </header>

      <p className={`mt-1 font-mono text-[13px] tabular-nums ${late ? 'text-bad' : 'text-ink-2'}`}>
        {p && ageS !== null
          ? `Sent ${new Date(p.unixTime * 1000).toLocaleTimeString()} · ${ageS.toFixed(1)}s ago`
          : 'Awaiting first packet'}
      </p>

      <p className={`mt-1 font-mono text-[13px] tabular-nums ${noAck ? 'text-bad' : ack ? 'text-ok' : 'text-ink-3'}`}>
        {ack && ackAgeS !== null
          ? `${noAck ? '✗' : '✓'} AEAC ack ${ackAgeS.toFixed(1)}s ago`
          : 'No AEAC ack yet'}
      </p>

      <div className="mt-1.5 flex flex-wrap items-center gap-1.5">
        {/* telemetryLinkStatus is 0 exactly when mode is link-lost, so mode covers both. */}
        {p && (
          <span className={linkLost ? 'pill-bad' : 'pill bg-edge text-ink-2'}>
            {p.mode.toUpperCase()}
          </span>
        )}
        {ack && (
          <>
            <Flag ok={ack.inside_boundary} okLabel="IN BOUNDS" badLabel="OUT OF BOUNDS" />
            <Flag ok={!ack.too_close_to_traffic} okLabel="CLEAR" badLabel="TOO CLOSE" />
          </>
        )}
      </div>

      <p
        className={`mt-1 font-mono text-[13px] tabular-nums ${recentInfraction ? 'text-bad' : 'text-ink-3'}`}
        title={infraction ? JSON.stringify(infraction.counts) : undefined}
      >
        {infraction && infractionAgeS !== null
          ? `Infractions ${infractionTotal} · last ${infraction.last_infraction} ${Math.round(infractionAgeS)}s ago`
          : 'No infractions reported'}
      </p>

      <dl className="mt-2 grid grid-cols-2 gap-x-4 gap-y-1">
        <Row label="Lat" value={p ? p.latitude.toFixed(5) : DASH} />
        <Row label="Lon" value={p ? p.longitude.toFixed(5) : DASH} />
        <Row label="Alt" value={p ? `${p.altitudeAGL.toFixed(1)} m` : DASH} />
        <Row
          label="Batt"
          value={p?.batteryPercentage != null ? `${Math.round(p.batteryPercentage)}%` : DASH}
        />
      </dl>
    </section>
  );
}
