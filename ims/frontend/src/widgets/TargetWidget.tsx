import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';
import { TOPICS } from '../topics';

/**
 * Drone position stamped with client receipt time (ms epoch). NavSatFix carries
 * a header stamp, but the trail is about cadence as the ground station saw it,
 * so arrival time is what gets recorded.
 */
interface TrailSample {
  lat: number;
  lon: number;
  t: number;
}

interface Fix {
  latitude: number;
  longitude: number;
  altitude: number;
}

/** airside_interfaces/Target: a colour name plus a Coordinate. */
interface TargetMsg {
  colour: string;
  location: { lat: number; lon: number; alt: number };
}

/** Seconds of history the trail represents. */
const TRAIL_WINDOW_S = 20;
/** Hard cap so a long flight cannot grow the trail without bound. */
const TRAIL_MAX = 600;

const PLOT_HALF_RANGE_M = 60;

/** mavros republishes the fix at ~5 Hz, so this much silence means it stopped. */
const FIX_STALE_AFTER_MS = 5000;
/** Re-evaluated on a timer so a stopped feed is noticed without new messages. */
const TICK_MS = 1000;

const DASH = '\u2014';

const R_EARTH_M = 6_371_000;
const toRad = (d: number) => (d * Math.PI) / 180;
const toDeg = (r: number) => (r * 180) / Math.PI;

function haversineM(aLat: number, aLon: number, bLat: number, bLon: number): number {
  const dLat = toRad(bLat - aLat);
  const dLon = toRad(bLon - aLon);
  const s =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(toRad(aLat)) * Math.cos(toRad(bLat)) * Math.sin(dLon / 2) ** 2;
  return 2 * R_EARTH_M * Math.asin(Math.min(1, Math.sqrt(s)));
}

function bearingDeg(aLat: number, aLon: number, bLat: number, bLon: number): number {
  const dLon = toRad(bLon - aLon);
  const y = Math.sin(dLon) * Math.cos(toRad(bLat));
  const x =
    Math.cos(toRad(aLat)) * Math.sin(toRad(bLat)) -
    Math.sin(toRad(aLat)) * Math.cos(toRad(bLat)) * Math.cos(dLon);
  return (toDeg(Math.atan2(y, x)) + 360) % 360;
}

function enuOffsetM(
  originLat: number,
  originLon: number,
  lat: number,
  lon: number,
): { east: number; north: number } {
  const east = toRad(lon - originLon) * Math.cos(toRad(originLat)) * R_EARTH_M;
  const north = toRad(lat - originLat) * R_EARTH_M;
  return { east, north };
}

const VIEW = 200; // svg viewbox size
const CENTER = VIEW / 2;

export default function TargetWidget() {
  const [fix, setFix] = useState<Fix | null>(null);
  const [fixAt, setFixAt] = useState(0);
  const [now, setNow] = useState(() => Date.now());
  const [target, setTarget] = useState<TargetMsg | null>(null);
  const [trail, setTrail] = useState<TrailSample[]>([]);

  useEffect(() => {
    const fixTopic = new ROSLIB.Topic<Fix>({
      ros,
      name: TOPICS.globalPosition.name,
      messageType: TOPICS.globalPosition.type,
    });
    const targetTopic = new ROSLIB.Topic<TargetMsg>({
      ros,
      name: TOPICS.target.name,
      messageType: TOPICS.target.type,
    });

    fixTopic.subscribe((message) => {
      const at = Date.now();
      setFix(message);
      setFixAt(at);
      setTrail((prev) =>
        [...prev, { lat: message.latitude, lon: message.longitude, t: at }].slice(-TRAIL_MAX),
      );
    });
    targetTopic.subscribe(setTarget);

    const ticker = window.setInterval(() => setNow(Date.now()), TICK_MS);
    return () => {
      window.clearInterval(ticker);
      fixTopic.unsubscribe();
      targetTopic.unsubscribe();
    };
  }, []);

  // A frozen panel is worse than an empty one: once the feed stops, the last
  // fix (and the trail it anchors) is dropped rather than drawn as current.
  const stale = fix != null && now - fixAt > FIX_STALE_AFTER_MS;
  const live = stale ? null : fix;

  const haveDrone = live != null;
  const haveTarget = !!(live && target);

  const distance = haveTarget
    ? haversineM(live.latitude, live.longitude, target.location.lat, target.location.lon)
    : null;
  const bearing = haveTarget
    ? bearingDeg(live.latitude, live.longitude, target.location.lat, target.location.lon)
    : null;

  // Wall-clock window, so a stopped feed lets the trail expire instead of
  // pinning it to the last sample received.
  const cutoff = now - TRAIL_WINDOW_S * 1000;
  const recentTrail = trail.filter((s) => s.t >= cutoff);

  const scale = (CENTER - 24) / PLOT_HALF_RANGE_M; // px per metre

  const project = (east: number, north: number) => ({
    x: CENTER + east * scale,
    y: CENTER - north * scale, // north is up
  });

  let targetPt: { x: number; y: number } | null = null;
  let targetClamped = false;
  if (live && target) {
    const o = enuOffsetM(live.latitude, live.longitude, target.location.lat, target.location.lon);
    const range = Math.hypot(o.east, o.north);
    if (range > PLOT_HALF_RANGE_M) {
      const k = PLOT_HALF_RANGE_M / range;
      targetPt = project(o.east * k, o.north * k);
      targetClamped = true;
    } else {
      targetPt = project(o.east, o.north);
    }
  }

  const trailPts = live
    ? recentTrail.map((p) => {
        const o = enuOffsetM(live.latitude, live.longitude, p.lat, p.lon);
        return project(o.east, o.north);
      })
    : [];

  const targetPill = stale
    ? { className: 'pill-warn', label: 'STALE' }
    : target
      ? { className: 'pill-accent', label: `${target.colour || 'TARGET'} · DETECTED` }
      : { className: 'pill bg-edge text-ink-3', label: 'NO TARGET' };

  return (
    <section
      className="widget flex h-full min-h-[120px] flex-col p-4"
      style={{
        ['--tgt-map' as string]: '#EDF1F7',
        ['--tgt-grid' as string]: '#C7D0DE',
        ['--tgt-ink3' as string]: '#6B7688',
        ['--tgt-drone' as string]: '#3B6FD4',
        ['--tgt-target' as string]: '#1FA463',
      }}
    >
      <header className="flex items-center justify-between gap-4">
        <div className="flex items-baseline gap-2">
          <h2 className="widget-label">Position / Target</h2>
          <span className="font-mono text-[11px] text-ink-3">
            {live ? `${live.latitude.toFixed(5)}, ${live.longitude.toFixed(5)}` : DASH}
          </span>
        </div>
        <span className={targetPill.className}>{targetPill.label}</span>
      </header>

      <div
        className="relative mt-3 flex-1 overflow-hidden rounded-lg"
        style={{ background: 'var(--tgt-map)' }}
      >
        <svg viewBox={`0 0 ${VIEW} ${VIEW}`} className="h-full w-full">
          {/* grid */}
          {[0.25, 0.5, 0.75].map((f) => (
            <g key={f} stroke="var(--tgt-grid)" strokeWidth="0.5">
              <line x1={VIEW * f} y1="0" x2={VIEW * f} y2={VIEW} />
              <line x1="0" y1={VIEW * f} x2={VIEW} y2={VIEW * f} />
            </g>
          ))}

          {/* north indicator */}
          <g transform={`translate(${VIEW - 16} 14)`}>
            <path d="M0 -6 l3 6 l-6 0 z" fill="var(--tgt-ink3)" />
            <text x="6" y="2" fontSize="8" fill="var(--tgt-ink3)">N</text>
          </g>

          {/* fixed-range ring + scale label, so on-screen distance is real */}
          <circle
            cx={CENTER}
            cy={CENTER}
            r={PLOT_HALF_RANGE_M * scale}
            fill="none"
            stroke="var(--tgt-grid)"
            strokeWidth="0.75"
          />
          <text x="6" y={VIEW - 6} fontSize="7" fill="var(--tgt-ink3)">
            {PLOT_HALF_RANGE_M} m radius
          </text>

          {haveDrone ? (
            <>
              {/* trail */}
              {trailPts.length > 1 && (
                <polyline
                  points={trailPts.map((p) => `${p.x},${p.y}`).join(' ')}
                  fill="none"
                  stroke="var(--tgt-drone)"
                  strokeWidth="1.5"
                  strokeDasharray="3 3"
                  opacity="0.7"
                />
              )}
              {/* line to target — dashed when the target is clamped to the edge */}
              {targetPt && (
                <line
                  x1={CENTER}
                  y1={CENTER}
                  x2={targetPt.x}
                  y2={targetPt.y}
                  stroke="var(--tgt-target)"
                  strokeWidth="1.5"
                  strokeDasharray={targetClamped ? '2 2' : undefined}
                />
              )}
              {/* target dot — hollow when clamped (true position is beyond range) */}
              {targetPt &&
                (targetClamped ? (
                  <circle
                    cx={targetPt.x}
                    cy={targetPt.y}
                    r="4"
                    fill="none"
                    stroke="var(--tgt-target)"
                    strokeWidth="1.5"
                  />
                ) : (
                  <circle cx={targetPt.x} cy={targetPt.y} r="4" fill="var(--tgt-target)" />
                ))}
              {/* drone dot (centre) */}
              <circle cx={CENTER} cy={CENTER} r="4.5" fill="var(--tgt-drone)" />
              <circle cx={CENTER} cy={CENTER} r="8" fill="none" stroke="var(--tgt-drone)" strokeWidth="1" opacity="0.4" />
            </>
          ) : (
            <text x={CENTER} y={CENTER} textAnchor="middle" fontSize="10" fill="var(--tgt-ink3)">
              Awaiting {TOPICS.globalPosition.name}
            </text>
          )}
        </svg>
      </div>

      <footer className="mt-3 flex items-center justify-between">
        <div className="flex items-center gap-4 text-[11px] text-ink-3">
          <span className="flex items-center gap-1.5"><span className="status-dot" style={{ background: 'var(--tgt-drone)' }} />Drone</span>
          <span className="flex items-center gap-1.5"><span className="status-dot" style={{ background: 'var(--tgt-target)' }} />Target</span>
        </div>
        <div className="flex items-baseline gap-4">
          <div className="flex flex-col items-end">
            <span className="widget-label">Altitude AMSL</span>
            <span className="font-mono text-sm font-semibold tabular-nums text-ink">
              {live ? `${live.altitude.toFixed(1)} m` : DASH}
            </span>
          </div>
          <div className="flex flex-col items-end">
            <span className="widget-label">Distance</span>
            <span className="font-mono text-sm font-semibold tabular-nums text-ink">
              {distance != null ? `${Math.round(distance)} m` : DASH}
            </span>
          </div>
          <div className="flex flex-col items-end">
            <span className="widget-label">Bearing</span>
            <span className="font-mono text-sm font-semibold tabular-nums text-ink">
              {bearing != null ? `${String(Math.round(bearing)).padStart(3, '0')}\u00B0` : DASH}
            </span>
          </div>
        </div>
      </footer>
    </section>
  );
}
