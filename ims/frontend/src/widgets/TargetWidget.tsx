import type { PositionMessage, TargetMessage } from '../types';
import { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';

export interface TrailSample {
  lat: number;
  lon: number;
  t: number;
}

/** Seconds of history the trail represents. */
const TRAIL_WINDOW_S = 20;

const TARGET_STALE_MS = 3000;

const PLOT_HALF_RANGE_M = 60;

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

interface NavSatFix {
  latitude: number;
  longitude: number;
  altitude: number;
}

interface RawTarget {
  colour: string;
  location: { lat: number; lon: number; alt: number };
}

/**
 * Dot colour per detected colour. Target.msg carries no target ID, so
 * targets are keyed and coloured by this field — two simultaneous targets
 * of the same colour aren't distinguishable and will collide (the newer
 * one wins).
 */
const TARGET_DOT_COLOURS: Record<string, string> = {
  RED: '#D9483F',
  RED2: '#D9483F',
  GREEN: '#1FA463',
  BLUE: '#4FA3E0',
  YELLOW: '#D4A017',
  WHITE: '#E8ECF3',
  BLACK: '#1C2230',
};
const DEFAULT_TARGET_COLOUR = '#1FA463';

export default function TargetWidget() {
  const [position, setPosition] = useState<PositionMessage>();
  const [targets, setTargets] = useState<Map<string, TargetMessage>>(new Map());
  const [trail, setTrail] = useState<TrailSample[]>([]);
  const lastSeenRef = useRef<Map<string, number>>(new Map());

  useEffect(() => {
    const positionTopic = new ROSLIB.Topic<NavSatFix>({
      ros,
      name: 'mavros/global_position/global',
      messageType: 'sensor_msgs/NavSatFix',
    });

    const onPosition = (message: NavSatFix) => {
      setPosition({ lat: message.latitude, lon: message.longitude, alt: message.altitude });

      const t = Date.now();
      setTrail((prev) => [
        ...prev.filter((s) => s.t >= t - TRAIL_WINDOW_S * 1000),
        { lat: message.latitude, lon: message.longitude, t },
      ]);
    };

    positionTopic.subscribe(onPosition);
    return () => positionTopic.unsubscribe(onPosition);
  }, []);

  useEffect(() => {
    const targetTopic = new ROSLIB.Topic<RawTarget>({
      ros,
      name: '/capture/target_location',
      messageType: 'airside_interfaces/Target',
    });

    const onTarget = (message: RawTarget) => {
      lastSeenRef.current.set(message.colour, Date.now());
      setTargets((prev) => {
        const next = new Map(prev);
        next.set(message.colour, {
          lat: message.location.lat,
          lon: message.location.lon,
          label: message.colour,
          tracking: true,
        });
        return next;
      });
    };

    targetTopic.subscribe(onTarget);

    const staleCheck = setInterval(() => {
      const now = Date.now();
      setTargets((prev) => {
        let changed = false;
        const next = new Map(prev);
        for (const [colour, lastSeen] of lastSeenRef.current) {
          const t = next.get(colour);
          if (t?.tracking && now - lastSeen > TARGET_STALE_MS) {
            next.set(colour, { ...t, tracking: false });
            changed = true;
          }
        }
        return changed ? next : prev;
      });
    }, 500);

    return () => {
      targetTopic.unsubscribe(onTarget);
      clearInterval(staleCheck);
    };
  }, []);

  const haveDrone = !!position;
  const targetList = Array.from(targets.values());

  // The header pill and distance/bearing footer summarize the whole set:
  // the nearest target drives the numeric readout, while every target still
  // gets its own dot on the map.
  let nearest: TargetMessage | null = null;
  let nearestDist = Infinity;
  if (position) {
    for (const t of targetList) {
      const d = haversineM(position.lat, position.lon, t.lat, t.lon);
      if (d < nearestDist) {
        nearestDist = d;
        nearest = t;
      }
    }
  }
  const haveTarget = !!(position && nearest);

  const distance = haveTarget ? nearestDist : null;
  const bearing = haveTarget
    ? bearingDeg(position!.lat, position!.lon, nearest!.lat, nearest!.lon)
    : null;

  const newestT = trail.length ? trail[trail.length - 1].t : 0;
  const cutoff = newestT - TRAIL_WINDOW_S * 1000;
  const recentTrail = trail.filter((s) => s.t >= cutoff);

  const scale = (CENTER - 24) / PLOT_HALF_RANGE_M; // px per metre

  const project = (east: number, north: number) => ({
    x: CENTER + east * scale,
    y: CENTER - north * scale, // north is up
  });

  const targetDots = position
    ? targetList.map((t) => {
        const o = enuOffsetM(position.lat, position.lon, t.lat, t.lon);
        const range = Math.hypot(o.east, o.north);
        const clamped = range > PLOT_HALF_RANGE_M;
        const pt = clamped ? project(o.east * (PLOT_HALF_RANGE_M / range), o.north * (PLOT_HALF_RANGE_M / range)) : project(o.east, o.north);
        return { colour: t.label ?? 'TARGET', pt, clamped, tracking: t.tracking };
      })
    : [];

  const trailPts = position
    ? recentTrail.map((p) => {
        const o = enuOffsetM(position.lat, position.lon, p.lat, p.lon);
        return project(o.east, o.north);
      })
    : [];

  const trackingCount = targetList.filter((t) => t.tracking).length;
  const targetCountLabel = `${targetList.length} TARGET${targetList.length > 1 ? 'S' : ''}`;
  const trackingPill =
    targetList.length === 0
      ? { className: 'pill bg-edge text-ink-3', label: 'NO TARGET' }
      : trackingCount > 0
        ? { className: 'pill-accent', label: `${targetCountLabel} \u00B7 TRACKING` }
        : { className: 'pill bg-edge text-ink-3', label: `${targetCountLabel} \u00B7 IDLE` };

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
        <h2 className="widget-label">Position / Target</h2>
        <span className={trackingPill.className}>{trackingPill.label}</span>
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
              {/* target dots — hollow when clamped (true position is beyond range) */}
              {targetDots.map((d) => {
                const colour = TARGET_DOT_COLOURS[d.colour] ?? DEFAULT_TARGET_COLOUR;
                return d.clamped ? (
                  <circle
                    key={d.colour}
                    cx={d.pt.x}
                    cy={d.pt.y}
                    r="4"
                    fill="none"
                    stroke={colour}
                    strokeWidth="1.5"
                  />
                ) : (
                  <circle
                    key={d.colour}
                    cx={d.pt.x}
                    cy={d.pt.y}
                    r="4"
                    fill={colour}
                    stroke="var(--tgt-ink3)"
                    strokeWidth="0.5"
                  />
                );
              })}
              {/* drone dot (centre) */}
              <circle cx={CENTER} cy={CENTER} r="4.5" fill="var(--tgt-drone)" />
              <circle cx={CENTER} cy={CENTER} r="8" fill="none" stroke="var(--tgt-drone)" strokeWidth="1" opacity="0.4" />
            </>
          ) : (
            <text x={CENTER} y={CENTER} textAnchor="middle" fontSize="10" fill="var(--tgt-ink3)">
              No position data
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