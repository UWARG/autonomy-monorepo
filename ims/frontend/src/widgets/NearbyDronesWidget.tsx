import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';
import { enuOffsetM, haversineM } from '../geo';
import type { NearbyDronesMessage, PositionMessage } from '../types';

const DASH = '—';

const GLOBAL_POSITION_TOPIC = 'mavros/global_position/global';

interface NavSatFix {
  latitude: number;
  longitude: number;
  altitude: number;
}

/** Plot half-range steps in metres; the smallest one that fits all traffic is used. */
const RANGE_STEPS_M = [50, 100, 250, 500, 1000, 2500, 5000];

/** Force a tight zoom once anything gets this close, so a near drone doesn't shrink to a dot. */
const NEAR_ZOOM_THRESHOLD_M = 200;
const NEAR_ZOOM_RANGE_M = 250;

const VIEW = 200; // svg viewbox size
const CENTER = VIEW / 2;
const EDGE_MARGIN = 24;
const MAX_LABEL_CHARS = 10;

function pickRangeM(maxDistanceM: number): number {
  return RANGE_STEPS_M.find((r) => r >= maxDistanceM) ?? RANGE_STEPS_M[RANGE_STEPS_M.length - 1];
}

export default function NearbyDronesWidget({
  nearby,
  connected,
  stale,
}: {
  nearby?: NearbyDronesMessage;
  connected: boolean;
  stale: boolean;
}) {
  const [position, setPosition] = useState<PositionMessage>();

  useEffect(() => {
    const fixTopic = new ROSLIB.Topic<NavSatFix>({
      ros,
      name: GLOBAL_POSITION_TOPIC,
      messageType: 'sensor_msgs/NavSatFix',
    });
    const onFix = (msg: NavSatFix) => {
      setPosition({ lat: msg.latitude, lon: msg.longitude, alt: msg.altitude });
    };
    fixTopic.subscribe(onFix);
    return () => fixTopic.unsubscribe(onFix);
  }, []);

  const drones = nearby?.drones ?? [];

  // Without our own position, centre on the traffic so it is still visible.
  const origin = position
    ? { lat: position.lat, lon: position.lon }
    : drones.length
      ? {
          lat: drones.reduce((s, d) => s + d.lat, 0) / drones.length,
          lon: drones.reduce((s, d) => s + d.lon, 0) / drones.length,
        }
      : null;

  const offsets = origin
    ? drones.map((d) => ({ drone: d, ...enuOffsetM(origin.lat, origin.lon, d.lat, d.lon) }))
    : [];
  const distances = offsets.map((o) => Math.hypot(o.east, o.north));
  const maxRange = distances.reduce((m, d) => Math.max(m, d), 0);
  const nearestRange = distances.length ? Math.min(...distances) : Infinity;
  const rangeM =
    nearestRange <= NEAR_ZOOM_THRESHOLD_M ? NEAR_ZOOM_RANGE_M : pickRangeM(maxRange);
  const scale = (CENTER - EDGE_MARGIN) / rangeM; // px per metre

  const nearest =
    position && drones.length
      ? drones
          .map((d) => ({ d, dist: haversineM(position.lat, position.lon, d.lat, d.lon) }))
          .reduce((a, b) => (b.dist < a.dist ? b : a))
      : null;

  const noFeed = !nearby || stale;
  const pill = !connected
    ? { className: 'pill-bad', label: 'RELAY OFFLINE' }
    : !nearby
      ? { className: 'pill bg-edge text-ink-3', label: 'NO FEED' }
      : stale
        ? { className: 'pill-warn', label: 'STALE' }
        : drones.length
          ? { className: 'pill-accent', label: `${drones.length} NEARBY` }
          : { className: 'pill-ok', label: 'NO TRAFFIC' };

  return (
    <section
      className="widget flex h-full min-h-[120px] flex-col p-4"
      style={{
        ['--nd-map' as string]: '#EDF1F7',
        ['--nd-grid' as string]: '#C7D0DE',
        ['--nd-ink3' as string]: '#6B7688',
        ['--nd-self' as string]: '#3B6FD4',
        ['--nd-traffic' as string]: '#D97706',
      }}
    >
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">Nearby Drones</h2>
        <span className={pill.className}>{pill.label}</span>
      </header>

      <div
        className="relative mt-3 flex-1 overflow-hidden rounded-lg"
        style={{ background: 'var(--nd-map)' }}
      >
        <svg viewBox={`0 0 ${VIEW} ${VIEW}`} className="h-full w-full">
          {[0.25, 0.5, 0.75].map((f) => (
            <g key={f} stroke="var(--nd-grid)" strokeWidth="0.5">
              <line x1={VIEW * f} y1="0" x2={VIEW * f} y2={VIEW} />
              <line x1="0" y1={VIEW * f} x2={VIEW} y2={VIEW * f} />
            </g>
          ))}

          <g transform={`translate(${VIEW - 16} 14)`}>
            <path d="M0 -6 l3 6 l-6 0 z" fill="var(--nd-ink3)" />
            <text x="6" y="2" fontSize="8" fill="var(--nd-ink3)">N</text>
          </g>

          <circle
            cx={CENTER}
            cy={CENTER}
            r={rangeM * scale}
            fill="none"
            stroke="var(--nd-grid)"
            strokeWidth="0.75"
          />
          <text x="6" y={VIEW - 6} fontSize="7" fill="var(--nd-ink3)">
            {rangeM} m radius
          </text>

          <g opacity={noFeed ? 0.35 : 1}>
            {offsets.map(({ drone, east, north }) => {
              const range = Math.hypot(east, north);
              const k = range > rangeM ? rangeM / range : 1;
              const x = CENTER + east * k * scale;
              const y = CENTER - north * k * scale;
              const keepAwayPx = drone.horizontal_keep_away * scale;
              const label =
                drone.name.length > MAX_LABEL_CHARS
                  ? `${drone.name.slice(0, MAX_LABEL_CHARS - 1)}…`
                  : drone.name;
              return (
                <g key={drone.id}>
                  {keepAwayPx >= 2 && (
                    <circle
                      cx={x}
                      cy={y}
                      r={keepAwayPx}
                      fill="var(--nd-traffic)"
                      fillOpacity="0.08"
                      stroke="var(--nd-traffic)"
                      strokeWidth="0.75"
                      strokeDasharray="2 2"
                    />
                  )}
                  <path
                    d="M0 -6 L4 5 L0 3 L-4 5 Z"
                    transform={`translate(${x} ${y}) rotate(${drone.direction})`}
                    fill="var(--nd-traffic)"
                  />
                  <text x={x + 7} y={y - 3} fontSize="7" fill="var(--nd-ink3)">
                    {label}
                  </text>
                  <text x={x + 7} y={y + 5} fontSize="6" fill="var(--nd-ink3)">
                    {Math.round(range)} m
                  </text>
                  <text x={x + 7} y={y + 11} fontSize="6" fill="var(--nd-ink3)">
                    {Math.round(drone.alt)} m alt
                  </text>
                </g>
              );
            })}

            {position && (
              <>
                <circle cx={CENTER} cy={CENTER} r="4.5" fill="var(--nd-self)" />
                <circle
                  cx={CENTER}
                  cy={CENTER}
                  r="8"
                  fill="none"
                  stroke="var(--nd-self)"
                  strokeWidth="1"
                  opacity="0.4"
                />
              </>
            )}
          </g>

          {!drones.length && (
            <text x={CENTER} y={CENTER + 14} textAnchor="middle" fontSize="10" fill="var(--nd-ink3)">
              {noFeed ? 'No traffic data' : 'No nearby drones'}
            </text>
          )}
        </svg>
      </div>

      <footer className="mt-3 flex items-center justify-between">
        <div className="flex items-center gap-4 text-[11px] text-ink-3">
          <span className="flex items-center gap-1.5">
            <span className="status-dot" style={{ background: 'var(--nd-self)' }} />
            {position ? 'Own' : 'Own (no position)'}
          </span>
          <span className="flex items-center gap-1.5">
            <span className="status-dot" style={{ background: 'var(--nd-traffic)' }} />
            Traffic
          </span>
        </div>
        <div className="flex flex-col items-end">
          <span className="widget-label">Nearest</span>
          <span className="font-mono text-sm font-semibold tabular-nums text-ink">
            {nearest ? `${Math.round(nearest.dist)} m` : DASH}
          </span>
        </div>
      </footer>
    </section>
  );
}
