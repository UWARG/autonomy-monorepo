import { useEffect, useRef, useState, type ReactNode } from 'react';
import L from 'leaflet';
import {
  Circle,
  MapContainer,
  Marker,
  Polyline,
  ScaleControl,
  TileLayer,
  Tooltip,
  useMap,
  useMapEvents,
} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import useNavigation from '../hooks/useNavigation';
import useOwnTrack from '../hooks/useOwnTrack';
import { bearingDeg, haversineM } from '../geo';
import type { AeacTrafficState } from '../hooks/useAeacTraffic';
import type { NearbyDrone, PositionMessage } from '../types';

const DASH = '—';

const MAX_NATIVE_ZOOM = 19; // deepest Esri World Imagery level that is reliably populated
const MAX_ZOOM = 22;
const FIT_MAX_ZOOM = 20;

const MAX_LABEL_CHARS = 12;

/** A target within this of a lap waypoint is that waypoint. */
const SAME_POINT_M = 1;

const COLORS = {
  self: '#5B8DEF',
  setpoint: '#E879F9',
  course: '#34D17E',
  traffic: '#F59E0B',
  /** Traffic we are vertically clear of: the avoider ignores it. */
  trafficClear: '#9BA5B7',
};

type LatLon = { lat: number; lon: number };
const ll = (p: LatLon): [number, number] => [p.lat, p.lon];

function droneIcon(heading?: number): L.DivIcon {
  const body =
    heading == null
      ? `<circle cx="14" cy="14" r="6" fill="${COLORS.self}" stroke="white" stroke-width="2"/>`
      : `<path d="M14 3 L21 23 L14 19 L7 23 Z" fill="${COLORS.self}" stroke="white" stroke-width="1.5"
           stroke-linejoin="round" transform="rotate(${heading} 14 14)"/>`;
  return L.divIcon({
    className: 'ims-map-icon',
    iconSize: [28, 28],
    iconAnchor: [14, 14],
    html: `<svg width="28" height="28" viewBox="0 0 28 28">${body}</svg>`,
  });
}

function waypointIcon(n: number, current: boolean): L.DivIcon {
  const size = current ? 24 : 20;
  const fill = current ? COLORS.course : 'rgba(12,15,22,0.75)';
  const ink = current ? '#0C0F16' : COLORS.course;
  return L.divIcon({
    className: 'ims-map-icon',
    iconSize: [size, size],
    iconAnchor: [size / 2, size / 2],
    html: `<div class="ims-map-wp" style="width:${size}px;height:${size}px;background:${fill};color:${ink};
             border-color:${COLORS.course}">${n}</div>`,
  });
}

/** Crosshair: the exact point the FC is being steered to. */
const SETPOINT_ICON = L.divIcon({
  className: 'ims-map-icon',
  iconSize: [26, 26],
  iconAnchor: [13, 13],
  html: `<svg width="26" height="26" viewBox="0 0 26 26" fill="none" stroke-linecap="round">
    <g stroke="white" stroke-width="4" opacity="0.8">
      <circle cx="13" cy="13" r="6"/><path d="M13 1v6M13 19v6M1 13h6M19 13h6"/>
    </g>
    <g stroke="${COLORS.setpoint}" stroke-width="2">
      <circle cx="13" cy="13" r="6"/><path d="M13 1v6M13 19v6M1 13h6M19 13h6"/>
    </g>
  </svg>`,
});

/** An engine target that is not a lap waypoint (launch point, landing pad). */
const OFF_COURSE_TARGET_ICON = L.divIcon({
  className: 'ims-map-icon',
  iconSize: [20, 20],
  iconAnchor: [10, 10],
  html: `<svg width="20" height="20" viewBox="0 0 20 20">
    <path d="M10 2 L18 10 L10 18 L2 10 Z" fill="${COLORS.course}" stroke="white" stroke-width="1.5"/>
  </svg>`,
});

function trafficIcon(direction: number, clear: boolean): L.DivIcon {
  return L.divIcon({
    className: 'ims-map-icon',
    iconSize: [22, 22],
    iconAnchor: [11, 11],
    html: `<svg width="22" height="22" viewBox="0 0 22 22">
      <path d="M11 3 L16 18 L11 15 L6 18 Z" fill="${clear ? COLORS.trafficClear : COLORS.traffic}" stroke="white" stroke-width="1.2"
        stroke-linejoin="round" transform="rotate(${direction} 11 11)"/>
    </svg>`,
  });
}

/**
 * Keeps Leaflet sized to its grid cell, follows the drone while `follow` is on,
 * fits everything once when data first arrives and again on each `fitKey` bump,
 * and reports the user dragging the map so the parent can drop out of follow.
 */
function ViewController({
  follow,
  center,
  fitPoints,
  fitKey,
  onUserPan,
}: {
  follow: boolean;
  center?: LatLon;
  fitPoints: LatLon[];
  fitKey: number;
  onUserPan: () => void;
}) {
  const map = useMap();
  const fittedOnce = useRef(false);

  useMapEvents({ dragstart: onUserPan });

  useEffect(() => {
    const observer = new ResizeObserver(() => map.invalidateSize({ animate: false }));
    observer.observe(map.getContainer());
    return () => observer.disconnect();
  }, [map]);

  const fit = () => {
    if (fitPoints.length === 0) return;
    if (fitPoints.length === 1) {
      map.setView(ll(fitPoints[0]), Math.max(map.getZoom(), 18), { animate: false });
      return;
    }
    map.fitBounds(L.latLngBounds(fitPoints.map(ll)), {
      padding: [28, 28],
      maxZoom: FIT_MAX_ZOOM,
      animate: false,
    });
  };

  // Frame the scene the first time there is more than one thing on it.
  useEffect(() => {
    if (fittedOnce.current || fitPoints.length < 2) return;
    fittedOnce.current = true;
    fit();
  });

  useEffect(() => {
    if (fitKey > 0) fit();
    // Only an explicit request refits; data updates must not yank the view.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [fitKey]);

  useEffect(() => {
    if (follow && center) map.panTo(ll(center), { animate: false });
  }, [map, follow, center?.lat, center?.lon]); // eslint-disable-line react-hooks/exhaustive-deps

  return null;
}

/** Inset from the map edge for off-screen traffic markers, px. */
const EDGE_INSET = 22;

/**
 * Pins traffic that is outside the current view to the map edge, pointing
 * toward it, so panning or zooming in never hides an aircraft.
 */
function OffscreenTraffic({
  drones,
  stale,
  distTo,
}: {
  drones: NearbyDrone[];
  stale: boolean;
  distTo: (p: PositionMessage) => number | null;
}) {
  const map = useMap();
  const [, setView] = useState(0);
  const redraw = () => setView((v) => v + 1);
  useMapEvents({ move: redraw, zoom: redraw, resize: redraw });

  const size = map.getSize();
  const cx = size.x / 2;
  const cy = size.y / 2;
  const halfW = cx - EDGE_INSET;
  const halfH = cy - EDGE_INSET;

  return (
    <div className="pointer-events-none absolute inset-0 z-[450]">
      {drones.map((d) => {
        const p = map.latLngToContainerPoint([d.lat, d.lon]);
        const dx = p.x - cx;
        const dy = p.y - cy;
        if (Math.abs(dx) <= cx && Math.abs(dy) <= cy) return null;
        // Scale the centre-to-aircraft vector back onto the inset rectangle.
        const k = Math.min(halfW / Math.abs(dx || 1e-9), halfH / Math.abs(dy || 1e-9));
        const x = cx + dx * k;
        const y = cy + dy * k;
        const angle = (Math.atan2(dy, dx) * 180) / Math.PI;
        const dist = distTo(d);
        // Hang the label inward off the left/right edges so it is not clipped.
        const side = x >= cx + halfW - 1 ? 'items-end' : x <= EDGE_INSET + 1 ? 'items-start' : 'items-center';
        const tx = side === 'items-end' ? 'calc(-100% + 8px)' : side === 'items-start' ? '-8px' : '-50%';
        return (
          <div
            key={d.id}
            className={`absolute flex flex-col ${side}`}
            style={{ left: x, top: y, transform: `translate(${tx}, -8px)`, opacity: stale ? 0.45 : 1 }}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" style={{ transform: `rotate(${angle}deg)` }}>
              <path d="M15 8 L3 2 L6 8 L3 14 Z" fill={COLORS.traffic} stroke="white" strokeWidth="1" />
            </svg>
            <span className="ims-map-edge-label">
              {trafficName(d.name, d.id)}
              {dist != null && ` ${Math.round(dist)} m`}
            </span>
          </div>
        );
      })}
    </div>
  );
}

function MapButton({
  active,
  onClick,
  title,
  children,
}: {
  active?: boolean;
  onClick: () => void;
  title: string;
  children: ReactNode;
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      title={title}
      className={`rounded-md border px-2 py-1 font-mono text-[11px] font-semibold backdrop-blur ${
        active
          ? 'border-accent bg-accent-dim/90 text-accent'
          : 'border-edge bg-app/80 text-ink-2 hover:text-ink'
      }`}
    >
      {children}
    </button>
  );
}

const trafficName = (name: string, id: number) =>
  !name ? `#${id}` : name.length > MAX_LABEL_CHARS ? `${name.slice(0, MAX_LABEL_CHARS - 1)}\u2026` : name;

const fmtCoord = (p?: LatLon) => (p ? `${p.lat.toFixed(6)}, ${p.lon.toFixed(6)}` : DASH);
const fmtAlt = (alt?: number) => (alt != null ? `${alt.toFixed(1)} m` : DASH);
const fmtDist = (m: number | null) => (m == null ? DASH : m < 100 ? `${m.toFixed(1)} m` : `${Math.round(m)} m`);
const fmtBearing = (deg?: number | null) =>
  deg == null ? DASH : `${String(Math.round(deg) % 360).padStart(3, '0')}°`;

function ReadoutRow({
  color,
  label,
  coord,
  alt,
  extra,
  dim,
  stale,
}: {
  color: string;
  label: string;
  coord?: LatLon;
  alt?: number;
  extra: string;
  dim?: boolean;
  /** Last value is shown dimmed and the readout replaced by STALE. */
  stale?: boolean;
}) {
  return (
    <div
      className={`grid grid-cols-[104px_minmax(0,184px)_60px_minmax(0,1fr)] items-baseline gap-2 font-mono text-[12px] tabular-nums ${
        dim || stale ? 'text-ink-3' : 'text-ink'
      }`}
    >
      <span className="flex items-center gap-1.5 truncate font-sans text-[11px] font-semibold text-ink-2">
        <span className="status-dot shrink-0" style={{ background: color }} />
        {label}
      </span>
      <span className="truncate" title={coord ? fmtCoord(coord) : undefined}>
        {fmtCoord(coord)}
      </span>
      <span className="text-right">{fmtAlt(alt)}</span>
      <span className={`truncate pl-4 ${stale ? 'text-warn' : 'text-ink-2'}`}>
        {stale ? 'STALE' : extra}
      </span>
    </div>
  );
}

/** AEAC feed health, most severe first. */
function trafficPill(
  traffic: AeacTrafficState,
  count: number,
  atAltitude: number,
): { className: string; label: string } {
  if (traffic.trafficProblem) return { className: 'pill-bad', label: traffic.trafficProblem };
  if (!traffic.connected) return { className: 'pill-bad', label: 'AEAC OFFLINE' };
  if (!traffic.nearbyDrones) return { className: 'pill bg-edge text-ink-3', label: 'NO FEED' };
  if (traffic.nearbyDronesStale) return { className: 'pill-warn', label: 'TRAFFIC STALE' };
  if (count && atAltitude) return { className: 'pill-warn', label: `${atAltitude}/${count} AT OUR ALT` };
  if (count) return { className: 'pill-accent', label: `${count} NEARBY \u00B7 ALT CLEAR` };
  return { className: 'pill-ok', label: 'NO TRAFFIC' };
}

/**
 * Whether an aircraft's vertical keep-away overlaps the altitudes we are flying
 * between. Mirrors gnc obstacle_avoidance traffic_to_obstacles: aircraft outside
 * the band are not obstacles (and not an AEAC breach, which needs both).
 */
function atOurAltitude(d: NearbyDrone, band: [number, number] | null): boolean {
  if (!band) return true; // altitude unknown: assume the worst
  return d.alt + d.vertical_keep_away >= band[0] && d.alt - d.vertical_keep_away <= band[1];
}

/** "+40 m" when the aircraft is above us. */
const fmtVertical = (m: number) => `${m >= 0 ? '+' : '\u2212'}${Math.abs(Math.round(m))} m`;

export default function MapWidget({ traffic }: { traffic: AeacTrafficState }) {
  const { position, heading, relAlt, trail } = useOwnTrack();
  const { setpoint, setpointStale, target, targetStale, waypoints } = useNavigation();
  const [follow, setFollow] = useState(true);
  const [fitKey, setFitKey] = useState(0);
  const [showTraffic, setShowTraffic] = useState(true);

  const currentIdx = target
    ? waypoints.findIndex((w) => haversineM(w.lat, w.lon, target.lat, target.lon) < SAME_POINT_M)
    : -1;
  const allDrones = traffic.nearbyDrones?.drones ?? [];
  const drones = showTraffic ? allDrones : [];
  const trafficStale = traffic.nearbyDronesStale;
  const altBand: [number, number] | null =
    relAlt == null ? null : [Math.min(relAlt, target?.alt ?? relAlt), Math.max(relAlt, target?.alt ?? relAlt)];
  const conflictCount = allDrones.filter((d) => atOurAltitude(d, altBand)).length;
  const verticalOffset = (d: NearbyDrone) => (relAlt == null ? null : d.alt - relAlt);

  const distTo = (p?: PositionMessage) =>
    position && p ? haversineM(position.lat, position.lon, p.lat, p.lon) : null;
  const bearingTo = (p?: PositionMessage) =>
    position && p ? bearingDeg(position.lat, position.lon, p.lat, p.lon) : null;

  const fitPoints: LatLon[] = [
    ...waypoints,
    ...(position ? [position] : []),
    ...(setpoint ? [setpoint] : []),
    ...(target ? [target] : []),
  ];
  const nearest =
    position && allDrones.length
      ? allDrones
          .map((d) => ({ d, dist: haversineM(position.lat, position.lon, d.lat, d.lon) }))
          .reduce((a, b) => (b.dist < a.dist ? b : a))
      : null;

  // Without our own position or a course, centre on the traffic so it is still visible.
  const anchor = position ?? setpoint ?? target ?? waypoints[0] ?? allDrones[0];

  const feedPill = trafficPill(traffic, allDrones.length, conflictCount);
  const navPill = !setpointStale
    ? { className: 'pill-accent', label: 'SETPOINT LIVE' }
    : !targetStale
      ? { className: 'pill-warn', label: 'NO SETPOINT' }
      : { className: 'pill bg-edge text-ink-3', label: 'IDLE' };

  const targetLabel =
    currentIdx >= 0 ? `WP ${currentIdx + 1}/${waypoints.length}` : target ? 'Target' : 'Waypoint';

  return (
    <section className="widget flex h-full min-h-[120px] flex-col p-4">
      <header className="flex items-center justify-between gap-4">
        <div className="flex items-baseline gap-2">
          <h2 className="widget-label">Map</h2>
          <span className="font-mono text-[11px] text-ink-3">
            {waypoints.length ? `${waypoints.length} WP` : 'no course'}
          </span>
        </div>
        <div className="flex items-center gap-2">
          <span className={feedPill.className}>{feedPill.label}</span>
          <span className={navPill.className}>{navPill.label}</span>
        </div>
      </header>

      <div className="ims-map relative isolate mt-3 min-h-0 flex-1 overflow-hidden rounded-lg bg-inset">
        {anchor ? (
          <>
            <MapContainer
              center={ll(anchor)}
              zoom={18}
              zoomSnap={0.25}
              zoomDelta={0.5}
              wheelPxPerZoomLevel={120}
              maxZoom={MAX_ZOOM}
              zoomControl={false}
              className="h-full w-full"
              style={{ background: '#0E131C' }}
            >
              <TileLayer
                url="/tiles/{z}/{x}/{y}"
                maxNativeZoom={MAX_NATIVE_ZOOM}
                maxZoom={MAX_ZOOM}
                attribution="Tiles &copy; Esri"
              />
              <ScaleControl position="bottomleft" imperial={false} />
              <ViewController
                follow={follow}
                center={position}
                fitPoints={fitPoints}
                fitKey={fitKey}
                onUserPan={() => setFollow(false)}
              />

              {waypoints.length > 1 && (
                <Polyline
                  positions={[...waypoints, waypoints[0]].map(ll)}
                  pathOptions={{ color: COLORS.course, weight: 2, opacity: 0.7, dashArray: '6 6' }}
                />
              )}

              {trail.length > 1 && (
                <Polyline
                  positions={trail.map(ll)}
                  pathOptions={{ color: COLORS.self, weight: 2.5, opacity: 0.55 }}
                />
              )}

              {drones.map((d) => {
                const conflict = atOurAltitude(d, altBand);
                return (
                  <Circle
                    key={`ka-${d.id}`}
                    center={[d.lat, d.lon]}
                    radius={d.horizontal_keep_away}
                    pathOptions={{
                      color: conflict ? COLORS.traffic : COLORS.trafficClear,
                      weight: conflict ? 1.5 : 1,
                      dashArray: conflict ? undefined : '2 4',
                      fillOpacity: conflict ? 0.18 : 0.04,
                      opacity: trafficStale ? 0.35 : conflict ? 0.95 : 0.6,
                    }}
                  />
                );
              })}

              {position && target && (
                <Polyline
                  positions={[ll(position), ll(target)]}
                  pathOptions={{
                    color: COLORS.course,
                    weight: 1.5,
                    dashArray: '2 5',
                    opacity: targetStale ? 0.3 : 0.9,
                  }}
                />
              )}
              {position && setpoint && (
                <Polyline
                  positions={[ll(position), ll(setpoint)]}
                  pathOptions={{ color: COLORS.setpoint, weight: 2.5, opacity: setpointStale ? 0.3 : 1 }}
                />
              )}

              {waypoints.map((w, i) => (
                <Marker
                  key={`wp-${i}`}
                  position={ll(w)}
                  icon={waypointIcon(i + 1, i === currentIdx)}
                  zIndexOffset={i === currentIdx ? 100 : 0}
                >
                  <Tooltip direction="top" offset={[0, -10]}>
                    WP {i + 1} · {fmtCoord(w)} · {fmtAlt(w.alt)}
                  </Tooltip>
                </Marker>
              ))}

              {target && currentIdx < 0 && (
                <Marker position={ll(target)} icon={OFF_COURSE_TARGET_ICON} opacity={targetStale ? 0.4 : 1}>
                  <Tooltip direction="top" offset={[0, -10]}>
                    Target · {fmtCoord(target)} · {fmtAlt(target.alt)}
                  </Tooltip>
                </Marker>
              )}

              {drones.map((d) => {
                const conflict = atOurAltitude(d, altBand);
                const dz = verticalOffset(d);
                return (
                  <Marker
                    key={`tr-${d.id}`}
                    position={[d.lat, d.lon]}
                    icon={trafficIcon(d.direction, !conflict)}
                    opacity={trafficStale ? 0.4 : conflict ? 1 : 0.75}
                  >
                    <Tooltip
                      permanent
                      direction="auto"
                      offset={[10, 0]}
                      className={`ims-map-label ${conflict ? '' : 'ims-map-label-clear'}`}
                    >
                      <div className="font-semibold">{trafficName(d.name, d.id)}</div>
                      <div>
                        {distTo(d) != null && `${Math.round(distTo(d)!)} m · `}
                        {Math.round(d.alt)} m alt
                        {dz != null && ` (${fmtVertical(dz)})`} · {d.speed.toFixed(1)} m/s
                      </div>
                      {!conflict && <div>vertically clear</div>}
                    </Tooltip>
                  </Marker>
                );
              })}

              {setpoint && (
                <Marker
                  position={ll(setpoint)}
                  icon={SETPOINT_ICON}
                  opacity={setpointStale ? 0.4 : 1}
                  zIndexOffset={200}
                >
                  <Tooltip direction="top" offset={[0, -12]}>
                    Setpoint · {fmtCoord(setpoint)} · {fmtAlt(setpoint.alt)}
                  </Tooltip>
                </Marker>
              )}

              <OffscreenTraffic drones={drones} stale={trafficStale} distTo={distTo} />

              {position && <Marker position={ll(position)} icon={droneIcon(heading)} zIndexOffset={300} />}
            </MapContainer>

            <div className="absolute right-2 top-2 z-[1000] flex gap-1.5">
              <MapButton
                active={showTraffic}
                onClick={() => setShowTraffic((s) => !s)}
                title="Show AEAC traffic and keep-away zones"
              >
                TRAFFIC
              </MapButton>
              <MapButton onClick={() => setFitKey((k) => k + 1)} title="Fit course, drone and setpoint">
                FIT
              </MapButton>
              <MapButton
                active={follow}
                onClick={() => setFollow((f) => !f)}
                title="Keep the drone centred (dragging the map turns this off)"
              >
                FOLLOW
              </MapButton>
            </div>
          </>
        ) : (
          <div className="flex h-full items-center justify-center text-[12px] text-ink-3">
            Waiting for position, setpoint or course
          </div>
        )}
      </div>

      <footer className="mt-3 flex flex-col gap-1">
        <ReadoutRow
          color={COLORS.self}
          label="Drone"
          coord={position}
          alt={relAlt}
          extra={`hdg ${fmtBearing(heading)}`}
          dim={!position}
        />
        <ReadoutRow
          color={COLORS.setpoint}
          label="Setpoint"
          coord={setpoint}
          alt={setpoint?.alt}
          extra={fmtDist(distTo(setpoint))}
          dim={!setpoint}
          stale={!!setpoint && setpointStale}
        />
        <ReadoutRow
          color={COLORS.course}
          label={targetLabel}
          coord={target}
          alt={target?.alt}
          extra={target ? `${fmtDist(distTo(target))} ${fmtBearing(bearingTo(target))}` : DASH}
          dim={!target}
          stale={!!target && targetStale}
        />
        <ReadoutRow
          color={nearest && !atOurAltitude(nearest.d, altBand) ? COLORS.trafficClear : COLORS.traffic}
          label={nearest ? trafficName(nearest.d.name, nearest.d.id) : 'Nearest'}
          coord={nearest?.d}
          alt={nearest?.d.alt}
          extra={
            nearest
              ? `${fmtDist(nearest.dist)} ${fmtBearing(bearingTo(nearest.d))}` +
                (verticalOffset(nearest.d) != null ? ` ${fmtVertical(verticalOffset(nearest.d)!)}` : '')
              : DASH
          }
          dim={!nearest}
          stale={!!nearest && trafficStale}
        />
      </footer>
    </section>
  );
}
