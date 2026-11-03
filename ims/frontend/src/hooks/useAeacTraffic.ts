import { useEffect, useRef, useState } from 'react';
import { AEAC_TOKEN, AEAC_WEBSOCKET_URL, RECONNECT_DELAY_MS, STALE_AFTER_MS } from '../constants';
import type { NearbyDrone, NearbyDronesMessage } from '../types';

/** One entry of the AEAC `traffic` event's payload.traffic list. */
interface AeacTrafficEntry {
  aircraftIndex: number;
  name?: string;
  horizontalKeepAway: number;
  verticalKeepAway: number;
  position: { lat: number; lon: number; altitude: number; speed: number; direction: number };
}

export interface AeacTrafficState {
  /** The WebSocket to the AEAC server is open. */
  connected: boolean;
  nearbyDrones?: NearbyDronesMessage;
  /** Why there is no traffic feed (no token, server error), if known. */
  trafficProblem?: string;
  /** True until a traffic event arrives, and again if none arrives for STALE_AFTER_MS. */
  nearbyDronesStale: boolean;
}

function toDrone(entry: AeacTrafficEntry): NearbyDrone {
  return {
    id: entry.aircraftIndex,
    name: entry.name ?? '',
    lat: entry.position.lat,
    lon: entry.position.lon,
    alt: entry.position.altitude,
    speed: entry.position.speed,
    direction: entry.position.direction,
    horizontal_keep_away: entry.horizontalKeepAway,
    vertical_keep_away: entry.verticalKeepAway,
  };
}

/**
 * Receive-only connection to the AEAC server: it pushes `traffic` at 1 Hz to
 * any team-token connection, without us sending telemetry on it.
 */
export default function useAeacTraffic(): AeacTrafficState {
  const [state, setState] = useState<Omit<AeacTrafficState, 'nearbyDronesStale'>>({
    connected: false,
    trafficProblem: AEAC_TOKEN ? undefined : 'NO TOKEN',
  });
  const [stale, setStale] = useState(true);
  const lastTrafficAt = useRef(0);

  useEffect(() => {
    if (!AEAC_TOKEN) return;

    let socket: WebSocket | null = null;
    let reconnectTimer: number | undefined;
    let closed = false;

    const connect = () => {
      const sep = AEAC_WEBSOCKET_URL.includes('?') ? '&' : '?';
      socket = new WebSocket(`${AEAC_WEBSOCKET_URL}${sep}Authorization=${encodeURIComponent(AEAC_TOKEN)}`);
      socket.onopen = () => setState((s) => ({ ...s, connected: true, trafficProblem: undefined }));
      socket.onclose = () => {
        setState((s) => ({ ...s, connected: false }));
        if (!closed) reconnectTimer = window.setTimeout(connect, RECONNECT_DELAY_MS);
      };
      socket.onmessage = (e: MessageEvent<string>) => {
        let message: { event?: string; payload?: { traffic?: AeacTrafficEntry[] } };
        try {
          message = JSON.parse(e.data);
        } catch {
          return;
        }
        if (message.event === 'traffic' && Array.isArray(message.payload?.traffic)) {
          lastTrafficAt.current = Date.now();
          setStale(false);
          const drones = message.payload.traffic.map(toDrone);
          setState((s) => ({ ...s, trafficProblem: undefined, nearbyDrones: { drones } }));
        } else if (message.event === 'error') {
          setState((s) => ({ ...s, trafficProblem: 'SERVER ERROR' }));
        }
      };
    };
    connect();

    const staleTimer = setInterval(
      () => setStale(Date.now() - lastTrafficAt.current > STALE_AFTER_MS),
      1000,
    );

    return () => {
      closed = true;
      clearInterval(staleTimer);
      window.clearTimeout(reconnectTimer);
      socket?.close();
    };
  }, []);

  return { ...state, nearbyDronesStale: stale };
}
