import { useEffect, useRef, useState } from 'react';
import { IMS_WS_URL, RECONNECT_DELAY_MS, STALE_AFTER_MS } from '../constants';
import type {
  AeacAckMessage,
  AeacInfractionMessage,
  NearbyDronesMessage,
  TelemetrySentMessage,
} from '../types';

export interface ImsState {
  connected: boolean;
  nearbyDrones?: NearbyDronesMessage;
  telemetrySent?: TelemetrySentMessage;
  /** Date.now() when telemetrySent arrived, so its age doesn't depend on any other clock. */
  telemetrySentAtMs?: number;
  aeacAck?: AeacAckMessage;
  aeacAckAtMs?: number;
  aeacInfraction?: AeacInfractionMessage;
  /** True until a nearby_drones message arrives, and again if none arrives for STALE_AFTER_MS. */
  nearbyDronesStale: boolean;
}

interface Envelope {
  type: string;
  payload: unknown;
}

/** Subscribes to the IMS relay and keeps the latest message of each type. Reconnects forever. */
export default function useImsSocket(): ImsState {
  const [messages, setMessages] = useState<Omit<ImsState, 'nearbyDronesStale'>>({
    connected: false,
  });
  const [stale, setStale] = useState(true);
  const lastNearbyAt = useRef(0);

  useEffect(() => {
    let socket: WebSocket | null = null;
    let retryTimer: ReturnType<typeof setTimeout> | undefined;
    let disposed = false;

    const open = () => {
      socket = new WebSocket(IMS_WS_URL);

      socket.onopen = () => setMessages((m) => ({ ...m, connected: true }));

      socket.onmessage = (event) => {
        let envelope: Envelope;
        try {
          envelope = JSON.parse(event.data as string) as Envelope;
        } catch {
          return;
        }
        if (envelope.type === 'nearby_drones') {
          lastNearbyAt.current = Date.now();
          setStale(false);
          setMessages((m) => ({ ...m, nearbyDrones: envelope.payload as NearbyDronesMessage }));
        } else if (envelope.type === 'telemetry_sent') {
          const telemetrySent = envelope.payload as TelemetrySentMessage;
          setMessages((m) => ({ ...m, telemetrySent, telemetrySentAtMs: Date.now() }));
        } else if (envelope.type === 'aeac_ack') {
          const aeacAck = envelope.payload as AeacAckMessage;
          setMessages((m) => ({ ...m, aeacAck, aeacAckAtMs: Date.now() }));
        } else if (envelope.type === 'aeac_infraction') {
          const aeacInfraction = envelope.payload as AeacInfractionMessage;
          setMessages((m) => ({ ...m, aeacInfraction }));
        }
      };

      socket.onclose = () => {
        setMessages((m) => ({ ...m, connected: false }));
        if (!disposed) retryTimer = setTimeout(open, RECONNECT_DELAY_MS);
      };
    };

    open();
    const staleTimer = setInterval(
      () => setStale(Date.now() - lastNearbyAt.current > STALE_AFTER_MS),
      1000,
    );

    return () => {
      disposed = true;
      clearTimeout(retryTimer);
      clearInterval(staleTimer);
      socket?.close();
    };
  }, []);

  return { ...messages, nearbyDronesStale: stale };
}
