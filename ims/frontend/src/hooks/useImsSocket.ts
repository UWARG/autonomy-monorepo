import { useEffect, useRef, useState } from 'react';
import { IMS_WS_URL, RECONNECT_DELAY_MS, STALE_AFTER_MS } from '../constants';
import type { NearbyDronesMessage } from '../types';

export interface ImsState {
  connected: boolean;
  nearbyDrones?: NearbyDronesMessage;
  /** True until a nearby_drones message arrives, and again if none arrives for STALE_AFTER_MS. */
  nearbyDronesStale: boolean;
}

interface Envelope {
  type: string;
  payload: unknown;
}

/** Subscribes to the IMS relay and keeps the latest message of each type. Reconnects forever. */
export default function useImsSocket(): ImsState {
  const [messages, setMessages] = useState<Pick<ImsState, 'connected' | 'nearbyDrones'>>({
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
