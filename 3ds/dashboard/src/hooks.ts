import { useEffect, useState } from "react";
import type { Connection, Transport } from "./ros";
import type { TopicInfo } from "./models";
export function useConnection(transport: Transport) {
  const [connection, setConnection] = useState<{
    state: Connection;
    error?: string;
  }>({ state: "connecting" });
  useEffect(
    () => transport.start((state, error) => setConnection({ state, error })),
    [transport],
  );
  return connection;
}
export function useTopics(transport: Transport, state: Connection) {
  const [result, setResult] = useState<{
    transport: Transport;
    topics: TopicInfo[];
    error?: string;
  }>({ transport, topics: [] });
  useEffect(() => {
    setResult((old) => ({
      transport,
      topics: old.transport === transport ? old.topics : [],
      error: "Topic list stale: waiting for rosbridge discovery",
    }));
    if (state !== "connected") return;
    let active = true;
    const refresh = async () => {
      try {
        const topics = await transport.discover();
        if (active) setResult({ transport, topics });
      } catch (error) {
        if (active)
          setResult((old) => ({
            ...old,
            error: `Topic list stale: ${String(error)}`,
          }));
      }
    };
    void refresh();
    const timer = setInterval(() => void refresh(), 10000);
    return () => {
      active = false;
      clearInterval(timer);
    };
  }, [transport, state]);
  const topics = result.transport === transport ? result.topics : [];
  return {
    topics,
    error:
      state !== "connected"
        ? "Topic list stale: waiting for rosbridge"
        : result.transport !== transport
          ? "Waiting for rosbridge discovery"
          : result.error,
  };
}
export interface Sample<T> {
  value?: T;
  received?: number;
  error?: string;
  times: number[];
}
export function useSubscription<T>(
  transport: Transport,
  state: Connection,
  name: string,
  type: string,
  parse: (value: unknown) => T,
  throttle = 500,
): Sample<T> {
  const [sample, setSample] = useState<Sample<T>>({ times: [] });
  // Keying the returned value prevents displaying a previous topic or connection's data.
  const key = `${state}:${name}:${type}`;
  const [storedKey, setStoredKey] = useState(key);
  useEffect(() => {
    let active = true;
    setSample({ times: [] });
    setStoredKey(key);
    if (state !== "connected" || !name) return;
    const stop = transport.subscribe(
      name,
      type,
      (value) => {
        if (!active) return;
        try {
          const parsed = parse(value);
          const now = Date.now();
          setSample((old) => ({
            value: parsed,
            received: now,
            times: [
              ...old.times.filter((t) => now - t < 10000).slice(-29),
              now,
            ],
          }));
        } catch (error) {
          setSample((old) => ({ ...old, error: String(error) }));
        }
      },
      throttle,
    );
    return () => {
      active = false;
      stop();
    };
  }, [transport, state, name, type, parse, throttle, key]);
  return storedKey === key ? sample : { times: [] };
}
export function useNow() {
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);
  return now;
}
export function sampleStatus<T>(
  sample: Sample<T>,
  now: number,
  connected: boolean,
  missing = false,
): string {
  if (!connected) return "Disconnected";
  if (missing) return "Topic unavailable";
  if (sample.error) return `Invalid data: ${sample.error}`;
  if (!sample.received) return "Waiting for data";
  const age = Math.max(0, (now - sample.received) / 1000);
  return `${age > 5 ? "STALE · " : ""}${age.toFixed(0)}s since receipt · ${new Date(sample.received).toLocaleTimeString()}`;
}
