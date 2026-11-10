import { useEffect, useState } from 'react';
import { ros } from '../ros.js';
import { createRosbagClient } from '../rosbagClient';
import type { RosbagClient, RosbagState } from '../rosbagClient';

export default function RosbagWidget({ client: suppliedClient }: { client?: RosbagClient }) {
  const [client] = useState(() => suppliedClient ?? createRosbagClient(ros));
  const [state, setState] = useState<RosbagState>({ connected: false, status: null, error: null });
  const [pending, setPending] = useState<'start' | 'stop' | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => client.subscribe(setState), [client]);

  const recording = state.status?.recording ?? false;
  const available = state.connected && state.status !== null;
  const label = pending === 'start' ? 'Starting…'
    : pending === 'stop' ? 'Stopping…'
      : recording ? 'Stop Recording' : 'Start Recording';
  const statusLabel = !state.connected ? 'ROS disconnected'
    : !state.status ? 'Waiting for recording controller'
      : recording ? 'Recording' : 'Idle';
  const visibleError = error ?? state.error ?? state.status?.error;

  async function toggle() {
    if (pending || !available) return;
    setError(null);
    setPending(recording ? 'stop' : 'start');
    try {
      await client.setRecording(!recording);
    } catch (failure) {
      setError(failure instanceof Error ? failure.message : String(failure));
    } finally {
      setPending(null);
    }
  }

  return (
    <section className="widget flex h-full min-h-[160px] flex-col gap-3 p-4">
      <header className="flex items-center justify-between gap-3">
        <h2 className="widget-label">Rosbag Recording</h2>
        <span role="status" className={recording && available ? 'pill-bad' : 'pill bg-edge text-ink-3'}>
          {statusLabel}
        </span>
      </header>
      {visibleError && <p role="alert" className="text-[13px] text-bad">{visibleError}</p>}
      <button
        type="button"
        onClick={() => { void toggle(); }}
        disabled={!available || pending !== null}
        className={`rounded-md border border-edge bg-card/60 py-2 text-[13px] font-semibold
          disabled:cursor-not-allowed disabled:opacity-40 ${recording ? 'text-bad' : 'text-ink-2'}`}
      >
        {label}
      </button>
      {state.status?.bag_path && (
        <p className="break-all font-mono text-[11px] text-ink-3">{state.status.bag_path}</p>
      )}
    </section>
  );
}
