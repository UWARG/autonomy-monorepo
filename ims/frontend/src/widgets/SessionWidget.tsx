import { useEffect, useMemo, useRef, useState, useSyncExternalStore } from 'react';
import {
  type SessionTopic,
  canStreamToFile,
  getSessionSnapshot,
  getTopicTypes,
  loadSessionFile,
  pausePlayback,
  playSession,
  rewindSession,
  setPlaybackSpeed,
  startRecording,
  stopRecording,
  subscribeSession,
} from '../session';
import { RECORDED_TOPICS } from '../constants';

const DASH = '\u2014';

const SPEEDS = [0.5, 1, 2, 4];

function formatDuration(ms: number): string {
  const total = Math.max(0, ms) / 1000;
  const mm = Math.floor(total / 60);
  const ss = total - mm * 60;
  return `${mm}:${ss.toFixed(1).padStart(4, '0')}`;
}

function formatBytes(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KiB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MiB`;
}

function download(blob: Blob, fileName: string): void {
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = fileName;
  // Firefox/Safari read the blob asynchronously after the click, so the URL
  // cannot be revoked in the same tick; keep the anchor attached until then.
  document.body.append(anchor);
  anchor.click();
  setTimeout(() => {
    URL.revokeObjectURL(url);
    anchor.remove();
  }, 0);
}

function Row({ label, value, tone = 'text-ink' }: { label: string; value: string; tone?: string }) {
  return (
    <div className="flex items-baseline justify-between gap-3">
      <dt className="text-[12px] text-ink-3">{label}</dt>
      <dd className={`font-mono text-[12px] tabular-nums ${tone}`}>{value}</dd>
    </div>
  );
}

function TopicPicker({
  topics,
  selected,
  disabled,
  onToggle,
}: {
  topics: SessionTopic[];
  selected: Set<string>;
  disabled: boolean;
  onToggle: (name: string) => void;
}) {
  return (
    <div className="mt-2 flex-1 overflow-y-auto rounded-md border border-edge bg-inset p-2">
      {topics.length === 0 ? (
        <p className="p-1 text-[12px] text-ink-3">No topics advertised yet</p>
      ) : (
        topics.map(({ name, type }) => (
          <label
            key={name}
            className={`flex cursor-pointer items-center gap-2 rounded px-1.5 py-1 text-[12px] ${
              disabled ? 'cursor-not-allowed opacity-50' : 'hover:bg-card'
            }`}
          >
            <input
              type="checkbox"
              className="accent-accent"
              checked={selected.has(name)}
              disabled={disabled}
              onChange={() => onToggle(name)}
            />
            <span className="font-mono text-ink">{name}</span>
            <span className="ml-auto truncate font-mono text-[11px] text-ink-3">{type}</span>
          </label>
        ))
      )}
    </div>
  );
}

export default function SessionWidget() {
  const { recorder, player } = useSyncExternalStore(subscribeSession, getSessionSnapshot);
  const [topics, setTopics] = useState<SessionTopic[]>([]);
  const [graphError, setGraphError] = useState<string | null>(null);
  const [selected, setSelected] = useState<Set<string>>(() => new Set(RECORDED_TOPICS));
  const [streamToFile, setStreamToFile] = useState(true);
  const fileInput = useRef<HTMLInputElement>(null);

  const refreshTopics = () => {
    getTopicTypes()
      .then((list) => {
        setTopics(list);
        setGraphError(null);
      })
      .catch((error) => setGraphError(String(error)));
  };

  useEffect(refreshTopics, []);

  const selectedTopics = useMemo(
    () => topics.filter(({ name }) => selected.has(name)),
    [topics, selected],
  );

  const onToggle = (name: string) => {
    setSelected((prev) => {
      const next = new Set(prev);
      if (next.has(name)) next.delete(name);
      else next.add(name);
      return next;
    });
  };

  const onStart = async () => {
    await startRecording(selectedTopics, { toFile: streamToFile });
  };

  const onStop = async () => {
    await stopRecording();
  };

  const onDownload = () => {
    const finished = recorder.lastRecording;
    if (finished?.blob) download(finished.blob, finished.fileName);
  };

  const { lastRecording } = recorder;

  const replayable = player.header != null && player.totalFrames > 0;
  const progress = player.durationMs > 0 ? Math.min(1, player.cursorMs / player.durationMs) : 0;
  const pill = recorder.recording
    ? { className: 'pill-bad', label: 'REC' }
    : player.playing
      ? { className: 'pill-accent', label: 'REPLAYING' }
      : { className: 'pill bg-edge text-ink-3', label: 'IDLE' };

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <div className="flex items-baseline gap-2">
          <h2 className="widget-label">Session Recording</h2>
          <span className="font-mono text-[11px] text-ink-3">jsonl</span>
        </div>
        <span className={pill.className}>
          {recorder.recording && <span className="status-dot bg-current" aria-hidden="true" />}
          {pill.label}
        </span>
      </header>

      <div className="mt-3 grid min-h-0 flex-1 grid-cols-1 gap-4 md:grid-cols-2">
        {/* ---------------------------------------------------------------- */}
        <div className="flex min-h-0 flex-col">
          <div className="flex items-center justify-between">
            <h3 className="widget-label">Record — ROS to file</h3>
            <button
              type="button"
              onClick={refreshTopics}
              className="text-[11px] text-ink-3 hover:text-ink-2"
            >
              refresh topics
            </button>
          </div>

          <TopicPicker
            topics={topics}
            selected={selected}
            disabled={recorder.recording}
            onToggle={onToggle}
          />

          {graphError && <p className="mt-1 text-[11px] text-bad">rosapi: {graphError}</p>}

          <label
            className={`mt-2 flex items-center gap-2 text-[12px] ${
              canStreamToFile() ? 'text-ink-2' : 'text-ink-3'
            }`}
          >
            <input
              type="checkbox"
              className="accent-accent"
              checked={streamToFile && canStreamToFile()}
              disabled={!canStreamToFile() || recorder.recording}
              onChange={(event) => setStreamToFile(event.target.checked)}
            />
            {canStreamToFile()
              ? 'Save straight to a file (unbounded length)'
              : 'File streaming unsupported here — buffered in memory'}
          </label>

          <div className="mt-3 flex items-center gap-3">
            {recorder.recording ? (
              <button
                type="button"
                onClick={onStop}
                className="flex-1 rounded-md border border-edge bg-bad-dim py-2 text-[13px] font-semibold text-bad"
              >
                Stop recording
              </button>
            ) : (
              <button
                type="button"
                onClick={onStart}
                disabled={selectedTopics.length === 0 || player.playing}
                className="flex-1 rounded-md border border-edge bg-card/60 py-2 text-[13px] font-semibold text-ink-2 disabled:cursor-not-allowed disabled:opacity-40"
              >
                Start recording
              </button>
            )}
            {lastRecording?.blob && (
              <button
                type="button"
                onClick={onDownload}
                className="rounded-md border border-edge bg-accent-dim px-3 py-2 text-[13px] font-semibold text-accent"
              >
                Download
              </button>
            )}
          </div>

          <dl className="mt-3 grid grid-cols-2 gap-x-4 gap-y-1">
            <Row label="Elapsed" value={formatDuration(recorder.elapsedMs)} />
            <Row label="Frames" value={recorder.frames.toLocaleString()} />
            <Row label="Size" value={formatBytes(recorder.bytes)} />
            <Row
              label="Destination"
              value={
                recorder.recording ? (recorder.sinkKind === 'file' ? 'file' : 'memory') : DASH
              }
            />
          </dl>

          {recorder.error && <p className="mt-2 text-[11px] text-warn">{recorder.error}</p>}
          {!recorder.error && lastRecording && !lastRecording.blob && (
            <p className="mt-2 text-[11px] text-ink-3">
              Saved {lastRecording.frames.toLocaleString()} frames to {lastRecording.fileName}
            </p>
          )}
        </div>

        {/* ---------------------------------------------------------------- */}
        <div className="flex min-h-0 flex-col">
          <div className="flex items-center justify-between">
            <h3 className="widget-label">Replay — file to ROS</h3>
            <input
              ref={fileInput}
              type="file"
              accept=".jsonl,application/x-ndjson"
              className="hidden"
              onChange={(event) => {
                const file = event.target.files?.[0];
                event.target.value = '';
                if (file) void loadSessionFile(file);
              }}
            />
            <button
              type="button"
              onClick={() => fileInput.current?.click()}
              disabled={recorder.recording}
              className="text-[11px] text-ink-3 hover:text-ink-2 disabled:opacity-40"
            >
              open session…
            </button>
          </div>

          <div className="mt-2 rounded-md border border-edge bg-inset p-2">
            <p className="truncate font-mono text-[12px] text-ink">
              {player.fileName ?? 'No session loaded'}
            </p>
            {player.header && (
              <dl className="mt-2 grid grid-cols-2 gap-x-4 gap-y-1">
                <Row label="Frames" value={player.totalFrames.toLocaleString()} />
                <Row label="Duration" value={formatDuration(player.durationMs)} />
                <Row label="Topic types" value={String(player.header.topics.length)} />
                <Row label="Published" value={player.published.toLocaleString()} tone="text-ok" />
                <Row
                  label="Blocked (control)"
                  value={player.blocked.toLocaleString()}
                  tone={player.blocked ? 'text-warn' : 'text-ink-3'}
                />
              </dl>
            )}
          </div>

          <div className="mt-3 h-2 overflow-hidden rounded-full bg-edge">
            <div
              className="h-full rounded-full bg-accent transition-[width]"
              style={{ width: `${progress * 100}%` }}
            />
          </div>
          <div className="mt-1 flex justify-between text-[11px] text-ink-3">
            <span className="font-mono">{formatDuration(player.cursorMs)}</span>
            <span className="font-mono">{formatDuration(player.durationMs)}</span>
          </div>

          <div className="mt-3 flex items-center gap-3">
            <button
              type="button"
              disabled={!replayable || recorder.recording}
              onClick={() => (player.playing ? pausePlayback() : playSession())}
              className="flex-1 rounded-md border border-edge bg-accent-dim py-2 text-[13px] font-semibold text-accent disabled:cursor-not-allowed disabled:opacity-40"
            >
              {player.playing ? 'Pause' : 'Play'}
            </button>
            <button
              type="button"
              disabled={!replayable}
              onClick={rewindSession}
              className="rounded-md border border-edge bg-card/60 px-3 py-2 text-[13px] font-semibold text-ink-2 disabled:cursor-not-allowed disabled:opacity-40"
            >
              Restart
            </button>
            <label className="flex items-center gap-1.5 text-[11px] text-ink-3">
              Speed
              <select
                value={player.speed}
                disabled={!replayable}
                onChange={(event) => setPlaybackSpeed(Number(event.target.value))}
                className="rounded border border-edge bg-inset px-1.5 py-1 font-mono text-[12px] text-ink disabled:opacity-40"
              >
                {SPEEDS.map((speed) => (
                  <option key={speed} value={speed}>
                    {speed}
                    {'\u00D7'}
                  </option>
                ))}
              </select>
            </label>
          </div>

          <p className="mt-2 text-[11px] text-ink-3">
            Replay only republishes the topics the recorder tracks, onto their original names;
            everything else — setpoints, /cmd_vel, RC, unknown topics — is blocked and counted.
          </p>
          {player.error && <p className="mt-1 text-[11px] text-bad">{player.error}</p>}
        </div>
      </div>
    </section>
  );
}
