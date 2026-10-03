/**
 * Session recording — the second data-flow direction.
 *
 *   capture : ROS topics -> JSONL session file
 *   replay  : session file -> same ROS topics
 *
 * A session is JSON Lines: the first line is a {@link SessionHeader}, every
 * following line is a {@link SessionFrame}. Timestamps are client receipt
 * times (ms since `startedAt`) so replay reproduces the arrival cadence the
 * dashboard actually saw; the ROS stamp stays untouched inside `msg`.
 *
 * State lives in this module (not in React) and is exposed through
 * `subscribeSession`/`getSessionSnapshot` for `useSyncExternalStore`, so
 * StrictMode double-effects cannot duplicate subscriptions.
 */
import ROSLIB from 'roslib';
import { ros } from './ros.js';
import { ROS_URL, isCommandTopic } from './constants.ts';

/** Bump when the frame/header shape changes; old files are then rejected. */
export const SESSION_VERSION = 1;

export interface SessionTopic {
  name: string;
  type: string;
}

export interface SessionHeader {
  version: number;
  /** Epoch ms when recording started. */
  startedAt: number;
  /** rosbridge URL the session was captured from. */
  rosUrl: string;
  topics: SessionTopic[];
}

export interface SessionFrame {
  /** Milliseconds since `startedAt`, stamped when the message arrived. */
  t: number;
  topic: string;
  msg: unknown;
}

export interface Recording {
  /** Session bytes; null when it was streamed straight to disk. */
  blob: Blob | null;
  fileName: string;
  frames: number;
  bytes: number;
}

type SinkKind = 'file' | 'memory';

interface SessionSink {
  kind: SinkKind;
  write(line: string): Promise<void>;
  /** Finalize; returns the downloadable blob, or null when already on disk. */
  close(): Promise<Blob | null>;
  abort(): Promise<void>;
}

export interface RecorderSnapshot {
  recording: boolean;
  startedAt: number | null;
  /** Milliseconds since `startedAt`; frozen at the last write once stopped. */
  elapsedMs: number;
  frames: number;
  bytes: number;
  sinkKind: SinkKind | null;
  topics: SessionTopic[];
  error: string | null;
}

export interface PlayerSnapshot {
  fileName: string | null;
  header: SessionHeader | null;
  totalFrames: number;
  durationMs: number;
  cursorMs: number;
  playing: boolean;
  speed: number;
  published: number;
  /** Frames the safety filter refused to publish. */
  blocked: number;
  error: string | null;
}

export interface SessionSnapshot {
  recorder: RecorderSnapshot;
  player: PlayerSnapshot;
}

// ---------------------------------------------------------------------------
// Sinks
// ---------------------------------------------------------------------------

interface PickWritable {
  write(data: string): Promise<void>;
  close(): Promise<void>;
  abort?(reason?: unknown): Promise<void>;
}
interface PickFileHandle {
  createWritable(): Promise<PickWritable>;
}
type ShowSaveFilePicker = (options?: {
  suggestedName?: string;
  types?: { description?: string; accept: Record<string, string[]> }[];
}) => Promise<PickFileHandle>;

type PickerWindow = { showSaveFilePicker?: ShowSaveFilePicker };

/** True when the browser can stream a session straight to a user-picked file. */
export const canStreamToFile = (): boolean =>
  typeof (window as unknown as PickerWindow).showSaveFilePicker === 'function';

function memorySink(): SessionSink {
  const lines: string[] = [];
  return {
    kind: 'memory',
    write: async (line) => {
      lines.push(line);
    },
    close: async () =>
      new Blob([`${lines.join('\n')}\n`], { type: 'application/x-ndjson' }),
    abort: async () => {
      lines.length = 0;
    },
  };
}

/**
 * Streams lines to a user-picked file so a long session never has to be held
 * in memory. Must be invoked from a click handler — the picker needs user
 * activation.
 */
async function fileSink(fileName: string, onError: (err: unknown) => void): Promise<SessionSink> {
  const pick = (window as unknown as PickerWindow).showSaveFilePicker;
  if (!pick) throw new Error('File System Access API unavailable');
  const handle = await pick({
    suggestedName: fileName,
    types: [
      { description: 'IMS session (JSON Lines)', accept: { 'application/x-ndjson': ['.jsonl'] } },
    ],
  });
  const writable = await handle.createWritable();
  // Writes are serialized so lines never interleave; a failed write must not
  // poison the chain for every later line.
  let chain: Promise<void> = Promise.resolve();
  return {
    kind: 'file',
    write: (line) => {
      chain = chain
        .then(() => writable.write(`${line}\n`))
        .catch((err) => onError(err));
      return chain;
    },
    close: async () => {
      await chain;
      await writable.close();
      return null;
    },
    abort: async () => {
      await chain;
      await writable.abort?.().catch(() => undefined);
    },
  };
}

// ---------------------------------------------------------------------------
// Store
// ---------------------------------------------------------------------------

const EMPTY_RECORDER: RecorderSnapshot = {
  recording: false,
  startedAt: null,
  elapsedMs: 0,
  frames: 0,
  bytes: 0,
  sinkKind: null,
  topics: [],
  error: null,
};

const EMPTY_PLAYER: PlayerSnapshot = {
  fileName: null,
  header: null,
  totalFrames: 0,
  durationMs: 0,
  cursorMs: 0,
  playing: false,
  speed: 1,
  published: 0,
  blocked: 0,
  error: null,
};

let snapshot: SessionSnapshot = { recorder: EMPTY_RECORDER, player: EMPTY_PLAYER };
const listeners = new Set<() => void>();

function emit(patch: Partial<SessionSnapshot>): void {
  snapshot = { ...snapshot, ...patch };
  for (const listener of listeners) listener();
}

export function subscribeSession(listener: () => void): () => void {
  listeners.add(listener);
  return () => {
    listeners.delete(listener);
  };
}

export function getSessionSnapshot(): SessionSnapshot {
  return snapshot;
}

// ---------------------------------------------------------------------------
// Topic discovery
// ---------------------------------------------------------------------------

/** Topics currently advertised on the ROS graph, with their message types. */
export function getTopicTypes(): Promise<SessionTopic[]> {
  const { promise, resolve, reject } = Promise.withResolvers<SessionTopic[]>();
  ros.getTopics(
    (result: { topics: string[]; types: string[] }) =>
      resolve(result.topics.map((name, i) => ({ name, type: result.types[i] }))),
    (error: unknown) => reject(error instanceof Error ? error : new Error(String(error))),
  );
  return promise;
}

// ---------------------------------------------------------------------------
// Recorder
// ---------------------------------------------------------------------------

let recStartedAt: number | null = null;
let recFrames = 0;
let recBytes = 0;
let recError: string | null = null;
let recSink: SessionSink | null = null;
let recSubs: ROSLIB.Topic[] = [];
let recFileName = '';
let lastRecEmit = 0;

function pushRecorder(force = false): void {
  const now = Date.now();
  if (!force && now - lastRecEmit < 100) return;
  lastRecEmit = now;
  emit({
    recorder: {
      recording: recSink != null,
      startedAt: recStartedAt,
      elapsedMs:
        recSink != null && recStartedAt != null ? now - recStartedAt : snapshot.recorder.elapsedMs,
      frames: recFrames,
      bytes: recBytes,
      sinkKind: recSink?.kind ?? null,
      topics: snapshot.recorder.topics,
      error: recError,
    },
  });
}

const sessionFileName = (startedAt: number): string =>
  `ims-session-${new Date(startedAt).toISOString().replace(/[:.]/g, '-')}.jsonl`;

/**
 * Start capturing `topics`. Opens the file picker first (it needs the click's
 * user activation); if the picker is unavailable or dismissed, recording falls
 * back to an in-memory buffer that `stopRecording` hands back as a download.
 */
export async function startRecording(
  topics: SessionTopic[],
  opts: { toFile?: boolean } = {},
): Promise<void> {
  if (snapshot.player.playing) {
    recError = 'Stop replay before recording.';
    pushRecorder(true);
    return;
  }
  if (recSink) return;
  if (topics.length === 0) {
    recError = 'No topics selected.';
    pushRecorder(true);
    return;
  }

  const startedAt = Date.now();
  recFileName = sessionFileName(startedAt);
  const onSinkError = (err: unknown) => {
    recError = `Session file write failed: ${String(err)}`;
    pushRecorder(true);
  };

  let sink: SessionSink;
  let notice: string | null = null;
  if (opts.toFile ?? true) {
    if (canStreamToFile()) {
      try {
        sink = await fileSink(recFileName, onSinkError);
      } catch {
        // Picker dismissed or blocked — keep the capture, buffer it in memory.
        sink = memorySink();
        notice = 'File picker dismissed — recording to memory; you can download it on stop.';
      }
    } else {
      sink = memorySink();
      notice = 'This browser cannot stream to a file — recording to memory.';
    }
  } else {
    sink = memorySink();
  }

  const header: SessionHeader = { version: SESSION_VERSION, startedAt, rosUrl: ROS_URL, topics };
  try {
    await sink.write(JSON.stringify(header));
  } catch (err) {
    await sink.abort();
    recError = `Could not write session header: ${String(err)}`;
    pushRecorder(true);
    return;
  }

  const subs: ROSLIB.Topic[] = [];
  for (const { name, type } of topics) {
    const topic = new ROSLIB.Topic({ ros, name, messageType: type });
    topic.subscribe((msg) => onFrame(name, msg, startedAt));
    subs.push(topic);
  }

  recStartedAt = startedAt;
  recFrames = 0;
  recBytes = 0;
  recSink = sink;
  recSubs = subs;
  recError = notice;
  emit({
    recorder: {
      recording: true,
      startedAt,
      elapsedMs: 0,
      frames: 0,
      bytes: 0,
      sinkKind: sink.kind,
      topics,
      error: recError,
    },
  });
}

function onFrame(topicName: string, msg: unknown, startedAt: number): void {
  if (!recSink) return;
  const frame: SessionFrame = { t: Date.now() - startedAt, topic: topicName, msg };
  let line: string;
  try {
    line = JSON.stringify(frame);
  } catch {
    return; // un-serializable payload (should not happen for rosbridge JSON)
  }
  recFrames += 1;
  recBytes += line.length; // JSON payloads are ASCII-dominant
  void recSink.write(line).catch((err) => {
    recError = `Session file write failed: ${String(err)}`;
    pushRecorder(true);
  });
  pushRecorder();
}

export async function stopRecording(): Promise<Recording | null> {
  const sink = recSink;
  if (!sink) return null;
  recSink = null;
  for (const topic of recSubs) topic.unsubscribe();
  recSubs = [];
  pushRecorder(true);

  let blob: Blob | null = null;
  try {
    blob = await sink.close();
  } catch (err) {
    recError = `Could not finalize session file: ${String(err)}`;
    pushRecorder(true);
  }
  return { blob, fileName: recFileName, frames: recFrames, bytes: recBytes };
}

// ---------------------------------------------------------------------------
// Player
// ---------------------------------------------------------------------------

const PLAY_TICK_MS = 25;

let playerFrames: SessionFrame[] = [];
let playerPubs = new Map<string, ROSLIB.Topic<unknown>>();
let playerTimer: number | null = null;
let playerStartWall = 0;
let playerNext = 0;
let pubCount = 0;
let blockedCount = 0;
let lastPlayerEmit = 0;

const elapsedMs = (): number =>
  playerTimer == null ? snapshot.player.cursorMs : (Date.now() - playerStartWall) * snapshot.player.speed;

function pushPlayer(force = false): void {
  const now = Date.now();
  if (!force && now - lastPlayerEmit < 60) return;
  lastPlayerEmit = now;
  emit({
    player: {
      ...snapshot.player,
      cursorMs: elapsedMs(),
      published: pubCount,
      blocked: blockedCount,
    },
  });
}

export async function loadSessionFile(file: File): Promise<void> {
  if (recSink) {
    emit({ player: { ...snapshot.player, error: 'Stop recording before loading a session.' } });
    return;
  }
  try {
    const frames = parseSession(await file.text());
    stopPlayback();
    playerFrames = frames.frames;
    playerNext = 0;
    pubCount = 0;
    blockedCount = 0;
    emit({
      player: {
        fileName: file.name,
        header: frames.header,
        totalFrames: frames.frames.length,
        durationMs: frames.durationMs,
        cursorMs: 0,
        playing: false,
        speed: snapshot.player.speed,
        published: 0,
        blocked: 0,
        error: null,
      },
    });
  } catch (err) {
    emit({ player: { ...snapshot.player, error: `Could not read session: ${String(err)}` } });
  }
}

function parseSession(text: string): {
  header: SessionHeader;
  frames: SessionFrame[];
  durationMs: number;
} {
  const lines = text.split('\n').filter((line) => line.trim().length > 0);
  if (lines.length === 0) throw new Error('file is empty');
  const header = JSON.parse(lines[0]) as SessionHeader;
  if (header?.version !== SESSION_VERSION) {
    throw new Error(`unsupported session version ${String(header?.version)}`);
  }
  if (!Array.isArray(header.topics)) throw new Error('header has no topic table');
  const frames: SessionFrame[] = [];
  for (let i = 1; i < lines.length; i++) {
    let frame: SessionFrame;
    try {
      frame = JSON.parse(lines[i]) as SessionFrame;
    } catch {
      throw new Error(`malformed frame on line ${i + 1}`);
    }
    if (typeof frame?.t !== 'number' || typeof frame?.topic !== 'string') {
      throw new Error(`malformed frame on line ${i + 1}`);
    }
    frames.push(frame);
  }
  frames.sort((a, b) => a.t - b.t);
  return {
    header,
    frames,
    durationMs: frames.length ? frames[frames.length - 1].t : 0,
  };
}

export function playSession(): void {
  if (playerTimer != null || playerFrames.length === 0) return;
  if (recSink) {
    emit({ player: { ...snapshot.player, error: 'Stop recording before replaying.' } });
    return;
  }

  const header = snapshot.player.header;
  if (header) {
    playerPubs = new Map(
      header.topics.map(({ name, type }) => [
        name,
        new ROSLIB.Topic<unknown>({ ros, name, messageType: type }),
      ]),
    );
  }

  const cursor = snapshot.player.cursorMs >= snapshot.player.durationMs ? 0 : snapshot.player.cursorMs;
  // At cursor 0 nothing has been published yet, so the frame at exactly 0 belongs.
  playerNext = playerFrames.findIndex((frame) => (cursor > 0 ? frame.t > cursor : frame.t >= cursor));
  if (playerNext === -1) playerNext = playerFrames.length;
  pubCount = 0;
  blockedCount = 0;
  playerStartWall = Date.now();
  emit({ player: { ...snapshot.player, cursorMs: cursor, playing: true, error: null } });
  if (cursor > 0) playerStartWall -= cursor / snapshot.player.speed;
  playerTimer = window.setInterval(tick, PLAY_TICK_MS);
  tick();
}

function tick(): void {
  const { speed, durationMs } = snapshot.player;
  const elapsed = (Date.now() - playerStartWall) * speed;

  while (playerNext < playerFrames.length && playerFrames[playerNext].t <= elapsed) {
    const frame = playerFrames[playerNext++];
    if (isCommandTopic(frame.topic)) {
      blockedCount += 1;
      continue;
    }
    const publisher = playerPubs.get(frame.topic);
    if (!publisher) {
      blockedCount += 1;
      continue;
    }
    try {
      publisher.publish(frame.msg);
      pubCount += 1;
    } catch (err) {
      emit({ player: { ...snapshot.player, error: `Publish failed on ${frame.topic}: ${String(err)}` } });
      stopPlayback();
      return;
    }
  }

  if (playerNext >= playerFrames.length && elapsed >= durationMs) {
    stopPlayback();
    emit({ player: { ...snapshot.player, cursorMs: durationMs } });
    return;
  }
  pushPlayer();
}

export function pausePlayback(): void {
  if (playerTimer == null) return;
  const cursor = elapsedMs();
  stopPlayback();
  emit({ player: { ...snapshot.player, cursorMs: Math.min(cursor, snapshot.player.durationMs) } });
}

export function stopPlayback(): void {
  if (playerTimer != null) {
    window.clearInterval(playerTimer);
    playerTimer = null;
  }
  for (const publisher of playerPubs.values()) publisher.unadvertise();
  playerPubs = new Map();
  emit({ player: { ...snapshot.player, playing: false, published: pubCount, blocked: blockedCount } });
}

/** Restart playback from the beginning; no-op while stopped at zero. */
export function rewindSession(): void {
  stopPlayback();
  playerNext = 0;
  pubCount = 0;
  blockedCount = 0;
  emit({ player: { ...snapshot.player, cursorMs: 0, published: 0, blocked: 0 } });
}

export function setPlaybackSpeed(speed: number): void {
  const cursor = playerTimer != null ? elapsedMs() : snapshot.player.cursorMs;
  if (playerTimer != null) playerStartWall = Date.now() - cursor / speed;
  emit({ player: { ...snapshot.player, speed, cursorMs: Math.min(cursor, snapshot.player.durationMs) } });
}
