import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';
import { TOPICS } from '../topics';

const DASH = '\u2014';

interface RosImage {
  width: number;
  height: number;
  encoding: string;
  step: number;
  data?: string | number[];
}

/**
 * `sensor_msgs/Image` carries the frame itself in `data`, which rosbridge hands
 * over as base64 — 640x480 rgb8 is ~900 kB per frame at 50 Hz. This panel shows
 * metadata only, so the subscription is throttled and `data` is discarded.
 * Shipping actual pixels needs a video path (web_video_server, or a compressed
 * transport), not this topic at full rate.
 */
const METADATA_THROTTLE_MS = 2000;
/** camera_node publishes at 50 Hz; older than this means the feed stopped. */
const STALE_AFTER_MS = 5000;
const RATE_WINDOW_MS = 5000;
const TICK_MS = 1000;

interface CameraMeta {
  width: number;
  height: number;
  encoding: string;
  step: number;
  frameBytes: number;
  frames: number;
  fps: number | null;
  ageMs: number;
}

/** Bytes a frame occupies on the wire: base64 is 3 bytes per 4 characters. */
function frameBytes(data: RosImage['data']): number {
  if (typeof data === 'string') return Math.floor((data.length * 3) / 4);
  if (Array.isArray(data)) return data.length;
  return 0;
}

export default function CameraWidget() {
  const [meta, setMeta] = useState<CameraMeta | null>(null);

  useEffect(() => {
    const topic = new ROSLIB.Topic<RosImage>({
      ros,
      name: TOPICS.camera.name,
      messageType: TOPICS.camera.type,
      throttle_rate: METADATA_THROTTLE_MS,
      queue_length: 1,
    });

    let frames = 0;
    let beats: number[] = [];
    let lastAt = 0;
    let latest: Omit<CameraMeta, 'frames' | 'fps' | 'ageMs'> | null = null;

    const publish = () => {
      if (!latest || lastAt === 0) return;
      const now = Date.now();
      beats = beats.filter((at) => now - at <= RATE_WINDOW_MS);
      const span = beats.length > 1 ? beats[beats.length - 1] - beats[0] : 0;
      setMeta({
        ...latest,
        frames,
        fps: span > 0 ? ((beats.length - 1) * 1000) / span : null,
        // Age comes from the newest arrival, not the rate window: once the
        // window empties the frame is stale, not fresh.
        ageMs: now - lastAt,
      });
    };

    topic.subscribe((message) => {
      frames += 1;
      lastAt = Date.now();
      beats.push(lastAt);
      latest = {
        width: message.width,
        height: message.height,
        encoding: message.encoding,
        step: message.step,
        frameBytes: frameBytes(message.data),
      };
      publish();
    });

    const ticker = window.setInterval(publish, TICK_MS);
    return () => {
      window.clearInterval(ticker);
      topic.unsubscribe();
    };
  }, []);

  const stale = meta != null && meta.ageMs > STALE_AFTER_MS;
  const pill = !meta
    ? { className: 'pill bg-edge text-ink-3', label: 'NO DATA' }
    : stale
      ? { className: 'pill-warn', label: 'STALE' }
      : { className: 'pill-ok', label: 'LIVE' };

  const resolution =
    meta && meta.width && meta.height ? `${meta.width}\u00D7${meta.height}` : DASH;

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <div className="flex items-baseline gap-2">
          <h2 className="widget-label">Camera</h2>
          <span className="font-mono text-[11px] text-ink-3">{resolution}</span>
        </div>
        <span className={pill.className}>{pill.label}</span>
      </header>

      <div className="mt-3 flex flex-1 flex-col justify-center rounded-lg bg-feed p-4">
        {meta ? (
          <dl className="grid grid-cols-2 gap-x-4 gap-y-2">
            <Meta label="Resolution" value={resolution} />
            <Meta label="Encoding" value={meta.encoding || DASH} />
            <Meta label="Row stride" value={`${meta.step} B`} />
            <Meta
              label="Frame size"
              value={meta.frameBytes ? `${(meta.frameBytes / 1024).toFixed(0)} KiB` : DASH}
            />
          </dl>
        ) : (
          <p className="text-center text-[13px] text-ink-3">
            Awaiting {TOPICS.camera.name}
          </p>
        )}
      </div>

      <footer className="mt-3 flex items-center justify-between text-[11px]">
        <span className="flex items-center gap-1.5">
          <span
            className={`status-dot ${meta && !stale ? 'bg-ok' : 'bg-bad'}`}
            aria-hidden="true"
          />
          <span className={meta && !stale ? 'text-ink-2' : 'text-ink-3'}>
            {meta && !stale ? 'RECEIVING' : 'IDLE'}
          </span>
        </span>
        <span className="font-mono text-ink-3">
          {meta?.fps != null ? `${meta.fps.toFixed(1)} Hz` : DASH} ·{' '}
          {meta ? `${meta.frames} frames` : 'no frames'} · metadata only
        </span>
      </footer>
    </section>
  );
}

function Meta({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-baseline justify-between gap-3">
      <dt className="text-[12px] text-ink-3">{label}</dt>
      <dd className="font-mono text-[12px] tabular-nums text-ink">{value}</dd>
    </div>
  );
}
