import type { CameraMessage } from '../types';

const DASH = '\u2014';

/** Header metadata line, e.g. "1080p · H.264 · 30 fps". Falls back to dashes. */
function formatMeta(camera?: CameraMessage): string {
  if (!camera) return DASH;
  const res = camera.height ? `${camera.height}p` : DASH;
  const codec = camera.encoding ?? DASH;
  const fps = camera.fps != null ? `${camera.fps} fps` : DASH;
  return `${res} \u00B7 ${codec} \u00B7 ${fps}`;
}

export default function CameraWidget({
  camera,
}: {
  camera?: CameraMessage;
}) {
  const online = camera?.online ?? false;

  const pill = online
    ? { className: 'pill-ok', label: 'LIVE' }
    : camera
      ? { className: 'pill-warn', label: 'NO SIGNAL' }
      : { className: 'pill bg-edge text-ink-3', label: 'NO DATA' };

  return (
    <section className="widget flex h-full min-h-[120px] flex-col p-4">
      <header className="flex items-center justify-between gap-4">
        <div className="flex items-baseline gap-2">
          <h2 className="widget-label">Camera</h2>
          <span className="font-mono text-[11px] text-ink-3">{formatMeta(camera)}</span>
        </div>
        <span className={pill.className}>{pill.label}</span>
      </header>

      {/* Video surface. Feed frames arrive over a separate downlink, not the
          telemetry socket ** when that exists, mount the decoder's
          <video>/<canvas> here in place of the placeholder. */}
      <div className="relative mt-3 flex-1 overflow-hidden rounded-lg bg-feed">
        {online ? (
          <div
            className="grid h-full w-full place-items-center text-[13px] text-ink-3"
            aria-label="Live camera feed"
          >
            {/* stream element mounts here! */}
          </div>
        ) : (
          <div className="grid h-full w-full place-items-center text-center">
            <div>
              <p className="text-[13px] text-ink-2">No video signal</p>
              <p className="mt-1 text-[11px] text-ink-3">Check camera downlink</p>
            </div>
          </div>
        )}
      </div>

      <footer className="mt-3 flex items-center justify-between text-[11px]">
        <span className="flex items-center gap-1.5">
          <span
            className={`status-dot ${online ? 'bg-ok' : 'bg-bad'}`}
            aria-hidden="true"
          />
          <span className={online ? 'text-ink-2' : 'text-ink-3'}>
            {online ? 'LIVE' : 'OFFLINE'}
          </span>
        </span>
        <span className="font-mono text-ink-3">
          {camera?.latencyMs != null ? `${(camera.latencyMs / 1000).toFixed(1)}s latency` : DASH}
        </span>
      </footer>
    </section>
  );
}