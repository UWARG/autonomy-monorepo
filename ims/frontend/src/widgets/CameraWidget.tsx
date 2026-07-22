import type { CameraMessage } from '../types';

const DASH = '—';

function formatMeta(camera?: CameraMessage): string {
  if (!camera?.width || !camera?.height) return DASH;
  return `${camera.width}\u00D7${camera.height}`;
}

export default function CameraWidget({
  camera,
}: {
  camera?: CameraMessage;
}) {
  const hasImage = !!camera?.src;

  const pill = hasImage
    ? { className: 'pill-ok', label: 'LIVE' }
    : camera
      ? { className: 'pill-warn', label: 'NO SIGNAL' }
      : { className: 'pill bg-edge text-ink-3', label: 'NO DATA' };

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <div className="flex items-baseline gap-2">
          <h2 className="widget-label">Camera</h2>
          <span className="font-mono text-[11px] text-ink-3">{formatMeta(camera)}</span>
        </div>
        <span className={pill.className}>{pill.label}</span>
      </header>

      <div className="relative mt-3 flex-1 overflow-hidden rounded-lg bg-feed">
        {hasImage ? (
          <img
            src={camera!.src}
            alt="Latest camera frame"
            className="h-full w-full object-contain"
          />
        ) : (
          <div className="grid h-full w-full place-items-center text-center">
            <div>
              <p className="text-[13px] text-ink-2">No image</p>
              <p className="mt-1 text-[11px] text-ink-3">Check camera downlink</p>
            </div>
          </div>
        )}
      </div>

      <footer className="mt-3 flex items-center justify-between text-[11px]">
        <span className="flex items-center gap-1.5">
          <span
            className={`status-dot ${hasImage ? 'bg-ok' : 'bg-bad'}`}
            aria-hidden="true"
          />
          <span className={hasImage ? 'text-ink-2' : 'text-ink-3'}>
            {hasImage ? 'RECEIVING' : 'OFFLINE'}
          </span>
        </span>
        <span className="font-mono text-ink-3">
          {camera?.latencyMs != null ? `${(camera.latencyMs / 1000).toFixed(1)}s latency` : DASH}
        </span>
      </footer>
    </section>
  );
}