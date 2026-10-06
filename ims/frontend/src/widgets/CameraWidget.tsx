import type { CameraMessage } from '../types';
import { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';

const DASH = '\u2014';

interface RosImage {
  width: number;
  height: number;
  encoding: string;
  step: number;
  data: string;
}

function decodeImage(message: RosImage): ImageData {
  const { width, height, step, encoding, data } = message;
  // The camera node publishes rgb8; rosbridge encodes uint8[] as base64.
  if (encoding !== 'rgb8') {
    throw new Error(`Unsupported camera encoding: ${encoding}`);
  }
  if (!Number.isSafeInteger(width) || width <= 0 ||
      !Number.isSafeInteger(height) || height <= 0 ||
      !Number.isSafeInteger(step) || step < width * 3 ||
      typeof data !== 'string') {
    throw new Error('Invalid camera image metadata');
  }
  const bytes = atob(data);
  if (bytes.length !== step * height) {
    throw new Error('Invalid camera image data length');
  }
  const image = new ImageData(width, height);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const source = y * step + x * 3;
      const target = (y * width + x) * 4;
      image.data[target] = bytes.charCodeAt(source);
      image.data[target + 1] = bytes.charCodeAt(source + 1);
      image.data[target + 2] = bytes.charCodeAt(source + 2);
      image.data[target + 3] = 255;
    }
  }
  return image;
}

function formatMeta(camera?: CameraMessage): string {
  if (!camera?.width || !camera?.height) return DASH;
  return `${camera.width}\u00D7${camera.height}`;
}

export default function CameraWidget({
  camera,
}: {
  camera?: CameraMessage;
}) {
  const [topic, setTopic] = useState('camera/image_raw');
  const [frame, setFrame] = useState<CameraMessage>();
  const [error, setError] = useState<string>();
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const imageTopic = new ROSLIB.Topic<RosImage>({
      ros,
      name: topic,
      messageType: 'sensor_msgs/Image',
    });
    let active = true;
    const onImage = (message: RosImage) => {
      if (!active) return;
      try {
        const image = decodeImage(message);
        const canvas = canvasRef.current;
        const context = canvas?.getContext('2d');
        if (!canvas || !context) throw new Error('Camera display unavailable');
        if (canvas.width !== image.width) canvas.width = image.width;
        if (canvas.height !== image.height) canvas.height = image.height;
        context.putImageData(image, 0, 0);
        setFrame({ width: image.width, height: image.height });
        setError(undefined);
      } catch (cause) {
        setFrame(undefined);
        setError(cause instanceof Error ? cause.message : 'Invalid camera image');
      }
    };
    const onClose = () => {
      setFrame(undefined);
      setError('Camera connection lost');
    };

    imageTopic.subscribe(onImage);
    ros.on('close', onClose);
    return () => {
      active = false;
      imageTopic.unsubscribe(onImage);
      ros.off('close', onClose);
    };
  }, [topic]);

  const displayedCamera = camera ?? frame;
  const hasImage = camera !== undefined ? !!camera.src : !!frame;
  const displayError = camera === undefined ? error : undefined;

  const pill = hasImage
    ? { className: 'pill-ok', label: 'LIVE' }
    : displayedCamera || displayError
      ? { className: 'pill-warn', label: 'NO SIGNAL' }
      : { className: 'pill bg-edge text-ink-3', label: 'NO DATA' };

  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-y-auto p-4">
      <header className="flex items-center justify-between gap-4">
        <div className="flex items-baseline gap-2">
          <h2 className="widget-label">Camera</h2>
          <span className="font-mono text-[11px] text-ink-3">{formatMeta(displayedCamera)}</span>
        </div>
        <select
          aria-label="Camera source"
          value={topic}
          disabled={camera !== undefined}
          onChange={(event) => {
            setFrame(undefined);
            setError(undefined);
            setTopic(event.target.value);
          }}
          className="rounded bg-feed p-1 text-[11px] text-ink-2"
        >
          <option value="camera/image_raw">Forward</option>
          <option value="/down/camera/image_raw">Downward</option>
        </select>
        <span className={pill.className}>{pill.label}</span>
      </header>

      <div className="relative mt-3 flex-1 overflow-hidden rounded-lg bg-feed">
        <canvas
          ref={canvasRef}
          role="img"
          aria-label="Latest camera frame"
          className={`h-full w-full object-contain ${camera !== undefined || !hasImage ? 'hidden' : ''}`}
        />
        {camera?.src ? (
          <img
            src={camera!.src}
            alt="Latest camera frame"
            className="h-full w-full object-contain"
          />
        ) : !hasImage ? (
          <div className="grid h-full w-full place-items-center text-center">
            <div>
              <p className="text-[13px] text-ink-2">No image</p>
              <p className="mt-1 text-[11px] text-ink-3" role={displayError ? 'alert' : undefined}>
                {displayError ?? 'Check camera downlink'}
              </p>
            </div>
          </div>
        ) : null}
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
