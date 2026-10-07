import { useCallback, useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from './ros.js';
import type { CaptureImageResponse } from './types';
import { rosImageToDataUrl } from './rosImage';
import { makeTestImageDataUrl } from './testImage';

const CAPTURE_TIMEOUT_MS = 10000;

export type CaptureStatus = 'idle' | 'capturing' | 'ready' | 'failed' | 'timeout';

export interface CaptureResult {
  imageUrl: string;
  location: { lat: number; lon: number; alt: number };
  orientation: { x: number; y: number; z: number; w: number };
}

/**
 * Calls the /capture_image service over a direct rosbridge connection (see
 * ros.js). A service rather than a trigger/response topic pair: rosbridge
 * subscribes to topics best-effort, which drops large one-off messages, while
 * service replies are reliable and matched to their request.
 */
export function useTriggeredCapture(onCapture?: (result: CaptureResult) => void) {
  const [status, setStatus] = useState<CaptureStatus>('idle');
  const [result, setResult] = useState<CaptureResult | null>(null);
  const [error, setError] = useState<string | null>(null);
  const statusRef = useRef<CaptureStatus>('idle');
  const isMountedRef = useRef(true);
  const onCaptureRef = useRef(onCapture);

  useEffect(() => {
    onCaptureRef.current = onCapture;
  }, [onCapture]);

  useEffect(() => {
    isMountedRef.current = true;
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  const capture = useCallback(() => {
    if (statusRef.current === 'capturing') return;
    statusRef.current = 'capturing';
    setStatus('capturing');
    setError(null);

    let done = false;
    const finish = (next: CaptureStatus, nextResult: CaptureResult, nextError: string | null) => {
      if (done) return;
      done = true;
      window.clearTimeout(timeoutId);
      statusRef.current = next;
      onCaptureRef.current?.(nextResult);
      if (!isMountedRef.current) return;
      setResult(nextResult);
      setError(nextError);
      setStatus(next);
    };

    // Without a usable reply, fall back to an obviously-fake test image so the
    // rest of the app (gallery, survey form) stays usable.
    const fail = (next: CaptureStatus, reason: string) =>
      finish(
        next,
        {
          imageUrl: makeTestImageDataUrl(`camera — ${reason}`),
          location: { lat: 0, lon: 0, alt: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
        reason,
      );

    const timeoutId = window.setTimeout(() => fail('timeout', 'no response'), CAPTURE_TIMEOUT_MS);

    const service = new ROSLIB.Service<Record<string, never>, CaptureImageResponse>({
      ros,
      name: '/capture_image',
      serviceType: 'airside_interfaces/srv/CaptureImage',
    });
    service.callService(
      {},
      (response) => {
        if (!response.success) {
          fail('failed', response.message);
          return;
        }
        finish(
          'ready',
          {
            imageUrl: rosImageToDataUrl(response.image),
            location: response.location,
            orientation: response.imu.orientation,
          },
          null,
        );
      },
      (serviceError) => fail('failed', serviceError),
    );
  }, []);

  return { status, result, error, capture };
}
