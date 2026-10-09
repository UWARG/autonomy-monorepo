import { useCallback, useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from './ros';
import type { CaptureImageResponse } from './types';

const CAPTURE_TIMEOUT_MS = 10000;

export type CaptureStatus = 'idle' | 'capturing' | 'ready' | 'failed' | 'timeout';

export interface CaptureResult {
  imageUrl: string;
  location: { lat: number; lon: number; alt: number };
  orientation: { x: number; y: number; z: number; w: number };
}

/** Calls /capture_image over rosbridge (a service, since topics drop large messages). */
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
    const finish = (next: CaptureStatus, nextResult: CaptureResult | null, nextError: string | null) => {
      if (done) return;
      done = true;
      window.clearTimeout(timeoutId);
      statusRef.current = next;
      if (nextResult) onCaptureRef.current?.(nextResult);
      if (!isMountedRef.current) return;
      setResult(nextResult);
      setError(nextError);
      setStatus(next);
    };

    const fail = (next: CaptureStatus, reason: string) => finish(next, null, reason);

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
            imageUrl: `data:image/jpeg;base64,${response.image.data}`,
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
