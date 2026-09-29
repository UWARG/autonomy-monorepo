import { useCallback, useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from './ros.js';
import type { TriggeredImageCapture } from './types';
import { rosImageToDataUrl } from './rosImage';
import { makeTestImageDataUrl } from './testImage';

const TRIGGER_TIMEOUT_MS = 5000;

export type CaptureStatus = 'idle' | 'capturing' | 'ready' | 'timeout';

export interface CaptureResult {
  forwardImageUrl: string;
  downwardImageUrl: string;
  location: { lat: number; lon: number; alt: number };
  orientation: { x: number; y: number; z: number; w: number };
  rangeM: number;
}

/**
 * Publishes /TriggerImageCapture and waits for the next /TriggeredImageCapture
 * response, over a direct rosbridge connection (see ros.js). There's no
 * request/response over ROS topics, so this is a simple "publish, then take
 * the next message or time out" correlation — fine for a single-operator,
 * one-at-a-time capture flow.
 */
export function useTriggeredCapture(onCapture?: (result: CaptureResult) => void) {
  const [status, setStatus] = useState<CaptureStatus>('idle');
  const [result, setResult] = useState<CaptureResult | null>(null);
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

    const responseTopic = new ROSLIB.Topic<TriggeredImageCapture>({
      ros,
      name: '/TriggeredImageCapture',
      messageType: 'airside_interfaces/msg/TriggeredImageCapture',
    });

    let timeoutId: number | null = null;
    const finish = (next: CaptureStatus, nextResult: CaptureResult | null) => {
      if (timeoutId !== null) window.clearTimeout(timeoutId);
      responseTopic.unsubscribe(onResponse);
      statusRef.current = next;
      if (!isMountedRef.current) return;
      if (nextResult) setResult(nextResult);
      setStatus(next);
    };

    const onResponse = (msg: TriggeredImageCapture) => {
      const captureResult: CaptureResult = {
        forwardImageUrl: rosImageToDataUrl(msg.forward_image),
        downwardImageUrl: rosImageToDataUrl(msg.downward_image),
        location: msg.location,
        orientation: msg.imu.orientation,
        rangeM: msg.range.range,
      };
      onCaptureRef.current?.(captureResult);
      finish('ready', captureResult);
    };

    responseTopic.subscribe(onResponse);
    timeoutId = window.setTimeout(() => {
      // No response from the drone — fall back to an obviously-fake test
      // image so the rest of the app (gallery, survey form) stays usable
      // while the ROS backend isn't running.
      const fallback: CaptureResult = {
        forwardImageUrl: makeTestImageDataUrl('forward camera — no response'),
        downwardImageUrl: makeTestImageDataUrl('downward camera — no response'),
        location: { lat: 0, lon: 0, alt: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
        rangeM: 0,
      };
      onCaptureRef.current?.(fallback);
      finish('timeout', fallback);
    }, TRIGGER_TIMEOUT_MS);

    const triggerTopic = new ROSLIB.Topic({
      ros,
      name: '/TriggerImageCapture',
      messageType: 'airside_interfaces/msg/TriggerImageCapture',
    });
    triggerTopic.publish(new ROSLIB.Message({ command: 'capture' }));
  }, []);

  return { status, result, capture };
}
