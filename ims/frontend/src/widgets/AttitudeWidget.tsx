import type { AttitudeMessage } from '../types';
import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js'

const DASH = '\u2014';
const DEG = 180 / Math.PI;

/** Degrees of pitch from horizon centre to the visible edge of the ball. */
const PITCH_RANGE_DEG = 25;

const toDeg = (rad: number) => rad * DEG;

/** Wrap yaw into 0..359 for a compass-style heading readout. */
function headingDeg(yawRad: number): number {
  const d = Math.round(toDeg(yawRad)) % 360;
  return d < 0 ? d + 360 : d;
}

function Readout({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex flex-col items-center gap-0.5">
      <span className="widget-label">{label}</span>
      <span className="font-mono text-sm font-semibold tabular-nums text-ink">
        {value}
      </span>
    </div>
  );
}

interface PoseStamped {
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}
 
function quaternionToEuler(q: { w: number; x: number; y: number; z: number }): {
  roll: number;
  pitch: number;
  yaw: number;
} {
  const sinrCosp = 2 * (q.w * q.x + q.y * q.z);
  const cosrCosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  const roll = Math.atan2(sinrCosp, cosrCosp);
 
  const sinp = Math.max(-1, Math.min(1, 2 * (q.w * q.y - q.z * q.x)));
  const pitch = Math.asin(sinp);
 
  const sinyCosp = 2 * (q.w * q.z + q.x * q.y);
  const cosyCosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);
 
  return { roll: roll, pitch: pitch, yaw: yaw };
}

/**
 * Artificial horizon. 
 */
function Horizon({ attitude }: { attitude?: AttitudeMessage }) {
  const rollDeg = attitude ? toDeg(attitude.roll) : 0;
  const pitchDeg = attitude ? toDeg(attitude.pitch) : 0;

  // px the horizon shifts per degree of pitch, given the ball radius (90) and
  // the visible pitch range.
  const pxPerDeg = 90 / PITCH_RANGE_DEG;
  const pitchShift = pitchDeg * pxPerDeg;

  // Pitch ladder rungs every 10deg within range.
  const rungs: number[] = [];
  for (let d = -PITCH_RANGE_DEG + 5; d <= PITCH_RANGE_DEG - 5; d += 5) {
    if (d !== 0) rungs.push(d);
  }

  return (
    <svg viewBox="0 0 200 200" className="h-full w-full" role="img"
      aria-label={attitude ? 'Attitude indicator' : 'Attitude indicator, no data'}>
      <defs>
        <clipPath id="att-ball">
          <circle cx="100" cy="100" r="90" />
        </clipPath>
      </defs>

      {/* rotating + translating sky/ground ball */}
      <g clipPath="url(#att-ball)">
        <g transform={`rotate(${rollDeg} 100 100)`}>
          <g transform={`translate(0 ${pitchShift})`}>
            <rect x="-100" y="-200" width="400" height="300" fill="var(--att-sky)" />
            <rect x="-100" y="100" width="400" height="300" fill="var(--att-ground)" />
            <line x1="-100" y1="100" x2="300" y2="100"
              stroke="#EAEEF5" strokeWidth="2" />

            {rungs.map((d) => {
              const y = 100 - d * pxPerDeg;
              const w = d % 10 === 0 ? 34 : 18;
              return (
                <line key={d} x1={100 - w / 2} y1={y} x2={100 + w / 2} y2={y}
                  stroke="#EAEEF5" strokeWidth="1.5" opacity="0.85" />
              );
            })}
          </g>
        </g>
      </g>

      {/* ball rim */}
      <circle cx="100" cy="100" r="90" fill="none"
        stroke="var(--att-edge)" strokeWidth="2" />

      {/* roll tick marks at top (fixed to frame) */}
      {[-30, -15, 15, 30].map((a) => {
        const rad = (a - 90) * (Math.PI / 180);
        const x1 = 100 + Math.cos(rad) * 90;
        const y1 = 100 + Math.sin(rad) * 90;
        const x2 = 100 + Math.cos(rad) * 82;
        const y2 = 100 + Math.sin(rad) * 82;
        return <line key={a} x1={x1} y1={y1} x2={x2} y2={y2}
          stroke="var(--att-tick)" strokeWidth="2" />;
      })}
      {/* roll pointer (fixed triangle at top) */}
      <path d="M100 10 l6 11 l-12 0 z" fill="var(--att-accent)" />

      {/* fixed aircraft reference */}
      <g stroke="var(--att-accent)" strokeWidth="3" fill="none">
        <line x1="62" y1="100" x2="86" y2="100" />
        <line x1="114" y1="100" x2="138" y2="100" />
      </g>
      <circle cx="100" cy="100" r="3.5" fill="var(--att-accent)" />
    </svg>
  );
}

export default function AttitudeWidget() {
  const [attitude, setAttitude] = useState<AttitudeMessage>();

  useEffect(() => {
    const poseTopic = new ROSLIB.Topic<PoseStamped>({
      ros,
      name: '/mavros/local_position/pose',
      messageType: 'geometry_msgs/PoseStamped',
    });

    const onPose = (message: PoseStamped) => {
      const { x, y, z, w } = message.pose.orientation;
      const { roll, pitch, yaw } = quaternionToEuler({ w, x, y, z });

      /* Unsure of what to put in roll, pitch, yaw speeds */
      setAttitude({ roll, pitch, yaw });
    }

    poseTopic.subscribe(onPose);
    return () => poseTopic.unsubscribe(onPose);
  }, []);
  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-hidden p-4"
      style={{
        ['--att-sky' as string]: '#BFE0F5',
        ['--att-ground' as string]: '#6E5744',
        ['--att-edge' as string]: '#252C3A',
        ['--att-tick' as string]: '#5E687A',
        ['--att-accent' as string]: '#5B8DEF',
      }}>
      <header className="flex items-center justify-between">
        <h2 className="widget-label">Attitude</h2>
        {!attitude && <span className="pill bg-edge text-ink-3">NO DATA</span>}
      </header>

      <div className="mx-auto my-3 aspect-square w-full min-h-0 max-w-[220px] flex-1">
        <Horizon attitude={attitude} />
      </div>

      <div className="grid grid-cols-3 gap-2">
        <Readout label="Roll" value={attitude ? `${Math.round(toDeg(attitude.roll))}\u00B0` : DASH} />
        <Readout label="Pitch" value={attitude ? `${Math.round(toDeg(attitude.pitch))}\u00B0` : DASH} />
        <Readout label="Yaw" value={attitude ? `${headingDeg(attitude.yaw)}\u00B0` : DASH} />
      </div>
    </section>
  );
}