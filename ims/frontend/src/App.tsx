// Grid (lg):
//   row 1:  Camera (5)   Target (4)    Attitude (3)
//   row 2:  Connection (4)        Script (8)
//   row 3:  Log (12)

import AttitudeWidget from './widgets/AttitudeWidget';
import CameraWidget from './widgets/CameraWidget';
import ConnectionWidget from './widgets/ConnectionWidget';
import ScriptWidget from './widgets/ScriptWidget';
import TargetWidget, { type TrailSample } from './widgets/TargetWidget';
import LogWidget, { type LogEntry } from './widgets/LogWidget';
import { useEffect, useState } from 'react';
import { subscribe, unsubscribe } from './socket';
import type {
  AttitudeMessage,
  CameraMessage,
  ConnectionMessage,
  PositionMessage,
  LogMessage,
  StatusMessage,
  TargetMessage,
} from './types';

const TRAIL_MAX = 300; 
const LOG_MAX = 200;

export default function App() {
  // Each is undefined until its first payload arrives; widgets render NO DATA
  // meanwhile. Nothing broadcasts until airside is up and SOCKET_URL is filled.
  const [attitude, setAttitude] = useState<AttitudeMessage>();
  const [connection, setConnection] = useState<ConnectionMessage>();
  const [camera, setCamera] = useState<CameraMessage>();
  const [position, setPosition] = useState<PositionMessage>();
  const [target, setTarget] = useState<TargetMessage>();
  const [status, setStatus] = useState<StatusMessage>();
  const [log, setLog] = useState<LogEntry[]>([]);
  const [trail, setTrail] = useState<TrailSample[]>([]);

  useEffect(() => {
    const onAttitude = (p: AttitudeMessage) => setAttitude(p);
    const onConnection = (p: ConnectionMessage) => setConnection(p);
    const onCamera = (p: CameraMessage) => setCamera(p);
    const onPosition = (p: PositionMessage) => {
      setPosition(p);
      setTrail((prev) =>
        [...prev, { lat: p.lat, lon: p.lon, t: Date.now() }].slice(-TRAIL_MAX),
      );
    };
    const onTarget = (p: TargetMessage) => setTarget(p);
    const onStatus = (p: StatusMessage) => setStatus(p);
    const onLog = (p: LogMessage) =>
      setLog((prev) => [...prev, { message: p.message, t: Date.now() }].slice(-LOG_MAX));
    subscribe('attitude', onAttitude);
    subscribe('connection', onConnection);
    subscribe('camera', onCamera);
    subscribe('position', onPosition);
    subscribe('target', onTarget);
    subscribe('status', onStatus);
    subscribe('log', onLog);

    return () => {
      unsubscribe('attitude', onAttitude);
      unsubscribe('connection', onConnection);
      unsubscribe('camera', onCamera);
      unsubscribe('position', onPosition);
      unsubscribe('target', onTarget);
      unsubscribe('status', onStatus);
      unsubscribe('log', onLog);
    };
  }, []);

  return (
    <div className="min-h-screen">
      <header className="sticky top-0 z-20 border-b border-edge bg-card">
        <div className="mx-auto flex max-w-[1400px] items-center gap-4 px-5 py-3">
          <span className="font-mono text-lg font-bold tracking-tight">IMS</span>
          <span className="widget-label">Ground Station</span>
        </div>
      </header>

      <main className="mx-auto grid max-w-[1400px] grid-cols-12 gap-4 px-5 py-5">
        <div className="col-span-12 h-[340px] lg:col-span-5">
          <CameraWidget camera={camera} />
        </div>
        <div className="col-span-12 h-[340px] md:col-span-7 lg:col-span-4">
          <TargetWidget position={position} target={target} trail={trail} />
        </div>
        <div className="col-span-12 h-[340px] md:col-span-5 lg:col-span-3">
          <AttitudeWidget attitude={attitude} />
        </div>
        <div className="col-span-12 h-[280px] md:col-span-6 lg:col-span-4">
          <ConnectionWidget connection={connection} />
        </div>
        <div className="col-span-12 h-[280px] md:col-span-6 lg:col-span-8">
          <ScriptWidget status={status} />
        </div>
        <div className="col-span-12 h-[220px]">
          <LogWidget entries={log} />
        </div>
      </main>
    </div>
  );
}