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
    <div className="flex min-h-screen flex-col lg:h-screen lg:overflow-hidden">
      <header className="shrink-0 border-b border-edge bg-card">
        <div className="mx-auto flex max-w-[1400px] items-center gap-4 px-5 py-3">
          <span className="font-mono text-lg font-bold tracking-tight">IMS</span>
          <span className="widget-label">Ground Station</span>
        </div>
      </header>

      <main
        className="mx-auto grid w-full max-w-[1400px] grid-cols-12 gap-4 px-5 py-4
                   lg:min-h-0 lg:flex-1 lg:overflow-y-auto
                   lg:[grid-template-rows:minmax(220px,5fr)_minmax(180px,4fr)_minmax(150px,3fr)]"
      >
        <div className="col-span-12 h-[340px] min-h-0 lg:col-span-5 lg:h-auto">
          <CameraWidget camera={camera} />
        </div>
        <div className="col-span-12 h-[340px] min-h-0 md:col-span-7 lg:col-span-4 lg:h-auto">
          <TargetWidget position={position} target={target} trail={trail} />
        </div>
        <div className="col-span-12 h-[340px] min-h-0 md:col-span-5 lg:col-span-3 lg:h-auto">
          <AttitudeWidget attitude={attitude} />
        </div>
        <div className="col-span-12 h-[240px] min-h-0 md:col-span-6 lg:col-span-4 lg:h-auto">
          <ConnectionWidget connection={connection} />
        </div>
        <div className="col-span-12 h-[240px] min-h-0 md:col-span-6 lg:col-span-8 lg:h-auto">
          <ScriptWidget status={status} />
        </div>
        <div className="col-span-12 h-[220px] min-h-0 lg:h-auto">
          <LogWidget entries={log} />
        </div>
      </main>
    </div>
  );
}