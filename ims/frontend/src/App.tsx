// Grid (lg):
//   row 1:  Camera (5)   Target (4)    Attitude (3)
//   row 2:  Connection (4)        Script (8)
//   row 3:  Log (12)

import { useEffect, useState } from 'react';
import { subscribe, unsubscribe } from './socket';

import AttitudeWidget from './widgets/AttitudeWidget';
import CameraWidget from './widgets/CameraWidget';
import ConnectionWidget from './widgets/ConnectionWidget';
import LogWidget from './widgets/LogWidget';
import ScriptWidget from './widgets/ScriptWidget';
import TargetWidget from './widgets/TargetWidget';

// Drone-connected indicator, driven by heartbeat messages over the socket.
function ConnectionIndicator() {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const onHeartbeat = () => setConnected(true);
    subscribe('heartbeat', onHeartbeat); // TODO: confirm type name
    return () => unsubscribe('heartbeat', onHeartbeat);
  }, []);

  return connected ? (
    <span className="pill-ok">
      <span className="status-dot bg-ok" />
      CONNECTED
    </span>
  ) : (
    <span className="pill-bad">
      <span className="status-dot bg-bad" />
      DISCONNECTED
    </span>
  );
}

export default function App() {
  return (
    <div className="min-h-screen">
      {/* Header — always visible */}
      <header className="sticky top-0 z-20 border-b border-edge bg-card">
        <div className="mx-auto flex max-w-[1400px] items-center gap-4 px-5 py-3">
          <span className="font-mono text-lg font-bold tracking-tight">IMS</span>
          <span className="widget-label">Ground Station</span>
          <div className="flex-1" />
          <ConnectionIndicator />
        </div>
      </header>

      {/* Six widget spaces */}
      <main className="mx-auto grid max-w-[1400px] grid-cols-12 gap-4 px-5 py-5">
        <div className="col-span-12 lg:col-span-5">
          <CameraWidget />
        </div>
        <div className="col-span-12 md:col-span-7 lg:col-span-4">
          <TargetWidget />
        </div>
        <div className="col-span-12 md:col-span-5 lg:col-span-3">
          <AttitudeWidget />
        </div>
        <div className="col-span-12 md:col-span-6 lg:col-span-4">
          <ConnectionWidget />
        </div>
        <div className="col-span-12 md:col-span-6 lg:col-span-8">
          <ScriptWidget />
        </div>
        <div className="col-span-12">
          <LogWidget />
        </div>
      </main>
    </div>
  );
}