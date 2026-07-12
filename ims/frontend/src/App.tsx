// Grid (lg):
//   row 1:  Camera (5)   Target (4)    Attitude (3)
//   row 2:  Connection (4)        Script (8)
//   row 3:  Log (12)

import AttitudeWidget from './widgets/AttitudeWidget';
import CameraWidget from './widgets/CameraWidget';
import ConnectionWidget from './widgets/ConnectionWidget';
import LogWidget from './widgets/LogWidget';
import ScriptWidget from './widgets/ScriptWidget';
import TargetWidget from './widgets/TargetWidget';
import { useEffect, useState } from 'react';
import { subscribe, unsubscribe } from './socket';
import type { AttitudeMessage, ConnectionMessage } from './types';

export default function App() {
  // undefined until the first 'connection' payload arrives 
  const [connection, setConnection] = useState<ConnectionMessage>();
  const [attitude, setAttitude] = useState<AttitudeMessage>();

  useEffect(() => {
    const onConnection = (payload: ConnectionMessage) => setConnection(payload);
    const onAttitude = (payload: AttitudeMessage) => setAttitude(payload);
    subscribe('connection', onConnection);
    subscribe('attitude', onAttitude);


    return () => {
      unsubscribe('connection', onConnection);
      unsubscribe('attitude', onAttitude);
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
        <div className="col-span-12 lg:col-span-5">
          <CameraWidget />
        </div>
        <div className="col-span-12 md:col-span-7 lg:col-span-4">
          <TargetWidget />
        </div>
        <div className="col-span-12 md:col-span-5 lg:col-span-3">
          <AttitudeWidget attitude={attitude} />
        </div>
        <div className="col-span-12 md:col-span-6 lg:col-span-4">
          <ConnectionWidget connection={connection} />
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