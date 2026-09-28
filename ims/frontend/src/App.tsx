// Grid (lg):
//   row 1:  Camera (5)   Target (4)    Attitude (3)
//   row 2:  Connection (3)   Script (5)   Nearby drones (4, spans rows 2-3)
//   row 3:  Log (8)

import useImsSocket from './hooks/useImsSocket';
import AttitudeWidget from './widgets/AttitudeWidget';
import CameraWidget from './widgets/CameraWidget';
import ConnectionWidget from './widgets/ConnectionWidget';
import NearbyDronesWidget from './widgets/NearbyDronesWidget';
import ScriptWidget from './widgets/ScriptWidget';
import TargetWidget from './widgets/TargetWidget';
import LogWidget from './widgets/LogWidget';
import HeaderStatus from './widgets/Header';

export default function App() {
  const ims = useImsSocket();

  return (
    <div className="flex min-h-screen flex-col lg:h-screen lg:overflow-hidden">
    <header className="shrink-0 border-b border-edge bg-card">
          <div className="mx-auto flex max-w-[1400px] items-center gap-4 px-5 py-3">
            <span className="font-mono text-lg font-bold tracking-tight">IMS</span>
            <span className="widget-label">Ground Station</span>
            <div className="ml-auto">
              <HeaderStatus />
            </div>
          </div>
        </header>


      <main
        className="mx-auto grid w-full max-w-[1400px] grid-cols-12 gap-4 px-5 py-4
                   lg:min-h-0 lg:flex-1 lg:overflow-y-auto
                   lg:[grid-template-rows:minmax(220px,5fr)_minmax(180px,4fr)_minmax(150px,3fr)]"
      >
        <div className="col-span-12 h-[340px] min-h-0 lg:col-span-5 lg:h-auto">
          <CameraWidget />
        </div>
        <div className="col-span-12 h-[340px] min-h-0 md:col-span-7 lg:col-span-4 lg:h-auto">
          <TargetWidget />
        </div>
        <div className="col-span-12 h-[340px] min-h-0 md:col-span-5 lg:col-span-3 lg:h-auto">
          <AttitudeWidget />
        </div>
        <div className="col-span-12 h-[240px] min-h-0 md:col-span-6 lg:col-span-3 lg:h-auto">
          <ConnectionWidget />
        </div>
        <div className="col-span-12 h-[240px] min-h-0 md:col-span-6 lg:col-span-5 lg:h-auto">
          <ScriptWidget />
        </div>
        <div className="col-span-12 h-[380px] min-h-0 lg:col-span-4 lg:row-span-2 lg:h-auto">
          <NearbyDronesWidget
            nearby={ims.nearbyDrones}
            connected={ims.connected}
            stale={ims.nearbyDronesStale}
          />
        </div>
        <div className="col-span-12 h-[220px] min-h-0 lg:col-span-8 lg:h-auto">
          <LogWidget />
        </div>
      </main>
    </div>
  );
}
