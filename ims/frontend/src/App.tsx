// Flight view grid (lg):
//   row 1:  Camera (5)   Target (4)    Attitude (3)
//   row 2:  Script (8)   Nearby drones (4, spans rows 2-3)
//   row 3:  Connection (3)   Log (5)
//
// Survey view grid (lg): Capture (7)   Survey report (5)

import { useState } from 'react';
import useAeacTraffic from './hooks/useAeacTraffic';
import AttitudeWidget from './widgets/AttitudeWidget';
import CameraWidget from './widgets/CameraWidget';
import CaptureWidget from './widgets/CaptureWidget';
import ConnectionWidget from './widgets/ConnectionWidget';
import NearbyDronesWidget from './widgets/NearbyDronesWidget';
import ScriptWidget from './widgets/ScriptWidget';
import SurveyWidget from './widgets/SurveyWidget';
import TargetWidget from './widgets/TargetWidget';
import LogWidget from './widgets/LogWidget';
import HeaderStatus from './widgets/Header';

type View = 'flight' | 'survey';

const VIEWS: { id: View; label: string }[] = [
  { id: 'flight', label: 'Flight' },
  { id: 'survey', label: 'Survey' },
];

function FlightView() {
  const traffic = useAeacTraffic();

  return (
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
      <div className="col-span-12 h-[240px] min-h-0 lg:col-span-8 lg:h-auto">
        <ScriptWidget />
      </div>
      <div className="col-span-12 h-[380px] min-h-0 lg:col-span-4 lg:row-span-2 lg:h-auto">
        <NearbyDronesWidget
          nearby={traffic.nearbyDrones}
          connected={traffic.connected}
          stale={traffic.nearbyDronesStale}
          problem={traffic.trafficProblem}
        />
      </div>
      <div className="col-span-12 h-[220px] min-h-0 md:col-span-5 lg:col-span-3 lg:h-auto">
        <ConnectionWidget />
      </div>
      <div className="col-span-12 h-[220px] min-h-0 md:col-span-7 lg:col-span-5 lg:h-auto">
        <LogWidget />
      </div>
    </main>
  );
}

function SurveyView({ active }: { active: boolean }) {
  return (
    <main className="mx-auto grid w-full max-w-[1400px] grid-cols-12 gap-4 px-5 py-4 lg:min-h-0 lg:flex-1">
      <div className="col-span-12 h-[520px] min-h-0 lg:col-span-7 lg:h-auto">
        <CaptureWidget hotkeyEnabled={active} />
      </div>
      <div className="col-span-12 min-h-0 lg:col-span-5">
        <SurveyWidget />
      </div>
    </main>
  );
}

export default function App() {
  const [view, setView] = useState<View>('flight');

  return (
    <div className="flex min-h-screen flex-col lg:h-screen lg:overflow-hidden">
      <header className="shrink-0 border-b border-edge bg-card">
        <div className="mx-auto flex max-w-[1400px] items-center gap-4 px-5 py-3">
          <span className="font-mono text-lg font-bold tracking-tight">IMS</span>
          <span className="widget-label">Ground Station</span>
          <nav className="ml-4 flex gap-1">
            {VIEWS.map(({ id, label }) => (
              <button
                key={id}
                onClick={() => setView(id)}
                className={`rounded-md px-3 py-1 text-[12px] font-semibold ${
                  view === id ? 'bg-edge text-ink' : 'text-ink-3 hover:text-ink-2'
                }`}
              >
                {label}
              </button>
            ))}
          </nav>
          <div className="ml-auto">
            <HeaderStatus />
          </div>
        </div>
      </header>

      {/* Both stay mounted so switching views keeps capture history and form state. */}
      <div className={view === 'flight' ? 'contents' : 'hidden'}>
        <FlightView />
      </div>
      <div className={view === 'survey' ? 'contents' : 'hidden'}>
        <SurveyView active={view === 'survey'} />
      </div>
    </div>
  );
}
