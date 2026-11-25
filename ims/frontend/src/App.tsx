// Flight view grid (lg):
//   Map + traffic (8, all rows)   Mission (4)
//                                 Connection (4)
//                                 Log (4, takes the remaining height)
//
// Survey view grid (lg): Capture (7)   Survey report (5)

import { useState } from 'react';
import useAeacTraffic from './hooks/useAeacTraffic';
import useLink, { type LinkState } from './hooks/useLink';
import useMissionStatus, { type MissionStatusState } from './hooks/useMissionStatus';
import CaptureWidget from './widgets/CaptureWidget';
import ConnectionWidget from './widgets/ConnectionWidget';
import MissionWidget from './widgets/MissionWidget';
import SurveyWidget from './widgets/SurveyWidget';
import LogWidget from './widgets/LogWidget';
import MapWidget from './widgets/MapWidget';
import HeaderStatus from './widgets/Header';

type View = 'flight' | 'survey';

const VIEWS: { id: View; label: string }[] = [
  { id: 'flight', label: 'Flight' },
  { id: 'survey', label: 'Survey' },
];

function FlightView({ link, mission }: { link: LinkState; mission: MissionStatusState }) {
  const traffic = useAeacTraffic();

  return (
    <main
      className="mx-auto grid w-full max-w-[1400px] grid-cols-12 gap-4 px-5 py-4
                 lg:min-h-0 lg:flex-1 lg:overflow-y-auto
                 lg:[grid-template-rows:auto_auto_minmax(160px,1fr)]"
    >
      <div className="col-span-12 h-[640px] min-h-0 lg:col-span-8 lg:col-start-1 lg:row-span-3 lg:row-start-1 lg:h-auto">
        <MapWidget traffic={traffic} />
      </div>
      <div className="col-span-12 min-h-0 md:col-span-6 lg:col-span-4 lg:col-start-9 lg:row-start-1">
        <MissionWidget mission={mission} linked={link.rosbridge} />
      </div>
      <div className="col-span-12 min-h-0 md:col-span-6 lg:col-span-4 lg:col-start-9 lg:row-start-2">
        <ConnectionWidget link={link} />
      </div>
      <div className="col-span-12 h-[260px] min-h-0 lg:col-span-4 lg:col-start-9 lg:row-start-3 lg:h-auto">
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
  const link = useLink();
  const mission = useMissionStatus();

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
            <HeaderStatus link={link} mission={mission} />
          </div>
        </div>
      </header>

      {/* Both stay mounted so switching views keeps capture history and form state. */}
      <div className={view === 'flight' ? 'contents' : 'hidden'}>
        <FlightView link={link} mission={mission} />
      </div>
      <div className={view === 'survey' ? 'contents' : 'hidden'}>
        <SurveyView active={view === 'survey'} />
      </div>
    </div>
  );
}
