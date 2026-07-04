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

export default function App() {
  return (
    <div className="min-h-screen">
      {/* Header */}
      <header className="sticky top-0 z-20 border-b border-edge bg-card">
        <div className="mx-auto flex max-w-[1400px] items-center gap-4 px-5 py-3">
          <span className="font-mono text-lg font-bold tracking-tight">IMS</span>
          <span className="widget-label">Ground Station</span>
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
