import { useCallback, useEffect, useState } from 'react';
import { CapturePanel } from './components/CapturePanel';
import { CaptureViewer } from './components/CaptureViewer';
import { CaptureHistoryList } from './components/CaptureHistoryList';
import { SurveyForm } from './components/SurveyForm';
import type { Capture } from './types';
import { useTriggeredCapture } from './useTriggeredCapture';

export default function App() {
  const [captures, setCaptures] = useState<Capture[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [hasSavedLive, setHasSavedLive] = useState(false);

  const { status, result, error, capture } = useTriggeredCapture();

  const handleCapture = useCallback(() => {
    setSelectedId(null);
    setHasSavedLive(false);
    capture();
  }, [capture]);

  useEffect(() => {
    function onKeyDown(e: KeyboardEvent) {
      const inInput = e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement;
      if (e.code === 'Space' && !inInput) {
        e.preventDefault();
        handleCapture();
      }
    }
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [handleCapture]);

  function handleDelete(id: string) {
    setCaptures((c) => c.filter((cap) => cap.id !== id));
    if (selectedId === id) setSelectedId(null);
  }

  function handleSaveToHistory() {
    if (!result) return;
    const created: Capture = {
      id: crypto.randomUUID(),
      time: new Date().toISOString(),
      imageUrl: result.imageUrl,
    };
    setCaptures((c) => [created, ...c]);
    setHasSavedLive(true);
  }

  const selectedCapture = selectedId ? captures.find((c) => c.id === selectedId) ?? null : null;
  const viewerImage = selectedCapture ? selectedCapture.imageUrl : (result?.imageUrl ?? null);
  const canSave = !selectedCapture && result !== null && !hasSavedLive;

  return (
    <main className="h-screen bg-white text-zinc-900 flex justify-center overflow-hidden">
      <div className="max-w-5xl w-full h-full px-4 py-3 flex flex-col gap-3 min-h-0">
        <div className="grid grid-cols-3 items-start shrink-0">
          <h1 className="text-sm font-semibold text-zinc-900 justify-self-start">Task 1 Ground Station — Herd Survey</h1>
          <div className="justify-self-center">
            <CapturePanel status={status} onCapture={handleCapture} />
          </div>
          <div />
        </div>

        <div className="flex-1 min-h-0">
          <CaptureViewer
            imageUrl={viewerImage}
            status={status}
            error={error}
            onSave={canSave ? handleSaveToHistory : undefined}
          />
        </div>

        <div className="grid grid-cols-[1fr_220px] gap-4 shrink-0 h-[55%] min-h-[260px]">
          <div className="h-full origin-bottom-left" style={{ transform: 'scale(0.9)', width: 'calc(100% / 0.9)' }}>
            <SurveyForm />
          </div>
          <div className="h-full origin-bottom-left" style={{ transform: 'scale(0.9)', width: 'calc(100% / 0.9)' }}>
            <CaptureHistoryList
              captures={captures}
              selectedId={selectedId}
              onSelect={setSelectedId}
              onDelete={handleDelete}
            />
          </div>
        </div>
      </div>
    </main>
  );
}
