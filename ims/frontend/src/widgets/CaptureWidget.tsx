import { useCallback, useEffect, useState } from 'react';
import useTriggeredCapture from '../hooks/useTriggeredCapture';
import type { Capture } from '../types';

/**
 * Task 1 triggered capture: request a photo from airside (button or Space),
 * view it, and keep the ones worth counting in a session history.
 */
export default function CaptureWidget({ hotkeyEnabled }: { hotkeyEnabled: boolean }) {
  const { status, result, error, capture } = useTriggeredCapture();
  const [captures, setCaptures] = useState<Capture[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [savedLive, setSavedLive] = useState(false);

  const handleCapture = useCallback(() => {
    setSelectedId(null);
    setSavedLive(false);
    capture();
  }, [capture]);

  useEffect(() => {
    if (!hotkeyEnabled) return;
    function onKeyDown(e: KeyboardEvent) {
      const inInput = e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement;
      if (e.code === 'Space' && !inInput) {
        e.preventDefault();
        handleCapture();
      }
    }
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [handleCapture, hotkeyEnabled]);

  function saveLive() {
    if (!result) return;
    setCaptures((c) => [
      { id: crypto.randomUUID(), time: new Date().toISOString(), imageUrl: result.imageUrl },
      ...c,
    ]);
    setSavedLive(true);
  }

  function remove(id: string) {
    setCaptures((c) => c.filter((cap) => cap.id !== id));
    if (selectedId === id) setSelectedId(null);
  }

  const selected = selectedId ? (captures.find((c) => c.id === selectedId) ?? null) : null;
  const imageUrl = selected ? selected.imageUrl : (result?.imageUrl ?? null);
  const canSave = !selected && result !== null && !savedLive;
  const failed = status === 'timeout' || status === 'failed';

  const pill =
    status === 'capturing'
      ? { className: 'pill-warn', label: 'CAPTURING' }
      : failed
        ? { className: 'pill-bad', label: 'FAILED' }
        : selected
          ? { className: 'pill-accent', label: 'HISTORY' }
          : result
            ? { className: 'pill-ok', label: 'LIVE' }
            : { className: 'pill bg-edge text-ink-3', label: 'IDLE' };

  return (
    <section className="widget flex h-full min-h-[120px] flex-col p-4">
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">Capture</h2>
        <div className="flex items-center gap-3">
          <span className="text-[11px] text-ink-3">
            <kbd className="rounded border border-edge px-1 font-mono">Space</kbd>
          </span>
          <button
            onClick={handleCapture}
            disabled={status === 'capturing'}
            className="rounded-md bg-accent px-4 py-1.5 font-mono text-[12px] font-semibold tracking-widest text-app disabled:cursor-wait disabled:opacity-60"
          >
            CAPTURE
          </button>
          <span className={pill.className}>{pill.label}</span>
        </div>
      </header>

      <div className="mt-3 flex min-h-0 flex-1 gap-3">
        <div className="relative flex min-h-0 flex-1 items-center justify-center rounded-lg bg-feed">
          {imageUrl ? (
            <img src={imageUrl} alt="Capture" className="max-h-full max-w-full object-contain" />
          ) : (
            <p className={`text-[13px] ${failed ? 'text-bad' : 'text-ink-3'}`}>
              {failed ? `Capture failed: ${error}` : 'No capture yet'}
            </p>
          )}
          {imageUrl && failed && (
            <p className="absolute left-2 top-2 rounded bg-bad-dim px-2 py-1 text-[12px] text-bad">
              Capture failed: {error}
            </p>
          )}
          {canSave && (
            <button
              onClick={saveLive}
              className="absolute right-2 top-2 rounded-md bg-card px-3 py-1.5 text-[12px] font-semibold text-ink"
            >
              + Save to history
            </button>
          )}
        </div>

        <div className="flex w-[160px] shrink-0 flex-col">
          <span className="widget-label">History ({captures.length})</span>
          <div className="mt-2 flex min-h-0 flex-1 flex-col gap-1 overflow-y-auto">
            {captures.length === 0 && <p className="text-[12px] text-ink-3">Nothing saved yet</p>}
            {captures.map((c) => (
              <div
                key={c.id}
                onClick={() => setSelectedId(c.id)}
                className={`cursor-pointer rounded p-1 ${
                  selectedId === c.id ? 'bg-accent-dim ring-1 ring-accent' : 'hover:bg-inset'
                }`}
              >
                <img src={c.imageUrl} alt="Capture" className="h-16 w-full rounded object-cover" />
                <div className="mt-0.5 flex items-center justify-between">
                  <span className="font-mono text-[10px] text-ink-3">
                    {new Date(c.time).toLocaleTimeString()}
                  </span>
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      remove(c.id);
                    }}
                    className="px-1 text-[12px] leading-none text-bad"
                    title="Delete"
                  >
                    ×
                  </button>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}
