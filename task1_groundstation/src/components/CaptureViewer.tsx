import type { CaptureStatus } from '../useTriggeredCapture';

type Props = {
  imageUrl: string | null;
  status: CaptureStatus;
  error: string | null;
  onSave?: () => void;
};

export function CaptureViewer({ imageUrl, status, error, onSave }: Props) {
  const failed = status === 'timeout' || status === 'failed';

  if (!imageUrl) {
    return (
      <div className={`flex items-center justify-center h-full text-sm ${failed ? 'text-red-500' : 'text-zinc-400'}`}>
        {failed ? `Capture failed — ${error}.` : 'No capture yet. Press Capture to request one.'}
      </div>
    );
  }

  return (
    <div className="flex items-center justify-center h-full min-h-0">
      <div className="relative flex items-center justify-center gap-3 h-full w-full">
        <img src={imageUrl} alt="Capture" className="max-h-full max-w-full object-contain rounded" />
        {failed && (
          <p className="absolute top-2 left-2 bg-white/90 text-red-500 text-xs px-2 py-1 rounded shadow">
            Capture failed — {error}.
          </p>
        )}
        {onSave && (
          <button
            onClick={onSave}
            className="absolute top-2 right-2 bg-zinc-900 text-white text-xs font-semibold px-3 py-1.5 rounded-md shadow"
          >
            + Save to History
          </button>
        )}
      </div>
    </div>
  );
}
