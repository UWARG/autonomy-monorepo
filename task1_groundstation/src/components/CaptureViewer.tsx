import type { CaptureStatus } from '../useTriggeredCapture';

type Props = {
  forwardImageUrl: string | null;
  downwardImageUrl: string | null;
  status: CaptureStatus;
  onSave?: () => void;
};

export function CaptureViewer({ forwardImageUrl, downwardImageUrl, status, onSave }: Props) {
  if (!forwardImageUrl || !downwardImageUrl) {
    return (
      <div className="flex items-center justify-center h-full text-sm text-zinc-400">
        No capture yet. Press Capture to request one.
      </div>
    );
  }

  return (
    <div className="flex items-center justify-center h-full min-h-0">
      <div className="relative flex items-center justify-center gap-3 h-full w-full">
        <img src={forwardImageUrl} alt="Forward" className="max-h-full max-w-[48%] object-contain rounded" />
        <img src={downwardImageUrl} alt="Downward" className="max-h-full max-w-[48%] object-contain rounded" />
        {status === 'timeout' && (
          <p className="absolute top-2 left-2 bg-white/90 text-red-500 text-xs px-2 py-1 rounded shadow">
            Timed out — no response from the drone.
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
