import type { CaptureStatus } from '../useTriggeredCapture';

type Props = {
  status: CaptureStatus;
  onCapture: () => void;
};

export function CapturePanel({ status, onCapture }: Props) {
  return (
    <div className="flex flex-col items-center gap-2 shrink-0">
      <button
        onClick={onCapture}
        disabled={status === 'capturing'}
        className="bg-zinc-900 text-white px-8 py-3 rounded-md text-sm font-semibold tracking-widest disabled:opacity-60 disabled:cursor-wait"
      >
        {status === 'capturing' ? 'CAPTURING...' : 'CAPTURE'}
      </button>
      <p className="text-xs text-zinc-400">
        or press{' '}
        <kbd className="px-1.5 py-0.5 bg-zinc-100 rounded border border-zinc-300 text-zinc-600 font-mono text-xs">
          Space
        </kbd>
      </p>
    </div>
  );
}
