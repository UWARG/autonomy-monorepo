import type { Capture } from '../types';

type Props = {
  captures: Capture[];
  selectedId: string | null;
  onSelect: (id: string) => void;
  onDelete: (id: string) => void;
};

export function CaptureHistoryList({ captures, selectedId, onSelect, onDelete }: Props) {
  return (
    <section className="flex flex-col gap-2 min-h-0 h-full">
      <h2 className="text-sm font-semibold text-zinc-900 shrink-0">
        History <span className="text-zinc-400 font-normal">({captures.length})</span>
      </h2>
      <div className="max-h-[350px] overflow-y-auto border border-zinc-200 rounded flex flex-col gap-1 p-1">
        {captures.length === 0 ? (
          <p className="text-xs text-zinc-400 p-1">No photos captured yet.</p>
        ) : (
          captures.map((c) => (
            <div
              key={c.id}
              onClick={() => onSelect(c.id)}
              className={`flex flex-col gap-0.5 p-1 rounded cursor-pointer ${
                selectedId === c.id ? 'bg-blue-50 ring-1 ring-blue-200' : 'hover:bg-zinc-50'
              }`}
            >
              <img src={c.imageUrl} alt="Capture" className="w-full h-14 object-cover rounded" />
              <div className="flex items-center justify-between">
                <span className="text-[10px] text-zinc-500">{new Date(c.time).toLocaleTimeString()}</span>
                <button
                  onClick={(e) => { e.stopPropagation(); onDelete(c.id); }}
                  className="w-4 h-4 rounded bg-red-50 hover:bg-red-100 flex items-center justify-center text-red-400 text-xs leading-none"
                  title="Delete"
                >
                  ×
                </button>
              </div>
            </div>
          ))
        )}
      </div>
    </section>
  );
}
