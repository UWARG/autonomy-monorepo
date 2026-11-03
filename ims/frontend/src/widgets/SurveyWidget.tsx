import { useState } from 'react';
import { buildSurveyText, SURVEY_FILENAME } from '../surveyReport';

function downloadText(filename: string, text: string) {
  const url = URL.createObjectURL(new Blob([text], { type: 'text/plain' }));
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

/** Chips plus a draft input: Enter adds the draft, Backspace on an empty draft removes the last chip. */
function ChipInput<T extends string | number>({
  label,
  items,
  setItems,
  draft,
  setDraft,
  parse,
  placeholder,
  mono = false,
}: {
  label: string;
  items: T[];
  setItems: (update: (items: T[]) => T[]) => void;
  draft: string;
  setDraft: (draft: string) => void;
  parse: (draft: string) => T | null;
  placeholder: string;
  mono?: boolean;
}) {
  function onKeyDown(e: React.KeyboardEvent<HTMLInputElement>) {
    if (e.key === 'Enter') {
      e.preventDefault();
      const item = parse(draft);
      if (item !== null) {
        setItems((all) => [...all, item]);
        setDraft('');
      }
    } else if (e.key === 'Backspace' && draft === '' && items.length > 0) {
      setItems((all) => all.slice(0, -1));
    }
  }

  return (
    <div className="flex flex-col gap-1.5">
      <span className="text-[12px] text-ink-3">{label}</span>
      <div className="flex flex-wrap items-center gap-1.5">
        {items.map((item, i) => (
          <span key={i} className="pill bg-edge text-ink">
            {item}
            <button onClick={() => setItems((all) => all.filter((_, j) => j !== i))} className="text-bad">
              ×
            </button>
          </span>
        ))}
        <input
          type="text"
          value={draft}
          onChange={(e) => setDraft(e.target.value)}
          onKeyDown={onKeyDown}
          placeholder={placeholder}
          className={`w-32 rounded border border-edge bg-inset px-2 py-1 text-[13px] text-ink outline-none focus:border-ink-3 ${mono ? 'font-mono' : ''}`}
        />
      </div>
    </div>
  );
}

/** Task 1 survey report (CONOPS §5.2.3): clusters, tag IDs and an optional anomaly note. */
export default function SurveyWidget() {
  const [clusters, setClusters] = useState<number[]>([]);
  const [tags, setTags] = useState<string[]>([]);
  const [description, setDescription] = useState('');
  const [clusterDraft, setClusterDraft] = useState('');
  const [tagDraft, setTagDraft] = useState('');

  const text = buildSurveyText(clusters, tags, description);

  return (
    <section className="widget flex h-full min-h-[120px] flex-col gap-4 overflow-y-auto p-4">
      <h2 className="widget-label">Survey Report</h2>

      <ChipInput
        label="Clusters: count + Enter (10 m radius; a lone deer is a cluster of one)"
        items={clusters}
        setItems={setClusters}
        draft={clusterDraft}
        setDraft={(d) => setClusterDraft(d.replace(/[^0-9]/g, ''))}
        parse={(d) => {
          const n = parseInt(d, 10);
          return Number.isFinite(n) && n > 0 ? n : null;
        }}
        placeholder="count + Enter"
      />

      <ChipInput
        label="Tag IDs: code + Enter (2-character alphanumeric, e.g. H7)"
        items={tags}
        setItems={setTags}
        draft={tagDraft}
        setDraft={(d) => setTagDraft(d.toUpperCase().slice(0, 2))}
        parse={(d) => d.trim().toUpperCase() || null}
        placeholder="H7 + Enter"
        mono
      />

      <label className="flex flex-col gap-1.5 text-[12px] text-ink-3">
        Anomalous deer description (optional)
        <textarea
          value={description}
          onChange={(e) => setDescription(e.target.value)}
          placeholder='e.g. "One deer is wearing a hat."'
          className="rounded border border-edge bg-inset px-3 py-2 text-[13px] text-ink outline-none focus:border-ink-3"
          rows={3}
        />
      </label>

      <p className="rounded border border-edge bg-inset p-3 text-[13px] text-ink">
        <span className="text-ink-3">Preview: </span>
        {text}
      </p>

      <button
        onClick={() => downloadText(SURVEY_FILENAME, text)}
        className="self-start rounded-md bg-accent px-4 py-1.5 text-[12px] font-semibold text-app"
      >
        Download {SURVEY_FILENAME}
      </button>
    </section>
  );
}
