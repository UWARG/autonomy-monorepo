import { useState } from 'react';
import { buildSurveyText, SURVEY_FILENAME } from '../surveyReport';

function downloadText(filename: string, text: string) {
  const blob = new Blob([text], { type: 'text/plain' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

export function SurveyForm() {
  const [clusters, setClusters] = useState<number[]>([]);
  const [tags, setTags] = useState<string[]>([]);
  const [description, setDescription] = useState('');
  const [clusterDraft, setClusterDraft] = useState('');
  const [tagDraft, setTagDraft] = useState('');

  const text = buildSurveyText(clusters, tags, description);

  function removeCluster(index: number) {
    setClusters((c) => c.filter((_, i) => i !== index));
  }
  function removeTag(index: number) {
    setTags((t) => t.filter((_, i) => i !== index));
  }

  function handleClusterKeyDown(e: React.KeyboardEvent<HTMLInputElement>) {
    if (e.key === 'Enter') {
      e.preventDefault();
      const n = parseInt(clusterDraft, 10);
      if (Number.isFinite(n) && n > 0) {
        setClusters((c) => [...c, n]);
        setClusterDraft('');
      }
    } else if (e.key === 'Backspace' && clusterDraft === '' && clusters.length > 0) {
      setClusters((c) => c.slice(0, -1));
    }
  }

  function handleTagKeyDown(e: React.KeyboardEvent<HTMLInputElement>) {
    if (e.key === 'Enter') {
      e.preventDefault();
      const code = tagDraft.trim().toUpperCase();
      if (code) {
        setTags((t) => [...t, code]);
        setTagDraft('');
      }
    } else if (e.key === 'Backspace' && tagDraft === '' && tags.length > 0) {
      setTags((t) => t.slice(0, -1));
    }
  }

  return (
    <section className="w-full h-full flex flex-col gap-4">
      <h2 className="text-base font-semibold text-zinc-900">Survey Report</h2>

      <div className="flex flex-col gap-2">
        <span className="text-sm font-medium text-zinc-600">
          Clusters — count + Enter (10m radius; a lone deer is a cluster of one)
        </span>
        <div className="flex flex-wrap items-center gap-2">
          {clusters.map((count, i) => (
            <span key={i} className="flex items-center gap-1 border border-zinc-200 rounded px-2 py-1 text-sm">
              {count}
              <button onClick={() => removeCluster(i)} className="text-red-400 px-1">×</button>
            </span>
          ))}
          <input
            type="text"
            inputMode="numeric"
            value={clusterDraft}
            onChange={(e) => setClusterDraft(e.target.value.replace(/[^0-9]/g, ''))}
            onKeyDown={handleClusterKeyDown}
            placeholder="count + Enter"
            className="w-32 border border-zinc-200 rounded px-2 py-1 text-sm outline-none focus:border-zinc-400"
          />
        </div>
      </div>

      <div className="flex flex-col gap-2">
        <span className="text-sm font-medium text-zinc-600">
          Tag IDs — code + Enter (2-character alphanumeric, e.g. H7)
        </span>
        <div className="flex flex-wrap items-center gap-2">
          {tags.map((code, i) => (
            <span key={i} className="flex items-center gap-1 border border-zinc-200 rounded px-2 py-1 text-sm font-mono">
              {code}
              <button onClick={() => removeTag(i)} className="text-red-400 px-1">×</button>
            </span>
          ))}
          <input
            type="text"
            value={tagDraft}
            onChange={(e) => setTagDraft(e.target.value.toUpperCase().slice(0, 2))}
            onKeyDown={handleTagKeyDown}
            placeholder="H7 + Enter"
            className="w-28 border border-zinc-200 rounded px-2 py-1 text-sm font-mono outline-none focus:border-zinc-400"
          />
        </div>
      </div>

      <label className="flex flex-col gap-2 text-sm font-medium text-zinc-600">
        Anomalous deer description (optional)
        <textarea
          value={description}
          onChange={(e) => setDescription(e.target.value)}
          placeholder='e.g. "One deer is wearing a hat."'
          className="border border-zinc-200 rounded px-3 py-2 text-sm text-zinc-900 outline-none focus:border-zinc-400"
          rows={3}
        />
      </label>

      <div className="rounded p-3 text-sm bg-zinc-50 border border-zinc-100">
        <span className="font-medium text-zinc-700">Preview: </span>
        <span className="text-zinc-900">{text}</span>
      </div>

      <button
        onClick={() => downloadText(SURVEY_FILENAME, text)}
        className="self-start bg-zinc-900 text-white px-6 py-2 rounded-md text-sm font-semibold"
      >
        Download {SURVEY_FILENAME}
      </button>
    </section>
  );
}
