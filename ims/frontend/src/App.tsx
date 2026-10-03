import { useEffect, useRef, useState } from 'react';
import GridLayout, { WidthProvider, type Layout } from 'react-grid-layout/legacy';
import HeaderStatus from './widgets/Header';
import WidgetRenderer from './WidgetRenderer';
import {
  availableWidgetTypes,
  DEFAULT_LAYOUT,
  GRID_COLUMNS,
  LAYOUT_STORAGE_KEY,
  MIN_WIDGET_SIZE,
  CUSTOM_RENDERERS,
  DATA_KINDS,
  DATA_FIELDS,
  layoutToJson,
  normalizeLayout,
  parseLayout,
  WIDGET_LABELS,
  type LayoutDocument,
  type LayoutItem,
  type WidgetType,
  type CustomWidgetDefinition,
  type CustomRenderer,
  type DataKind,
} from './layout';

const ROW_HEIGHT = 128;
const GRID_MARGIN: [number, number] = [16, 16];
const THEME_STORAGE_KEY = 'ims-theme';
const ResponsiveGridLayout = WidthProvider(GridLayout);

function loadTheme(): 'light' | 'dark' {
  return localStorage.getItem(THEME_STORAGE_KEY) === 'dark' ? 'dark' : 'light';
}

function loadLayout(): LayoutDocument {
  try {
    const saved = localStorage.getItem(LAYOUT_STORAGE_KEY);
    if (!saved) return DEFAULT_LAYOUT;
    const layout = parseLayout(saved);
    return layout.items.length ? layout : DEFAULT_LAYOUT;
  } catch {
    return DEFAULT_LAYOUT;
  }
}

function toGridLayout(document: LayoutDocument): Layout {
  return document.items.map(({ id, type, x, y, w, h }) => ({
    i: id,
    x,
    y,
    w,
    h,
    minW: type === 'custom' ? 3 : MIN_WIDGET_SIZE[type].w,
    minH: type === 'custom' ? 1 : Math.max(MIN_WIDGET_SIZE[type].h, type === 'script' ? 2 : MIN_WIDGET_SIZE[type].h),
    isBounded: true,
  }));
}

function fromGridLayout(grid: Layout, current: LayoutDocument): LayoutDocument {
  const positions = new Map(grid.map((item) => [item.i, item]));
  return normalizeLayout({
    ...current,
    items: current.items.map((item) => {
      const next = positions.get(item.id);
      return next ? { ...item, x: next.x, y: next.y, w: next.w, h: next.h } : item;
    }),
  });
}

function findItem(layout: LayoutDocument, id: string | null): LayoutItem | undefined {
  return id ? layout.items.find((item) => item.id === id) : undefined;
}

function itemLabel(item: LayoutItem, customWidget?: CustomWidgetDefinition): string {
  return item.type === 'custom' ? customWidget?.title ?? 'Custom widget' : WIDGET_LABELS[item.type];
}

function minimumFor(item: LayoutItem): { w: number; h: number } {
  return item.type === 'custom' ? { w: 3, h: 1 } : MIN_WIDGET_SIZE[item.type];
}

function CustomWidgetBuilder({
  onCancel,
  onCreate,
}: {
  onCancel: () => void;
  onCreate: (definition: Omit<CustomWidgetDefinition, 'id'>) => void;
}) {
  const [title, setTitle] = useState('Custom widget');
  const [renderer, setRenderer] = useState<CustomRenderer>('number');
  const [dataKind, setDataKind] = useState<DataKind>('attitude');
  const [field, setField] = useState('roll');
  const [decimals, setDecimals] = useState(2);
  return (
    <div className="ims-modal-backdrop" onClick={onCancel}>
      <form className="ims-modal" onSubmit={(event) => { event.preventDefault(); onCreate({ title, renderer, dataKind, field, decimals, label: title }); }} onClick={(event) => event.stopPropagation()}>
        <div className="flex items-center justify-between"><h2 className="ims-modal-title">Create custom widget</h2><button type="button" className="ims-button" onClick={onCancel}>Close</button></div>
        <label className="ims-form-field">Title<input value={title} onChange={(event) => setTitle(event.target.value)} /></label>
        <label className="ims-form-field">Renderer<select value={renderer} onChange={(event) => setRenderer(event.target.value as CustomRenderer)}>{CUSTOM_RENDERERS.map((option) => <option key={option}>{option}</option>)}</select></label>
        <label className="ims-form-field">IMS data type<select value={dataKind} onChange={(event) => { const next = event.target.value as DataKind; setDataKind(next); setField(DATA_FIELDS[next][0]); }}>{DATA_KINDS.map((option) => <option key={option}>{option}</option>)}</select></label>
        <label className="ims-form-field">Field<select value={field} onChange={(event) => setField(event.target.value)}>{DATA_FIELDS[dataKind].map((option) => <option key={option}>{option}</option>)}</select></label>
        {(renderer === 'number' || renderer === 'line' || renderer === 'bar' || renderer === 'pie') && <label className="ims-form-field">Decimal places<input type="number" min="0" max="6" value={decimals} onChange={(event) => setDecimals(Number(event.target.value))} /></label>}
        <p className="ims-toolbar-hint">The selected data type limits the compatible fields. Chart data uses a sample stream until the corresponding live source is wired.</p>
        <button type="submit" className="ims-button ims-button-primary">Create widget</button>
      </form>
    </div>
  );
}

export default function App() {
  const [layout, setLayout] = useState<LayoutDocument>(() => loadLayout());
  const [editing, setEditing] = useState(false);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [showGrid, setShowGrid] = useState(true);
  const [importError, setImportError] = useState('');
  const [showBuilder, setShowBuilder] = useState(false);
  const [gridWidth, setGridWidth] = useState(0);
  const [theme, setTheme] = useState<'light' | 'dark'>(() => loadTheme());
  const [isResizing, setIsResizing] = useState(false);
  const [pointerPosition, setPointerPosition] = useState<{ x: number; y: number } | null>(null);
  const importInput = useRef<HTMLInputElement>(null);
  const gridRef = useRef<HTMLDivElement>(null);
  const selected = findItem(layout, selectedId);
  const selectedCustom = selected?.type === 'custom'
    ? layout.customWidgets.find((widget) => widget.id === selected.customId)
    : undefined;

  useEffect(() => {
    localStorage.setItem(LAYOUT_STORAGE_KEY, layoutToJson(layout));
  }, [layout]);

  useEffect(() => {
    document.documentElement.dataset.theme = theme;
    localStorage.setItem(THEME_STORAGE_KEY, theme);
  }, [theme]);

  useEffect(() => {
    if (!gridRef.current) return;
    const observer = new ResizeObserver(() => {
      const grid = gridRef.current?.querySelector<HTMLElement>('.react-grid-layout');
      setGridWidth(grid?.getBoundingClientRect().width ?? 0);
    });
    observer.observe(gridRef.current);
    return () => observer.disconnect();
  }, []);

  const updateSelected = (update: Partial<Pick<LayoutItem, 'type' | 'x' | 'y' | 'w' | 'h' | 'config'>>) => {
    if (!selectedId) return;
    setLayout(
      normalizeLayout({
        ...layout,
        items: layout.items.map((item) => (item.id === selectedId ? { ...item, ...update } : item)),
      }),
    );
  };

  const addWidget = (type: WidgetType) => {
    const nextY = Math.max(0, ...layout.items.map((item) => item.y + item.h));
    const id = `${type}-${nextY}`;
    setLayout({
      ...layout,
      items: [...layout.items, { id, type, x: 0, y: nextY, w: Math.max(4, MIN_WIDGET_SIZE[type].w), h: Math.max(2, MIN_WIDGET_SIZE[type].h) }],
    });
    setSelectedId(id);
  };

  const createCustomWidget = (definition: Omit<CustomWidgetDefinition, 'id'>) => {
    const id = `custom-${Date.now()}`;
    const nextY = Math.max(0, ...layout.items.map((item) => item.y + item.h));
    setLayout({
      ...layout,
      customWidgets: [...layout.customWidgets, { ...definition, id }],
      items: [...layout.items, { id: `item-${id}`, type: 'custom', customId: id, x: 0, y: nextY, w: 4, h: 2 }],
    });
    setSelectedId(`item-${id}`);
    setShowBuilder(false);
  };

  const removeWidget = (id: string) => {
    const removed = layout.items.find((item) => item.id === id);
    setLayout({
      ...layout,
      items: layout.items.filter((item) => item.id !== id),
      customWidgets: removed?.customId
        ? layout.customWidgets.filter((widget) => widget.id !== removed.customId)
        : layout.customWidgets,
    });
    if (selectedId === id) setSelectedId(null);
  };

  const resetLayout = () => {
    setLayout(DEFAULT_LAYOUT);
    setSelectedId(null);
    setImportError('');
  };

  const exportLayout = () => {
    const blob = new Blob([layoutToJson(layout)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'ims-layout.json';
    link.click();
    URL.revokeObjectURL(url);
  };

  const importLayout = (file: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const imported = parseLayout(String(reader.result));
        if (!imported.items.length) throw new Error('The layout contains no known widgets.');
        setLayout(imported);
        setSelectedId(null);
        setImportError('');
      } catch (error) {
        setImportError(error instanceof Error ? error.message : 'Could not import layout.');
      }
    };
    reader.onerror = () => setImportError('Could not read layout file.');
    reader.readAsText(file);
  };

  return (
    <div className="ims-shell">
      <div className="ims-main">
        <header className="ims-header">
          <div className="ims-topbar">
            <div className="ims-topbar-identity">
              <div className="ims-brand-block">
                <span className="ims-brand-mark">I</span>
                <div><strong>IMS</strong><span>Ground Station</span></div>
              </div>
              <div className="ims-system-status">
                <span className="ims-status-dot" aria-hidden="true" />
                <div><strong>System status</strong><span>Awaiting connection</span></div>
              </div>
            </div>
            <div className="ims-breadcrumb"><span>Workspace</span><b>/</b><strong>Overview</strong></div>
            <div className="ims-topbar-actions">
              <HeaderStatus />
              <button
                type="button"
                className="ims-button ims-theme-toggle"
                aria-label={`Switch to ${theme === 'light' ? 'dark' : 'light'} mode`}
                onClick={() => setTheme((current) => current === 'light' ? 'dark' : 'light')}
              >
                <span className="ims-theme-icon" aria-hidden="true">
                  {theme === 'light' ? (
                    <svg viewBox="0 0 24 24" focusable="false"><path d="M12 3v2m0 14v2M5.64 5.64l1.42 1.42m9.9 9.9 1.4 1.4M3 12h2m14 0h2M5.64 18.36l1.42-1.42m9.9-9.9 1.4-1.4M12 8a4 4 0 1 0 0 8 4 4 0 0 0 0-8Z" /></svg>
                  ) : (
                    <svg viewBox="0 0 24 24" focusable="false"><path d="M20.5 15.2A8.5 8.5 0 0 1 8.8 3.5 8.5 8.5 0 1 0 20.5 15.2Z" /></svg>
                  )}
                </span>
                <span>{theme === 'light' ? 'Dark mode' : 'Light mode'}</span>
              </button>
            </div>
          </div>
        </header>

      <div className="ims-editor-bar">
        <div className="mx-auto flex min-h-14 w-full max-w-[1400px] flex-wrap items-center gap-2 px-5 py-2">
          <button
            type="button"
            className="ims-button ims-button-primary"
            onClick={() => {
              if (editing) setEditing(false);
              else setEditing(true);
            }}
          >
            {editing ? 'Finish editing' : selected ? 'Edit selected' : 'Edit layout'}
          </button>
          {editing && selected ? (
            <>
              <span className="ims-toolbar-title">{itemLabel(selected, selectedCustom)}</span>
              {selected.type === 'attitude' && (
                <details className="ims-config-popover">
                  <summary className="ims-button">Elements</summary>
                  <label><input type="checkbox" checked={selected.config?.visibleElements?.includes('horizon') !== false} onChange={(event) => updateSelected({ config: { visibleElements: event.target.checked ? ['horizon', 'readouts'] : ['readouts'] } })} /> Horizon graph</label>
                  <label><input type="checkbox" checked={selected.config?.visibleElements?.includes('readouts') !== false} onChange={(event) => updateSelected({ config: { visibleElements: event.target.checked ? ['horizon', 'readouts'] : ['horizon'] } })} /> Roll / Pitch / Yaw</label>
                </details>
              )}
              {selectedCustom && <span className="ims-toolbar-hint">Custom: {selectedCustom.renderer} · {selectedCustom.dataKind}.{selectedCustom.field}</span>}
              {selected.type !== 'custom' && <label className="ims-select-field">
                <span>Widget</span>
                <select value={selected.type} onChange={(event) => updateSelected({ type: event.target.value as WidgetType })}>
                  {Object.entries(WIDGET_LABELS).map(([type, label]) => <option key={type} value={type}>{label}</option>)}
                </select>
              </label>}
              {(['x', 'y', 'w', 'h'] as const).map((field) => (
                <label key={field} className="ims-number-field">
                  <span>{field.toUpperCase()}</span>
                  <input
                    type="number"
                    min={field === 'w' ? minimumFor(selected).w : field === 'h' ? minimumFor(selected).h : 0}
                    max={field === 'x' ? GRID_COLUMNS - selected.w : field === 'w' ? GRID_COLUMNS : 100}
                    value={selected[field]}
                    onChange={(event) => updateSelected({ [field]: Number(event.target.value) })}
                  />
                </label>
              ))}
              <button type="button" className="ims-button ims-button-danger" onClick={() => removeWidget(selected.id)}>Remove</button>
            </>
          ) : editing ? (
            <>
              <span className="ims-toolbar-hint">Select a widget to edit it</span>
              <button type="button" className="ims-button ims-button-primary" onClick={() => setShowBuilder(true)}>Create custom widget</button>
              {availableWidgetTypes(layout).map((type) => (
                <button key={type} type="button" className="ims-button" onClick={() => addWidget(type)}>+ {WIDGET_LABELS[type]}</button>
              ))}
              <button type="button" className="ims-button" onClick={() => setShowGrid(!showGrid)}>
                {showGrid ? 'Hide snap grid' : 'Show snap grid'}
              </button>
            </>
          ) : null}
          <span className="ims-toolbar-spacer" />
          <button type="button" className="ims-button" onClick={resetLayout}>Reset</button>
          <button type="button" className="ims-button" onClick={exportLayout}>Export</button>
          <button type="button" className="ims-button" onClick={() => importInput.current?.click()}>Import</button>
          <input ref={importInput} className="hidden" type="file" accept="application/json,.json" onChange={(event) => {
            const file = event.target.files?.[0];
            if (file) importLayout(file);
            event.target.value = '';
          }} />
          {importError && <span className="text-bad">{importError}</span>}
        </div>
      </div>

      <main className="ims-workspace" onClick={() => editing && setSelectedId(null)}>
        <div className="ims-page-heading">
          <div><p className="ims-eyebrow">Mission operations</p><h1>Dashboard</h1><p>Monitor your vehicle and mission telemetry in real time.</p></div>
          <div className="ims-heading-meta"><span className="ims-live-dot" />Live workspace</div>
        </div>
        <div
          ref={gridRef}
          className={`ims-grid-wrapper ${editing ? 'ims-editing' : ''} mx-auto w-full max-w-[1400px] px-5 py-5`}
          onPointerMove={(event) => {
            if (!isResizing || !gridRef.current) return;
            const grid = gridRef.current.querySelector<HTMLElement>('.react-grid-layout');
            const bounds = grid?.getBoundingClientRect();
            if (bounds) setPointerPosition({ x: event.clientX - bounds.left, y: event.clientY - bounds.top });
          }}
        >
          {editing && showGrid && isResizing && gridWidth > 0 && (() => {
            const columnWidth = (gridWidth - (GRID_COLUMNS - 1) * GRID_MARGIN[0]) / GRID_COLUMNS;
            const rows = Math.max(8, ...layout.items.map((item) => item.y + item.h + 2));
            const columnPositions = [...Array(GRID_COLUMNS + 1).keys()].map((column) => column * (columnWidth + GRID_MARGIN[0]) - GRID_MARGIN[0] / 2);
            const rowPositions = [...Array(rows + 1).keys()].map((row) => row * (ROW_HEIGHT + GRID_MARGIN[1]) - GRID_MARGIN[1] / 2);
            return (
              <div className="ims-snap-dots" aria-hidden="true">
                {rowPositions.flatMap((y, row) => columnPositions.map((x, column) => {
                  const distance = pointerPosition
                    ? Math.hypot(pointerPosition.x - x, pointerPosition.y - y)
                    : 280;
                  const opacity = Math.max(0.12, 1 - distance / 280);
                  return <span key={`${column}-${row}`} className="ims-snap-dot" style={{ left: x, top: y, opacity }} />;
                }))}
              </div>
            );
          })()}
          <ResponsiveGridLayout
            className={`ims-grid ${editing && showGrid ? 'ims-grid-snap' : ''}`}
            layout={toGridLayout(layout)}
            cols={GRID_COLUMNS}
            rowHeight={ROW_HEIGHT}
            margin={GRID_MARGIN}
            containerPadding={[0, 0]}
            isDraggable={editing}
            isResizable={editing}
            draggableHandle=".widget-drag-region"
            draggableCancel=".widget-close"
            resizeHandles={['se']}
            compactType="vertical"
            preventCollision={false}
            onLayoutChange={(next) => setLayout((current) => fromGridLayout(next, current))}
            onResizeStart={() => {
              setIsResizing(true);
              setPointerPosition(null);
            }}
            onResizeStop={() => {
              setIsResizing(false);
              setPointerPosition(null);
            }}
          >
            {layout.items.map((item) => (
              <div key={item.id} className={`ims-grid-card ims-widget-${item.type} ${selectedId === item.id ? 'ims-grid-card-selected' : ''}`} onClick={(event) => {
                event.stopPropagation();
                setSelectedId(item.id);
              }}>
                {editing && (
                  <>
                    <button type="button" aria-label={`Remove ${itemLabel(item, layout.customWidgets.find((widget) => widget.id === item.customId))}`} className="widget-close" onClick={() => removeWidget(item.id)} onMouseDown={() => setSelectedId(item.id)}>×</button>
                    <button type="button" aria-label={`Move ${itemLabel(item, layout.customWidgets.find((widget) => widget.id === item.customId))}`} className="widget-drag-region" onMouseDown={() => setSelectedId(item.id)} onClick={(event) => event.stopPropagation()}>
                      <span className="widget-drag-icon" aria-hidden="true"><i /><i /><i /><i /><i /><i /></span>
                    </button>
                  </>
                )}
                <div className="h-full min-h-0"><WidgetRenderer item={item} customWidget={layout.customWidgets.find((widget) => widget.id === item.customId)} /></div>
              </div>
            ))}
          </ResponsiveGridLayout>
        </div>
      </main>
      {showBuilder && <CustomWidgetBuilder onCancel={() => setShowBuilder(false)} onCreate={createCustomWidget} />}
      </div>
    </div>
  );
}
