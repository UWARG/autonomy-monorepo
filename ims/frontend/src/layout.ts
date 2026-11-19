export const LAYOUT_STORAGE_KEY = 'ims-layout-v2';
export const GRID_COLUMNS = 12;
export const MIN_WIDGET_SIZE: Record<WidgetType, { w: number; h: number }> = {
  camera: { w: 3, h: 2 },
  target: { w: 3, h: 2 },
  attitude: { w: 3, h: 2 },
  connection: { w: 3, h: 1 },
  script: { w: 4, h: 1 },
  log: { w: 4, h: 1 },
};

export type WidgetType =
  | 'camera'
  | 'target'
  | 'attitude'
  | 'connection'
  | 'script'
  | 'log';

export interface LayoutItem {
  id: string;
  type: WidgetType | 'custom';
  customId?: string;
  x: number;
  y: number;
  w: number;
  h: number;
  config?: WidgetConfig;
}

export type WidgetElement = 'horizon' | 'readouts';

export interface WidgetConfig {
  visibleElements?: WidgetElement[];
}

export type CustomRenderer = 'number' | 'line' | 'bar' | 'pie' | 'coordinates' | 'label' | 'button';
export type DataKind = 'attitude' | 'position' | 'connection' | 'status' | 'camera' | 'log';

export interface CustomWidgetDefinition {
  id: string;
  title: string;
  renderer: CustomRenderer;
  dataKind: DataKind;
  field: string;
  decimals?: number;
  label?: string;
}

export interface LayoutDocument {
  version: 3;
  items: LayoutItem[];
  customWidgets: CustomWidgetDefinition[];
}

export const WIDGET_LABELS: Record<WidgetType, string> = {
  camera: 'Camera',
  target: 'Position / Target',
  attitude: 'Attitude',
  connection: 'Connection',
  script: 'Mission Script',
  log: 'Telemetry Log',
};

export const CUSTOM_RENDERERS: CustomRenderer[] = ['number', 'line', 'bar', 'pie', 'coordinates', 'label', 'button'];
export const DATA_KINDS: DataKind[] = ['attitude', 'position', 'connection', 'status', 'camera', 'log'];
export const DATA_FIELDS: Record<DataKind, string[]> = {
  attitude: ['roll', 'pitch', 'yaw'],
  position: ['lat', 'lon', 'alt'],
  connection: ['heartbeatHz'],
  status: ['task', 'state', 'text'],
  camera: ['width', 'height', 'latencyMs'],
  log: ['message'],
};

export const DEFAULT_LAYOUT: LayoutDocument = {
  version: 3,
  customWidgets: [],
  items: [
    { id: 'camera', type: 'camera', x: 0, y: 0, w: 5, h: 2 },
    { id: 'target', type: 'target', x: 5, y: 0, w: 4, h: 2 },
    { id: 'attitude', type: 'attitude', x: 9, y: 0, w: 3, h: 2, config: { visibleElements: ['horizon', 'readouts'] } },
    { id: 'connection', type: 'connection', x: 0, y: 2, w: 4, h: 1 },
    { id: 'script', type: 'script', x: 4, y: 2, w: 8, h: 1 },
    { id: 'log', type: 'log', x: 0, y: 3, w: 12, h: 1 },
  ],
};

const WIDGET_TYPES = Object.keys(WIDGET_LABELS) as WidgetType[];

function isWidgetType(value: unknown): value is WidgetType {
  return typeof value === 'string' && WIDGET_TYPES.includes(value as WidgetType);
}

function boundedInteger(value: unknown, minimum: number, maximum: number, fallback: number): number {
  if (typeof value !== 'number' || !Number.isInteger(value)) return fallback;
  return Math.max(minimum, Math.min(maximum, value));
}

function normalizeItem(value: unknown, index: number): LayoutItem | null {
  if (!value || typeof value !== 'object') return null;
  const item = value as Partial<LayoutItem> & {
    column?: number;
    row?: number;
    columnSpan?: number;
    rowSpan?: number;
  };
  if (!isWidgetType(item.type) && item.type !== 'custom') return null;

  const x = boundedInteger(item.x ?? (item.column ? item.column - 1 : undefined), 0, GRID_COLUMNS - 1, 0);
  const maxWidth = GRID_COLUMNS - x;
  const minimum = item.type === 'custom' ? { w: 3, h: 1 } : MIN_WIDGET_SIZE[item.type];
  const visibleElements =
    item.config && Array.isArray(item.config.visibleElements)
      ? item.config.visibleElements.filter((element): element is WidgetElement => element === 'horizon' || element === 'readouts')
      : undefined;
  return {
    id: typeof item.id === 'string' && item.id ? item.id : `${item.type}-${index}`,
    type: item.type,
    customId: typeof item.customId === 'string' ? item.customId : undefined,
    x,
    y: boundedInteger(item.y ?? (item.row ? item.row - 1 : undefined), 0, 100, index),
    w: boundedInteger(item.w ?? item.columnSpan, Math.min(minimum.w, maxWidth), maxWidth, minimum.w),
    h: boundedInteger(item.h ?? item.rowSpan, minimum.h, 20, minimum.h),
    config: visibleElements ? { visibleElements } : undefined,
  };
}

export function normalizeLayout(value: unknown): LayoutDocument {
  const rawItems =
    value && typeof value === 'object' && Array.isArray((value as { items?: unknown }).items)
      ? (value as { items: unknown[] }).items
      : [];
  const seen = new Set<string>();
  const items = rawItems
    .map((item, index) => normalizeItem(item, index))
    .filter((item): item is LayoutItem => {
      if (!item || seen.has(item.id)) return false;
      seen.add(item.id);
      return true;
    });

  const customWidgets =
    value && typeof value === 'object' && Array.isArray((value as { customWidgets?: unknown }).customWidgets)
      ? (value as { customWidgets: unknown[] }).customWidgets.filter(isCustomWidget)
      : [];
  return { version: 3, items, customWidgets };
}

function isCustomWidget(value: unknown): value is CustomWidgetDefinition {
  if (!value || typeof value !== 'object') return false;
  const widget = value as Partial<CustomWidgetDefinition>;
  return (
    typeof widget.id === 'string' &&
    typeof widget.title === 'string' &&
    typeof widget.field === 'string' &&
    typeof widget.renderer === 'string' &&
    CUSTOM_RENDERERS.includes(widget.renderer as CustomRenderer) &&
    typeof widget.dataKind === 'string' &&
    DATA_KINDS.includes(widget.dataKind as DataKind) &&
    DATA_FIELDS[widget.dataKind as DataKind].includes(widget.field)
  );
}

export function parseLayout(text: string): LayoutDocument {
  return normalizeLayout(JSON.parse(text));
}

export function layoutToJson(layout: LayoutDocument): string {
  return JSON.stringify(layout, null, 2);
}

export function availableWidgetTypes(layout: LayoutDocument): WidgetType[] {
  const present = new Set(layout.items.map((item) => item.type));
  return WIDGET_TYPES.filter((type) => !present.has(type));
}
