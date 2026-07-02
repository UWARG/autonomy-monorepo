import { useState, useCallback, useEffect, useRef } from 'react'
import { WIDGETS } from './widgetRegistry'
import { WidgetFrame } from './WidgetFrame'

const G = 24
const snap = (v: number) => Math.round(v / G) * G
const MIN_W = 216
const MIN_H = 150

interface Panel { id: string; x: number; y: number; w: number; h: number }

const DEFAULTS: Panel[] = [
  { id: 'camera',     x: 0,   y: 0,   w: 504, h: 480 },
  { id: 'map',        x: 504, y: 0,   w: 360, h: 312 },
  { id: 'attitude',   x: 864, y: 0,   w: 216, h: 312 },
  { id: 'connection', x: 504, y: 312, w: 360, h: 240 },
  { id: 'target',     x: 864, y: 312, w: 216, h: 240 },
  { id: 'script',     x: 0,   y: 480, w: 504, h: 240 },
  { id: 'log',        x: 504, y: 552, w: 576, h: 240 },
]

const LK = 'ims-layout-v2'
const VK = 'ims-visible'
const OK = 'ims-order'

const load = <T,>(key: string, fallback: T): T => {
  try { const s = localStorage.getItem(key); return s ? JSON.parse(s) : fallback } catch { return fallback }
}

export function Dashboard() {
  const [panels, setPanels]   = useState<Panel[]>(() => load(LK, DEFAULTS))
  const [visible, setVisible] = useState<string[]>(() => load(VK, WIDGETS.map(w => w.id)))
  const [order, setOrder]     = useState<string[]>(() => load(OK, WIDGETS.map(w => w.id)))
  const [showPicker, setShowPicker] = useState(false)

  const drag   = useRef<{ id: string; sx: number; sy: number; ox: number; oy: number } | null>(null)
  const resize = useRef<{ id: string; sx: number; sy: number; ow: number; oh: number } | null>(null)

  const patch = useCallback((id: string, p: Partial<Panel>) => {
    setPanels(prev => prev.map(panel => panel.id === id ? { ...panel, ...p } : panel))
  }, [])

  const front = useCallback((id: string) => {
    setOrder(prev => {
      const next = [...prev.filter(v => v !== id), id]
      localStorage.setItem(OK, JSON.stringify(next))
      return next
    })
  }, [])

  useEffect(() => {
    const onMove = (e: PointerEvent) => {
      const d = drag.current
      if (d) patch(d.id, {
        x: Math.max(0, snap(d.ox + e.clientX - d.sx)),
        y: Math.max(0, snap(d.oy + e.clientY - d.sy)),
      })
      const r = resize.current
      if (r) patch(r.id, {
        w: Math.max(MIN_W, snap(r.ow + e.clientX - r.sx)),
        h: Math.max(MIN_H, snap(r.oh + e.clientY - r.sy)),
      })
    }
    const onUp = () => {
      if (drag.current || resize.current) {
        setPanels(prev => { localStorage.setItem(LK, JSON.stringify(prev)); return prev })
      }
      drag.current = null
      resize.current = null
    }
    window.addEventListener('pointermove', onMove)
    window.addEventListener('pointerup', onUp)
    return () => { window.removeEventListener('pointermove', onMove); window.removeEventListener('pointerup', onUp) }
  }, [patch])

  useEffect(() => {
    const close = (e: KeyboardEvent) => { if (e.key === 'Escape') setShowPicker(false) }
    window.addEventListener('keydown', close)
    return () => window.removeEventListener('keydown', close)
  }, [])

  const removeWidget = useCallback((id: string) => {
    setVisible(prev => { const next = prev.filter(v => v !== id); localStorage.setItem(VK, JSON.stringify(next)); return next })
  }, [])

  const addWidget = useCallback((id: string) => {
    setVisible(prev => { const next = [...prev, id]; localStorage.setItem(VK, JSON.stringify(next)); return next })
    setShowPicker(false)
  }, [])

  const reset = useCallback(() => {
    const ids = WIDGETS.map(w => w.id)
    setPanels(DEFAULTS); setVisible(ids); setOrder(ids)
    localStorage.setItem(LK, JSON.stringify(DEFAULTS))
    localStorage.setItem(VK, JSON.stringify(ids))
    localStorage.setItem(OK, JSON.stringify(ids))
  }, [])

  const visibleWidgets = WIDGETS.filter(w => visible.includes(w.id))
  const hiddenWidgets  = WIDGETS.filter(w => !visible.includes(w.id))

  return (
    <div className="dashboard">
      <div className="dashboard-toolbar">
        <button className="toolbar-btn" onClick={() => setShowPicker(true)}>+ Add widget</button>
        <button className="toolbar-btn" onClick={reset}>Reset layout</button>
        <span className="live-banner">LIVE FLIGHT · DRAG TO ARRANGE</span>
      </div>

      {showPicker && (
        <div className="picker-overlay" onClick={() => setShowPicker(false)}>
          <div className="picker-modal" onClick={e => e.stopPropagation()}>
            <div className="picker-header">Add widget</div>
            {hiddenWidgets.length === 0
              ? <p className="picker-empty">All widgets are visible</p>
              : hiddenWidgets.map(w => (
                <button key={w.id} className="picker-item" onClick={() => addWidget(w.id)}>{w.label}</button>
              ))}
          </div>
        </div>
      )}

      <div className="canvas">
        {visibleWidgets.map(def => {
          const p = panels.find(l => l.id === def.id) ?? DEFAULTS.find(l => l.id === def.id) ?? { id: def.id, x: 0, y: 0, w: 288, h: 240 }
          const zIndex = order.indexOf(def.id) + 5
          return (
            <div
              key={def.id}
              className="panel"
              style={{ left: p.x, top: p.y, width: p.w, height: p.h, zIndex }}
              onPointerDown={() => front(def.id)}
            >
              <WidgetFrame
                definition={def}
                onClose={() => removeWidget(def.id)}
                onDragStart={(sx, sy) => { drag.current = { id: def.id, sx, sy, ox: p.x, oy: p.y } }}
              />
              <div
                className="resize-handle"
                onPointerDown={e => {
                  e.preventDefault()
                  e.stopPropagation()
                  resize.current = { id: def.id, sx: e.clientX, sy: e.clientY, ow: p.w, oh: p.h }
                  e.currentTarget.setPointerCapture(e.pointerId)
                  front(def.id)
                }}
              />
            </div>
          )
        })}
      </div>
    </div>
  )
}
