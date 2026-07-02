import { useState } from 'react'
import type { WidgetDefinition } from './types'
import { useWidgetData } from './useWidgetData'
import { send } from './socket'

interface WidgetFrameProps {
  definition: WidgetDefinition
  onClose: () => void
  onDragStart?: (sx: number, sy: number) => void
}

export function WidgetFrame({ definition, onClose, onDragStart }: WidgetFrameProps) {
  const [showSettings, setShowSettings] = useState(false)
  const data = useWidgetData(definition.subscriptions ?? [])
  const Settings = definition.settings

  return (
    <div className="widget-frame">
      <div
        className="widget-header"
        onPointerDown={e => {
          if ((e.target as HTMLElement).closest('button')) return
          e.preventDefault()
          onDragStart?.(e.clientX, e.clientY)
        }}
      >
        <div className="widget-header-left">
          <span className="drag-dots">⠿</span>
          <span className="widget-title">{definition.label}</span>
        </div>
        <div className="widget-header-right">
          {definition.actions?.map(action => (
            <button
              key={action.id}
              className="widget-action-btn"
              onMouseDown={e => e.stopPropagation()}
              onClick={() => action.handler(send)}
            >
              {action.label}
            </button>
          ))}
          {Settings && (
            <button
              className="widget-icon-btn"
              title="Settings"
              onMouseDown={e => e.stopPropagation()}
              onClick={() => setShowSettings(s => !s)}
            >
              ◉
            </button>
          )}
          <button
            className="widget-icon-btn"
            title="Close"
            onMouseDown={e => e.stopPropagation()}
            onClick={onClose}
          >
            ✕
          </button>
        </div>
      </div>

      {showSettings && Settings && (
        <div className="widget-settings-panel">
          <Settings onClose={() => setShowSettings(false)} />
        </div>
      )}

      <div className="widget-body">
        <definition.component data={data} send={send} />
      </div>
    </div>
  )
}
