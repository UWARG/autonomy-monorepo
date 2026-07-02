import type React from 'react'

export interface WidgetProps {
  data: Record<string, unknown>
  send: (type: string, payload?: unknown) => void
}

export interface WidgetSettingsProps {
  onClose: () => void
}

export interface WidgetAction {
  id: string
  label: string
  handler: (send: (type: string, payload?: unknown) => void) => void
}

export interface WidgetDefinition {
  id: string
  label: string
  defaultLayout: { x: number; y: number; w: number; h: number; minW?: number; minH?: number }
  subscriptions?: string[]
  actions?: WidgetAction[]
  settings?: React.FC<WidgetSettingsProps>
  component: React.FC<WidgetProps>
}
