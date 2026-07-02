import type { WidgetDefinition } from './types'
import { CameraWidget } from './widgets/CameraWidget'
import { MapWidget } from './widgets/MapWidget'
import { AttitudeWidget } from './widgets/AttitudeWidget'
import { ConnectionWidget } from './widgets/ConnectionWidget'
import { TargetWidget } from './widgets/TargetWidget'
import { ScriptWidget } from './widgets/ScriptWidget'
import { LogWidget } from './widgets/LogWidget'

export const WIDGETS: WidgetDefinition[] = [
  CameraWidget,
  MapWidget,
  AttitudeWidget,
  ConnectionWidget,
  TargetWidget,
  ScriptWidget,
  LogWidget,
]
