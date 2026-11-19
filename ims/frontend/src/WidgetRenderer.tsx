import AttitudeWidget from './widgets/AttitudeWidget';
import CameraWidget from './widgets/CameraWidget';
import ConnectionWidget from './widgets/ConnectionWidget';
import LogWidget from './widgets/LogWidget';
import ScriptWidget from './widgets/ScriptWidget';
import TargetWidget from './widgets/TargetWidget';
import type { WidgetType } from './layout';
import CustomWidget from './widgets/CustomWidget';
import type { CustomWidgetDefinition, LayoutItem } from './layout';

const WIDGETS: Record<WidgetType, (config?: LayoutItem['config']) => ReactElement> = {
  camera: () => <CameraWidget />,
  target: () => <TargetWidget />,
  attitude: (config) => <AttitudeWidget config={config} />,
  connection: () => <ConnectionWidget />,
  script: () => <ScriptWidget />,
  log: () => <LogWidget />,
};

export default function WidgetRenderer({ item, customWidget }: { item: LayoutItem; customWidget?: CustomWidgetDefinition }) {
  if (item.type === 'custom' && customWidget) return <CustomWidget definition={customWidget} />;
  const Widget = WIDGETS[item.type as WidgetType];
  return Widget(item.config);
}
import type { ReactElement } from 'react';
