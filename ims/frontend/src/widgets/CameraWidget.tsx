import type { WidgetDefinition, WidgetProps } from '../types'

function CameraView({ send }: WidgetProps) {
  return (
    <div className="camera-widget">
      <div className="camera-topbar">
        <span className="camera-badge"><span className="dot dot-red" />PHOTO MODE</span>
        <span className="camera-no-video">NO VIDEO DOWNLINK</span>
      </div>

      <div className="camera-preview">
        <span className="camera-placeholder-text">PRESS SHUTTER TO CAPTURE</span>
      </div>

      <div className="camera-footer-bar">
        <span className="camera-status-text">awaiting first capture</span>
        <span className="camera-exif">f/2.8  1/500  ISO100</span>
      </div>

      <div className="camera-no-photos">No photos yet — capture stores stills locally</div>

      <div className="camera-controls">
        <button className="shutter-btn" onClick={() => send('camera_capture')}>
          <span className="shutter-circle" />
        </button>
        <div className="camera-control-right">
          <span className="camera-meta-pill">AUTO OFF</span>
          <span className="camera-meta-pill">Interval 5s</span>
        </div>
        <div className="camera-shots">
          <span>0 shots</span>
          <span>0.0 MB</span>
        </div>
      </div>
    </div>
  )
}

export const CameraWidget: WidgetDefinition = {
  id: 'camera',
  label: 'Camera (Photo)',
  defaultLayout: { x: 0, y: 0, w: 6, h: 9, minW: 3, minH: 4 },
  subscriptions: ['camera'],
  actions: [
    { id: 'capture', label: 'Capture', handler: send => send('camera_capture') },
  ],
  component: CameraView,
}
