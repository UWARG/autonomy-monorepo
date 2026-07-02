import { useRef, useEffect } from 'react'
import type { WidgetDefinition, WidgetProps } from '../types'

interface Point { lat: number; lon: number }

function MapView({ data }: WidgetProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const position = (data.position ?? {}) as Record<string, number>
  const target = (data.target ?? {}) as Record<string, number>

  const droneLat = position.lat ?? 37.44248
  const droneLon = position.lon ?? -122.1430
  const tgtLat = target.lat ?? 37.44339
  const tgtLon = target.lon ?? -122.14141

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const W = canvas.width
    const H = canvas.height
    ctx.clearRect(0, 0, W, H)

    const latRange = 0.003
    const lonRange = 0.003

    const toCanvas = (p: Point) => ({
      x: ((p.lon - (droneLon - lonRange / 2)) / lonRange) * W,
      y: H - ((p.lat - (droneLat - latRange / 2)) / latRange) * H,
    })

    // Grid
    ctx.strokeStyle = 'rgba(255,255,255,0.06)'
    ctx.lineWidth = 1
    for (let i = 0; i <= 4; i++) {
      ctx.beginPath(); ctx.moveTo((W / 4) * i, 0); ctx.lineTo((W / 4) * i, H); ctx.stroke()
      ctx.beginPath(); ctx.moveTo(0, (H / 4) * i); ctx.lineTo(W, (H / 4) * i); ctx.stroke()
    }

    // Trail (placeholder)
    const dronePos = toCanvas({ lat: droneLat, lon: droneLon })
    const tgtPos = toCanvas({ lat: tgtLat, lon: tgtLon })

    ctx.strokeStyle = 'rgba(59, 130, 246, 0.5)'
    ctx.lineWidth = 1.5
    ctx.setLineDash([4, 4])
    ctx.beginPath()
    ctx.moveTo(dronePos.x - 40, dronePos.y)
    ctx.lineTo(dronePos.x, dronePos.y)
    ctx.stroke()
    ctx.setLineDash([])

    // Target dot
    ctx.fillStyle = '#22c55e'
    ctx.beginPath()
    ctx.arc(tgtPos.x, tgtPos.y, 5, 0, Math.PI * 2)
    ctx.fill()

    // Drone arrow
    ctx.save()
    ctx.translate(dronePos.x, dronePos.y)
    ctx.fillStyle = '#f59e0b'
    ctx.beginPath()
    ctx.moveTo(0, -10)
    ctx.lineTo(6, 8)
    ctx.lineTo(0, 4)
    ctx.lineTo(-6, 8)
    ctx.closePath()
    ctx.fill()
    ctx.restore()

    // N indicator
    ctx.fillStyle = 'rgba(255,255,255,0.7)'
    ctx.font = '11px monospace'
    ctx.fillText('N', W - 18, 18)
    ctx.fillStyle = '#ef4444'
    ctx.fillText('▲', W - 16, 28)
  }, [droneLat, droneLon, tgtLat, tgtLon])

  return (
    <div className="map-widget">
      <canvas ref={canvasRef} className="map-canvas" width={400} height={260} />
      <div className="map-coords">
        <span>N {droneLat.toFixed(5)}°</span>
        <span>W {Math.abs(droneLon).toFixed(5)}°</span>
      </div>
      <div className="map-legend">
        <span><span className="dot dot-orange" /> Drone</span>
        <span><span className="dot dot-green" /> Target</span>
        <span className="legend-line" /> Trail
      </div>
    </div>
  )
}

export const MapWidget: WidgetDefinition = {
  id: 'map',
  label: 'Map Tracker',
  defaultLayout: { x: 6, y: 0, w: 4, h: 5, minW: 2, minH: 3 },
  subscriptions: ['position', 'target'],
  component: MapView,
}
