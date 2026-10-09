import { defineConfig, type Connect, type Plugin } from 'vite'
import react, { reactCompilerPreset } from '@vitejs/plugin-react'
import babel from '@rolldown/plugin-babel'
import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const TILE_CACHE_DIR = fileURLToPath(new URL('./tile-cache', import.meta.url))
const TILE_UPSTREAM = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile'

/**
 * Serves /tiles/{z}/{x}/{y} from tile-cache/ (gitignored), fetching and saving
 * missing tiles from Esri World Imagery. Any area browsed once while online
 * keeps working offline at the field.
 */
function tileCache(): Plugin {
  const middleware: Connect.NextHandleFunction = async (req, res, next) => {
    const m = req.url?.match(/^\/tiles\/(\d+)\/(\d+)\/(\d+)$/)
    if (!m) return next()
    const [, z, x, y] = m
    const file = path.join(TILE_CACHE_DIR, z, x, `${y}.jpg`)
    try {
      let body: Buffer
      try {
        body = await fs.readFile(file)
      } catch {
        const upstream = await fetch(`${TILE_UPSTREAM}/${z}/${y}/${x}`) // Esri is z/y/x
        if (!upstream.ok) {
          res.statusCode = upstream.status
          return res.end()
        }
        body = Buffer.from(await upstream.arrayBuffer())
        await fs.mkdir(path.dirname(file), { recursive: true })
        await fs.writeFile(file, body)
      }
      res.setHeader('Content-Type', 'image/jpeg')
      res.setHeader('Cache-Control', 'public, max-age=31536000, immutable') // let the browser skip re-requests
      res.end(body)
    } catch {
      res.statusCode = 504 // offline and not cached
      res.end()
    }
  }
  return {
    name: 'tile-cache',
    configureServer: (server) => void server.middlewares.use(middleware),
    configurePreviewServer: (server) => void server.middlewares.use(middleware),
  }
}

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    react(),
    babel({ presets: [reactCompilerPreset()] }),
    tileCache(),
  ],
})
