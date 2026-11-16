import express from 'express'
import cors from 'cors'
import path from 'path'
import fs from 'fs'
import https from 'https'
import { fileURLToPath } from 'url'

const PORT = process.env.PORT || 3001
const isProd = process.env.NODE_ENV === 'production'
// Optional: serve https (browsers block ws:// from an https page, see services/rosbridge.js)
const TLS_CERT = process.env.GOAWAY_TLS_CERT
const TLS_KEY = process.env.GOAWAY_TLS_KEY

const __dirname = path.dirname(fileURLToPath(import.meta.url))

const app = express()

// Vite's dev server proxies /api to this process from a different origin,
// so CORS is only needed in dev. In prod this same process serves the UI
// too, making everything same-origin.
if (!isProd) {
  app.use(cors({ origin: 'http://localhost:5173' }))
}
app.use(express.json())

// The header's api ping (hooks/usePing.js)
app.get('/api/health', (_req, res) => {
  res.json({ status: 'ok', timestamp: Date.now() })
})

// System reboot: thin proxy to reboot_manager.py on the host (app/goaway_ws), which can run
// supervisorctl and docker; this container can't. The manager binds loopback only, reachable
// here through host networking, so the sudo path stays off the network.
const REBOOT_MANAGER_URL = process.env.GOAWAY_REBOOT_URL || 'http://127.0.0.1:5055'
const REBOOT_TARGETS = new Set(['rosbridge', 'fastlio', 'goaway_ws'])

// Upstream status codes pass straight through: 409 (a reboot is already running) reads
// differently from a failure.
async function proxyToRebootManager(res, urlPath, init) {
  try {
    const upstream = await fetch(`${REBOOT_MANAGER_URL}${urlPath}`, init)
    res.status(upstream.status).json(await upstream.json())
  } catch (err) {
    // The manager is its own supervisor program and may be stopped or not deployed yet.
    console.error(`[ui_backend] reboot manager ${urlPath} failed:`, err.message)
    res.status(503).json({ error: 'reboot manager unreachable' })
  }
}

app.get('/api/system/status', (_req, res) =>
  proxyToRebootManager(res, '/status'))

app.post('/api/system/reboot/:target', (req, res) => {
  // Allowlisted here too, so an unknown target never reaches the host service.
  const { target } = req.params
  if (!REBOOT_TARGETS.has(target)) {
    return res.status(400).json({ error: `unknown reboot target: ${target}` })
  }
  proxyToRebootManager(res, `/reboot/${target}`, { method: 'POST' })
})

if (isProd) {
  app.use(express.static(path.join(__dirname, 'dist')))
}

if (TLS_CERT && TLS_KEY) {
  https.createServer({
    cert: fs.readFileSync(TLS_CERT),
    key: fs.readFileSync(TLS_KEY),
  }, app).listen(PORT, () => {
    console.log(`[ui_backend] https listening on ${PORT}`)
  })
} else {
  app.listen(PORT, () => {
    console.log(`[ui_backend] listening on http://localhost:${PORT}`)
  })
}
