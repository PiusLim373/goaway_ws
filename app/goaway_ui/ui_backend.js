import express from 'express'
import cors from 'cors'
import path from 'path'
import fs from 'fs'
import os from 'os'
import https from 'https'
import archiver from 'archiver'
import multer from 'multer'
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

// ---------------------------------------------------------------------------
// 2D maps, tmms's /api/maps2d (the C2 reads the same routes and store)
//
//   <MAPS_DIR>/png/<name>.png + .yaml    what map_server / AMCL / C2 load; the editor writes here
//   <MAPS_DIR>/slam/<name>.*             mapping_manager's untouched copy + slam_toolbox pose graph

const MAPS_DIR = process.env.GOAWAY_MAPS_DIR || path.join(os.homedir(), '.htxgrrt', 'maps')
const PNG_DIR = path.join(MAPS_DIR, 'png')
const SLAM_DIR = path.join(MAPS_DIR, 'slam')
// Not created here: an unmounted store (goaway_ui compose) must not look like an empty one.
console.log(`[ui_backend] map store ${MAPS_DIR}`
  + (fs.existsSync(MAPS_DIR) ? '' : ' NOT FOUND: mount it (goaway_ui-compose.yaml)'))

// Bare names only: this regex is the whole path-traversal defence (extensions added here).
const MAP_NAME_RE = /^[A-Za-z0-9_]+$/
const SLAM_EXTS = ['png', 'yaml', 'posegraph', 'data']

// map_server: occ = (255 - pixel) / 255, so 0 -> occupied, 205 -> unknown, 255 -> free.
// free_thresh 0.15 rather than 0.196 so 205 is not a float-rounding call (as tmms).
const MAP_FREE_THRESH = 0.15
const MAP_OCCUPIED_THRESH = 0.65

const mapsUpload = multer({
  storage: multer.memoryStorage(),
  limits: { fileSize: 256 * 1024 * 1024 },
})

const png2dPath = (name) => path.join(PNG_DIR, `${name}.png`)
const yaml2dPath = (name) => path.join(PNG_DIR, `${name}.yaml`)

// Flat `key: value` + `origin: [x, y, yaw]` + # comments: all a map yaml holds.
function parseMapYaml(text) {
  const out = {}
  for (const rawLine of text.split('\n')) {
    const line = rawLine.replace(/\s+#.*$/, '').trim()
    if (!line || line.startsWith('#') || line.startsWith('-')) continue
    const sep = line.indexOf(':')
    if (sep < 0) continue
    const key = line.slice(0, sep).trim()
    let value = line.slice(sep + 1).trim()
    if (!key) continue
    if (value.startsWith('[') && value.endsWith(']')) {
      out[key] = value.slice(1, -1).split(',').map((v) => Number(v.trim()))
      continue
    }
    if ((value.startsWith('"') && value.endsWith('"')) ||
        (value.startsWith("'") && value.endsWith("'"))) {
      out[key] = value.slice(1, -1)
      continue
    }
    out[key] = value !== '' && !Number.isNaN(Number(value)) ? Number(value) : value
  }
  return out
}

// `image` always names <name>.png, so the pair stays self-consistent.
function writeMapYaml({ name, resolution, origin }) {
  const [ox, oy, oyaw] = origin
  return [
    `image: ${name}.png`,
    'mode: trinary',
    `resolution: ${resolution}`,
    `origin: [${ox}, ${oy}, ${oyaw}]`,
    'negate: 0',
    `occupied_thresh: ${MAP_OCCUPIED_THRESH}`,
    `free_thresh: ${MAP_FREE_THRESH}`,
  ].join('\n') + '\n'
}

// tmp + rename, so map_server never reads half a file.
function writeAtomic(file, data) {
  fs.writeFileSync(`${file}.tmp`, data)
  fs.renameSync(`${file}.tmp`, file)
}

// Width / height from the PNG's IHDR (big-endian at 16 / 20), without decoding it.
function readPngSize(filePath) {
  let fd
  try {
    fd = fs.openSync(filePath, 'r')
    const header = Buffer.alloc(24)
    if (fs.readSync(fd, header, 0, 24, 0) < 24) return null
    if (header.toString('ascii', 1, 4) !== 'PNG') return null
    return { width: header.readUInt32BE(16), height: header.readUInt32BE(20) }
  } catch {
    return null
  } finally {
    if (fd !== undefined) fs.closeSync(fd)
  }
}

// A map is the pair: half of one is not listed (map_server can't load it).
function readMap2d(name) {
  const pngPath = png2dPath(name)
  const yamlPath = yaml2dPath(name)
  if (!fs.existsSync(pngPath) || !fs.existsSync(yamlPath)) return null
  let meta
  try {
    meta = parseMapYaml(fs.readFileSync(yamlPath, 'utf-8'))
  } catch {
    return null
  }
  const size = readPngSize(pngPath)
  const stat = fs.statSync(pngPath)
  return {
    name,
    pngBytes: stat.size,
    mtime: stat.mtime.toISOString(),
    width: size?.width ?? null,
    height: size?.height ?? null,
    resolution: typeof meta.resolution === 'number' ? meta.resolution : null,
    origin: Array.isArray(meta.origin) ? meta.origin : null,
    // slam_toolbox's pose graph is there (mapped on this robot, not uploaded / Save As)
    slam: fs.existsSync(path.join(SLAM_DIR, `${name}.posegraph`)),
  }
}

app.use('/api/maps2d', cors({
  exposedHeaders: ['Content-Range', 'Content-Length', 'Accept-Ranges'],
}))

app.use('/api/maps2d', (_req, res, next) => {
  if (fs.existsSync(MAPS_DIR)) return next()
  res.status(503).json({ error: `map store ${MAPS_DIR} not found (not mounted?)` })
})

// Name-descending: with the YYMMDD_ prefix the newest is on top.
app.get('/api/maps2d', (_req, res) => {
  if (!fs.existsSync(PNG_DIR)) return res.json([])
  const names = fs.readdirSync(PNG_DIR)
    .filter((f) => f.endsWith('.png'))
    .map((f) => f.replace(/\.png$/, ''))
    .filter((n) => MAP_NAME_RE.test(n))
    .sort()
    .reverse()
  res.json(names.map(readMap2d).filter(Boolean))
})

app.get('/api/maps2d/:name/image', (req, res) => {
  const { name } = req.params
  if (!MAP_NAME_RE.test(name)) return res.status(400).end()
  if (!fs.existsSync(png2dPath(name))) return res.status(404).json({ error: 'map not found', name })
  // Inline (no Content-Disposition): the editor loads it into an <img>. Never cached: it is
  // rewritten in place by the editor.
  res.set('Cache-Control', 'no-store')
  res.sendFile(`${name}.png`, { root: PNG_DIR })
})

app.get('/api/maps2d/:name/meta', (req, res) => {
  const { name } = req.params
  if (!MAP_NAME_RE.test(name)) return res.status(400).end()
  const entry = readMap2d(name)
  if (!entry) return res.status(404).json({ error: 'map not found', name })
  res.json(entry)
})

// Zipped: a 2D map is the pair.
app.get('/api/maps2d/:name/download', (req, res) => {
  const { name } = req.params
  if (!MAP_NAME_RE.test(name)) return res.status(400).end()
  if (!readMap2d(name)) return res.status(404).json({ error: 'map not found', name })

  res.set('Content-Disposition', `attachment; filename="${name}_map.zip"`)
  res.set('Content-Type', 'application/zip')
  const archive = archiver('zip')
  archive.on('warning', (err) => console.warn('[ui_backend] archiver warning:', err))
  archive.on('error', (err) => {
    console.error('[ui_backend] map2d zip failed:', err.message)
    res.end()
  })
  archive.pipe(res)
  archive.file(png2dPath(name), { name: `${name}.png` })
  archive.file(yaml2dPath(name), { name: `${name}.yaml` })
  archive.finalize()
})

// Upload a png + yaml pair (same base name). The yaml is rewritten in our format.
app.post('/api/maps2d', mapsUpload.fields([{ name: 'png', maxCount: 1 }, { name: 'yaml', maxCount: 1 }]),
  (req, res) => {
    const png = req.files?.png?.[0]
    const yaml = req.files?.yaml?.[0]
    if (!png || !yaml) {
      return res.status(400).json({ error: 'both a .png and a .yaml file are required' })
    }
    const pngName = png.originalname.replace(/\.png$/i, '')
    const yamlName = yaml.originalname.replace(/\.ya?ml$/i, '')
    if (pngName !== yamlName) {
      return res.status(400).json({
        error: `filenames must match: got ${png.originalname} and ${yaml.originalname}`,
      })
    }
    if (!MAP_NAME_RE.test(pngName)) {
      return res.status(400).json({ error: 'map name must match [A-Za-z0-9_]+' })
    }
    if (png.buffer.toString('ascii', 1, 4) !== 'PNG') {
      return res.status(400).json({ error: `${png.originalname} is not a PNG` })
    }
    // Checked before writing: without these the map loads at the wrong scale / place.
    const meta = parseMapYaml(yaml.buffer.toString('utf-8'))
    if (typeof meta.resolution !== 'number' || !Array.isArray(meta.origin) || meta.origin.length < 2) {
      return res.status(400).json({ error: 'yaml must contain a numeric resolution and an origin [x, y, yaw]' })
    }

    fs.mkdirSync(PNG_DIR, { recursive: true })
    const overwritten = fs.existsSync(png2dPath(pngName))
    writeAtomic(png2dPath(pngName), png.buffer)
    writeAtomic(yaml2dPath(pngName), writeMapYaml({
      name: pngName,
      resolution: meta.resolution,
      origin: [meta.origin[0], meta.origin[1], meta.origin[2] ?? 0],
    }))
    res.json({ name: pngName, overwritten })
  })

// Save from the editor: the PNG blob + meta { resolution, origin }.
app.post('/api/maps2d/:name', mapsUpload.single('png'), (req, res) => {
  const { name } = req.params
  if (!MAP_NAME_RE.test(name)) {
    return res.status(400).json({ error: 'map name must match [A-Za-z0-9_]+' })
  }
  if (!req.file) return res.status(400).json({ error: 'no png uploaded' })
  let meta
  try {
    meta = JSON.parse(req.body?.meta ?? '')
  } catch {
    return res.status(400).json({ error: 'meta must be a JSON object' })
  }
  if (typeof meta?.resolution !== 'number' || !Array.isArray(meta.origin) || meta.origin.length < 2) {
    return res.status(400).json({ error: 'meta needs a numeric resolution and an origin array' })
  }

  fs.mkdirSync(PNG_DIR, { recursive: true })
  const overwritten = fs.existsSync(png2dPath(name))
  writeAtomic(png2dPath(name), req.file.buffer)
  writeAtomic(yaml2dPath(name), writeMapYaml({
    name,
    resolution: meta.resolution,
    origin: [meta.origin[0], meta.origin[1], meta.origin[2] ?? 0],
  }))
  res.json({ name, overwritten })
})

// Also removes slam/<name>.* (the pose graph has nothing to come back to without the map).
app.delete('/api/maps2d/:name', (req, res) => {
  const { name } = req.params
  if (!MAP_NAME_RE.test(name)) return res.status(400).end()
  if (!fs.existsSync(png2dPath(name)) && !fs.existsSync(yaml2dPath(name))) {
    return res.status(404).json({ error: 'map not found', name })
  }
  // force: a half pair (not listed) can still be cleaned up
  fs.rmSync(png2dPath(name), { force: true })
  fs.rmSync(yaml2dPath(name), { force: true })
  for (const ext of SLAM_EXTS) fs.rmSync(path.join(SLAM_DIR, `${name}.${ext}`), { force: true })
  res.json({ name, deleted: true })
})

if (isProd) {
  // extensions: /navplan_tester serves navplan_tester.html (the second Vite entry)
  app.use(express.static(path.join(__dirname, 'dist'), { extensions: ['html'] }))
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
