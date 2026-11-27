import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useTopicActivity } from '../../hooks/useTopicActivity'
import { startMapping, stopMapping } from '../../services/rosbridge'
import { imageToCells, loadMapImage } from '../../lib/gridCodec'
import { todayPrefix } from '../../lib/dates'
import { MapEditorModal } from '../ui/MapEditorModal'
import { Toast } from '../ui/Toast'
import { WarningModal } from '../ui/WarningModal'

// mapping_manager's state (JSON, latched, 1 Hz): state idle | starting | mapping | saving,
// map_name, started_at, scans_paused, map_size [w, h] m, last_saved, message.
const STATUS_TOPIC = '/mapping_manager/status'
// Same rule as mapping_manager and tmms: letters, digits, _
const NAME_RE = /^[A-Za-z0-9_]+$/
const PNG_DIR = '~/.htxgrrt/maps/png/'

const mono = { fontFamily: 'var(--font-mono)' }
const labelStyle = { ...mono, fontSize: 10, color: 'var(--text-dim)', letterSpacing: '0.06em' }
const ellipsis = { whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis', minWidth: 0 }
const bigBtn = { padding: '6px 14px', fontSize: 12 }

function clock(since) {
  const s = Math.max(0, Math.floor(Date.now() / 1000 - since))
  return `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`
}

function formatBytes(bytes) {
  if (bytes < 1024 * 1024) return `${Math.max(1, Math.round(bytes / 1024))} kB`
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`
}

// fetch -> JSON, or throw the backend's { error } (e.g. the map store isn't mounted).
async function api(url, init) {
  const r = await fetch(url, init)
  if (r.ok) return r.json()
  const body = await r.json().catch(() => ({}))
  throw new Error(body.error || `HTTP ${r.status}`)
}

// As tmms's 3D map section: the session on top (＋ NEW MAP -> name + START -> mapping +
// ✎ EDIT & SAVE), the maps in the system below it, always shown (edit, download, delete,
// upload). blocked: navigation owns the robot (unpaused); mapping_manager refuses then.
export function MappingWidget({ blocked }) {
  const { active, lastMsg } = useTopicActivity(STATUS_TOPIC, 'std_msgs/String', 3000)
  const st = useMemo(() => {
    try { return JSON.parse(lastMsg?.data) } catch { return null }
  }, [lastMsg])
  const [creating, setCreating] = useState(false)  // the name row is open
  const [name, setName] = useState(todayPrefix)
  const [busy, setBusy] = useState(false)
  const [result, setResult] = useState(null)   // { ok, text } of the last start / stop

  const [maps2d, setMaps2d] = useState([])
  const [storeError, setStoreError] = useState(null)
  const [editor, setEditor] = useState(null)   // null | { source }
  const [opening, setOpening] = useState(null) // map name whose editor is loading
  const [uploadError, setUploadError] = useState(null)
  const [warning, setWarning] = useState(null) // null | { title, body, onConfirm }
  const [toast, setToast] = useState(null)     // null | { message, tone }
  const toastTimerRef = useRef(null)
  const uploadRef = useRef(null)

  const state = active ? st?.state : undefined
  const mapping = state === 'mapping'
  const inSession = mapping || state === 'saving'

  // Ticks the elapsed time while mapping.
  const [, setTick] = useState(0)
  useEffect(() => {
    if (!mapping) return
    const id = setInterval(() => setTick((t) => t + 1), 1000)
    return () => clearInterval(id)
  }, [mapping])

  const showToast = useCallback((message, tone = 'ok') => {
    setToast({ message, tone })
    clearTimeout(toastTimerRef.current)
    toastTimerRef.current = setTimeout(() => setToast(null), 4000)
  }, [])
  useEffect(() => () => clearTimeout(toastTimerRef.current), [])

  const fetchMaps2d = useCallback(() => {
    api('/api/maps2d')
      .then((list) => { setMaps2d(list); setStoreError(null) })
      .catch((err) => setStoreError(err.message))
  }, [])
  useEffect(() => { fetchMaps2d() }, [fetchMaps2d])

  // -- session --------------------------------------------------------------

  const validName = NAME_RE.test(name)
  const nameTaken = maps2d.some((m) => m.name === name)
  const canStart = active && state === 'idle' && validName && !nameTaken && !busy && !blocked
  const canStop = mapping && !busy

  const openNew = () => {
    setName(todayPrefix())
    setResult(null)
    setCreating(true)
  }

  const start = () => {
    if (!canStart) return
    setBusy(true)
    setResult(null)
    startMapping(name,
      (res) => {
        setBusy(false)
        setResult({ ok: res.success, text: res.message })
        if (res.success) setCreating(false)
      },
      (err) => { setBusy(false); setResult({ ok: false, text: `Error: ${err}` }) })
  }

  // Save first, then edit: the map is on disk before the editor opens, so closing the
  // editor (or the page) keeps the unedited map.
  const stop = () => {
    if (!canStop) return
    const session = st.map_name
    setBusy(true)
    setResult(null)
    stopMapping(
      (res) => {
        setBusy(false)
        setResult({ ok: res.success, text: res.message })
        fetchMaps2d()
        if (res.message?.startsWith('saved png/')) openEditor(session)
      },
      (err) => { setBusy(false); setResult({ ok: false, text: `Error: ${err}` }) })
  }

  // -- maps -----------------------------------------------------------------

  async function openEditor(mapName) {
    setOpening(mapName)
    try {
      const entry = await api(`/api/maps2d/${mapName}/meta`)
      const img = await loadMapImage(mapName)
      const { cells, width, height } = imageToCells(img)
      setEditor({
        source: {
          cells, width, height,
          resolution: entry.resolution ?? 0.05,
          origin: entry.origin ?? [0, 0, 0],
          name: mapName,
        },
      })
    } catch (err) {
      showToast(`Could not open ${mapName}: ${err.message}`, 'warn')
    } finally {
      setOpening(null)
    }
  }

  async function saveFromEditor(mapName, blob, meta) {
    const form = new FormData()
    form.append('png', blob, `${mapName}.png`)
    form.append('meta', JSON.stringify(meta))
    await api(`/api/maps2d/${mapName}`, { method: 'POST', body: form })
    fetchMaps2d()
    showToast(`Saved ${mapName}`)
  }

  function upload(e) {
    const files = Array.from(e.target.files || [])
    e.target.value = ''
    if (files.length === 0) return
    const png = files.find((f) => /\.png$/i.test(f.name))
    const yaml = files.find((f) => /\.ya?ml$/i.test(f.name))
    if (files.length !== 2 || !png || !yaml) {
      setUploadError('Select two files: the .png and its .yaml')
      return
    }
    setUploadError(null)

    const mapName = png.name.replace(/\.png$/i, '')
    const doIt = () => {
      setBusy(true)
      const form = new FormData()
      form.append('png', png)
      form.append('yaml', yaml)
      api('/api/maps2d', { method: 'POST', body: form })
        .then(({ name: saved, overwritten }) => {
          showToast(overwritten ? `Replaced ${saved}` : `Uploaded ${saved}`)
          fetchMaps2d()
        })
        .catch((err) => setUploadError(`Upload failed: ${err.message}`))
        .finally(() => setBusy(false))
    }

    if (maps2d.some((m) => m.name === mapName)) {
      setWarning({
        title: `"${mapName}" already exists`,
        body: 'Uploading overwrites both the .png and the .yaml on disk. This cannot be undone.',
        onConfirm: () => { setWarning(null); doIt() },
      })
    } else {
      doIt()
    }
  }

  function remove(entry) {
    setWarning({
      title: `"${entry.name}" will be deleted`,
      body: `This permanently removes ${entry.name}.png and ${entry.name}.yaml`
        + (entry.slam ? ', and its slam_toolbox pose graph in slam/' : '')
        + '. This cannot be undone.',
      onConfirm: () => {
        setWarning(null)
        setBusy(true)
        api(`/api/maps2d/${entry.name}`, { method: 'DELETE' })
          .then(() => { showToast(`Deleted ${entry.name}`); fetchMaps2d() })
          .catch((err) => showToast(`Delete failed: ${err.message}`, 'warn'))
          .finally(() => setBusy(false))
      },
    })
  }

  // -- render ---------------------------------------------------------------

  let line, color = 'var(--text-dim)'
  if (!active) {
    line = '⚠ mapping_manager not running'
    color = '#EF4444'
  } else if (state === 'starting') {
    line = '⟳ Starting slam_toolbox…'
    color = 'var(--accent-blue)'
  } else if (mapping) {
    line = st.map_size ? `map ${st.map_size[0]} × ${st.map_size[1]} m · ` : ''
    line += "walk slowly, don't spin on the spot"
  } else if (state === 'saving') {
    line = '⟳ Saving the map…'
    color = 'var(--accent-blue)'
  } else if (opening) {
    line = `⟳ Opening ${opening} in the editor…`
    color = 'var(--accent-blue)'
  } else {
    line = st?.last_saved ? `Last saved: png/${st.last_saved}.png` : 'Ready'
  }

  const nameHint = !creating || state !== 'idle' ? null
    : name && !validName ? 'Letters, digits and _ only'
    : nameTaken ? `"${name}" exists: pick another name`
    : blocked ? 'Navigation owns the robot: pause it to start mapping'
    : null
  const side = nameHint ?? result?.text ?? (state === 'idle' && st?.message?.startsWith('map NOT') ? st.message : null)

  return (
    <div className="panel flex flex-col" style={{ flex: '1 1 auto', minHeight: 0, overflow: 'hidden' }}>
      <div className="panel-header" style={{ gap: 10 }}>
        <span>MAPPING</span>
        {/* textTransform none: the header is uppercase, map names are case-sensitive */}
        <div style={{ minWidth: 0, textTransform: 'none' }}>
          {toast && <Toast visible message={toast.message} tone={toast.tone} />}
        </div>
      </div>

      {/* Session */}
      <div className="flex flex-col gap-2 p-3" style={{ flexShrink: 0, borderBottom: '1px solid var(--border)' }}>
        <div className="flex items-center gap-2" style={{ minHeight: 30 }}>
          {inSession ? (
            <>
              <span style={{ ...mono, ...ellipsis, flex: 1, fontSize: 12, color: mapping ? '#22C55E' : 'var(--accent-blue)' }}>
                ● MAPPING {st.map_name}{mapping ? ` — ${clock(st.started_at)}` : ''}
                {mapping && st.scans_paused && (
                  <span style={{ color: '#F59E0B' }}> — scans paused (skill / upright walk)</span>
                )}
              </span>
              <button
                onClick={stop}
                disabled={!canStop}
                className="btn-icon"
                title={canStop ? 'Stop, save to png/<name>.png, then open the map editor' : undefined}
                style={{ ...bigBtn, opacity: canStop ? 1 : 0.4, borderColor: canStop ? '#EF4444' : undefined }}
              >
                ✎ EDIT &amp; SAVE
              </button>
            </>
          ) : creating || state === 'starting' ? (
            <>
              <span style={labelStyle}>MAP NAME</span>
              <input
                value={name}
                autoFocus
                onChange={(e) => setName(e.target.value.trim())}
                // Caret after the date seed while it is untouched (tmms)
                onFocus={(e) => {
                  if (e.target.value === todayPrefix()) {
                    const end = e.target.value.length
                    e.target.setSelectionRange(end, end)
                  }
                }}
                onKeyDown={(e) => {
                  if (e.key === 'Enter') start()
                  if (e.key === 'Escape') setCreating(false)
                }}
                disabled={!active || state !== 'idle' || busy}
                placeholder={`${todayPrefix()}lab_level2`}
                spellCheck={false}
                style={{
                  ...mono,
                  flex: 1,
                  minWidth: 0,
                  fontSize: 12,
                  padding: '6px 8px',
                  borderRadius: 4,
                  border: `1px solid ${name && (!validName || nameTaken) ? '#EF4444' : 'var(--border)'}`,
                  background: 'var(--bg)',
                  color: 'var(--text-h)',
                }}
              />
              <button
                onClick={start}
                disabled={!canStart}
                className="btn-icon"
                title={canStart ? 'Start a mapping session' : undefined}
                style={{ ...bigBtn, opacity: canStart ? 1 : 0.4, borderColor: canStart ? 'var(--accent-bright)' : undefined }}
              >
                ● START
              </button>
              <button
                onClick={() => setCreating(false)}
                disabled={busy || state === 'starting'}
                className="btn-icon"
                title="Cancel (Esc)"
                style={{ ...bigBtn, padding: '6px 10px' }}
              >
                ✕
              </button>
            </>
          ) : (
            <button
              onClick={openNew}
              disabled={!active || state !== 'idle'}
              className="btn-icon"
              title="Name a new map, then start mapping"
              style={{ ...bigBtn, borderColor: active && state === 'idle' ? 'var(--accent-bright)' : undefined }}
            >
              ＋ NEW MAP
            </button>
          )}
        </div>

        {/* State on the left, the last result / name hint on the right: one line */}
        <div className="flex items-center justify-between gap-3" style={{ minHeight: 16, minWidth: 0 }}>
          <span style={{ ...mono, ...ellipsis, fontSize: 11, color }}>{line}</span>
          {side && (
            <span
              title={side}
              style={{
                ...mono, ...ellipsis, fontSize: 10,
                color: nameHint || (result && !result.ok) || !result ? '#F87171' : 'var(--text-dim)',
              }}
            >
              {side}
            </span>
          )}
        </div>
      </div>

      {/* Maps in the system */}
      <div style={{ flex: 1, minHeight: 0, overflowY: 'auto', padding: '8px 10px', display: 'flex', flexDirection: 'column', gap: 6 }}>
        <div className="flex items-center gap-2">
          <span style={labelStyle}>MAPS{maps2d.length > 0 ? ` (${maps2d.length})` : ''}</span>
          <span style={{ ...mono, ...ellipsis, flex: 1, fontSize: 10, color: 'var(--text-dim)' }}>{PNG_DIR}</span>
          <button
            className="btn-icon"
            style={{ fontSize: 11, padding: '4px 10px', flexShrink: 0 }}
            onClick={() => uploadRef.current?.click()}
            disabled={busy || storeError != null}
            title="Add a map: pick its .png and .yaml together"
          >
            ⬆ Upload .png + .yaml
          </button>
          <input
            ref={uploadRef} type="file" accept=".png,.yaml,.yml" multiple
            style={{ display: 'none' }} onChange={upload}
          />
        </div>
        {uploadError && <div style={{ ...mono, fontSize: 10, color: '#EF4444' }}>{uploadError}</div>}

        {storeError ? (
          <div style={{ ...mono, fontSize: 11, color: '#EF4444' }}>⚠ Maps unavailable: {storeError}</div>
        ) : maps2d.length === 0 ? (
          <div style={{ fontSize: 12, color: 'var(--text-dim)' }}>No maps yet: ＋ NEW MAP, or upload one.</div>
        ) : maps2d.map((entry) => (
          <div
            key={entry.name}
            className="flex items-center justify-between"
            style={{ gap: 8, padding: '5px 8px', border: '1px solid var(--border)', borderRadius: 4, flexShrink: 0 }}
          >
            <div style={{ minWidth: 0 }}>
              <div style={{ ...mono, ...ellipsis, fontSize: 12, color: 'var(--text-h)' }} title={entry.name}>
                {entry.name}
              </div>
              <div style={{ ...mono, ...ellipsis, fontSize: 10, color: 'var(--text-dim)' }}>
                {entry.width && entry.height ? `${entry.width}×${entry.height} px` : '? px'}
                {entry.resolution ? ` · ${entry.resolution} m/px` : ''}
                {entry.width && entry.resolution
                  ? ` · ${(entry.width * entry.resolution).toFixed(1)}×${(entry.height * entry.resolution).toFixed(1)} m` : ''}
                {' · '}{formatBytes(entry.pngBytes)}
              </div>
            </div>
            <div className="flex" style={{ gap: 6, flexShrink: 0 }}>
              <button
                className="btn-icon" style={{ fontSize: 11, padding: '4px 8px' }}
                onClick={() => openEditor(entry.name)} disabled={busy || opening != null}
                title="Open in the map editor (Save / Save as)"
              >
                {opening === entry.name ? '⟳ Opening…' : '✎ Edit'}
              </button>
              <a
                className="btn-icon"
                style={{ fontSize: 11, padding: '4px 8px', textDecoration: 'none' }}
                href={`/api/maps2d/${entry.name}/download`}
                download={`${entry.name}_map.zip`}
                title="Zip of the .png + .yaml"
              >
                ⬇ Download
              </a>
              <button
                className="btn-icon"
                style={{ fontSize: 11, padding: '4px 8px', borderColor: '#DC2626', color: '#DC2626' }}
                onClick={() => remove(entry)} disabled={busy}
              >
                ✕ Delete
              </button>
            </div>
          </div>
        ))}
      </div>

      <WarningModal
        open={warning != null}
        heading="MAP FILES"
        title={warning?.title}
        body={warning?.body}
        onConfirm={warning?.onConfirm}
        onCancel={() => setWarning(null)}
      />

      <MapEditorModal
        open={editor != null}
        source={editor?.source}
        existingNames={maps2d.map((m) => m.name)}
        onSave={saveFromEditor}
        onClose={() => { setEditor(null); fetchMaps2d() }}
      />
    </div>
  )
}
