import { useState, useEffect, useRef } from 'react'
import { useRos } from './hooks/useRos'
import { useKeyboard } from './hooks/useKeyboard'
import { useTopicActivity } from './hooks/useTopicActivity'
import { setQuadrupedPaused, WS_URL } from './services/rosbridge'
import { Header } from './components/Header'
import { Footer } from './components/Footer'
import { ResizeHandle } from './components/ui/ResizeHandle'
import { QuadrupedWidget } from './components/widgets/QuadrupedWidget'

// goaway_lichtblick (Caddy on :8080). ds/ds.url make Lichtblick auto-connect to rosbridge on
// load; layoutUrl re-fetches and re-applies dashboard_layout.json on every load, overriding
// whatever layout is cached in the browser (see goaway_lichtblick-compose.yaml).
const LICHTBLICK = `http://${window.location.hostname}:8080`
const LICHTBLICK_URL = `${LICHTBLICK}/?ds=rosbridge-websocket&ds.url=${encodeURIComponent(WS_URL)}`
  + `&layoutUrl=${encodeURIComponent(`${LICHTBLICK}/layouts/dashboard_layout.json`)}`

export default function App() {
  const [theme, setTheme] = useState(() => localStorage.getItem('theme') || 'dark')

  useEffect(() => {
    document.documentElement.className = theme === 'light' ? 'light' : ''
    localStorage.setItem('theme', theme)
  }, [theme])

  const { connected } = useRos()
  const { heldKeys }  = useKeyboard()

  // Lichtblick's share of the width. Not persisted — same as tmms's page splits.
  const mainRef = useRef(null)
  const [lichtblickPct, setLichtblickPct] = useState(0.6)

  // Subscribed once for the whole app and passed down: the header's battery and the
  // quadruped widget read the same message.
  const { active: statusActive, lastMsg: status } = useTopicActivity(
    '/quadruped_main_status', 'goaway_msgs/QuadrupedMainStatus', 1000
  )

  const isPaused = status?.is_paused ?? true   // assume paused until told otherwise
  const battery = statusActive ? status?.battery_percentage : undefined

  return (
    <div
      style={{
        width: '100vw',
        height: '100vh',
        display: 'flex',
        flexDirection: 'column',
        background: 'var(--bg)',
        overflow: 'hidden',
      }}
    >
      {/* ── Header (52px) ── */}
      <Header
        connected={connected}
        battery={battery}
        theme={theme}
        onThemeToggle={() => setTheme((t) => (t === 'dark' ? 'light' : 'dark'))}
      />

      {/* ── Dashboard: Lichtblick (robot model, sensors) | quadruped control ── */}
      <div ref={mainRef} style={{ flex: 1, minHeight: 0, display: 'flex', flexDirection: 'row', overflow: 'hidden' }}>
        <div style={{ width: `${lichtblickPct * 100}%`, flexShrink: 0, overflow: 'hidden' }}>
          <iframe
            src={LICHTBLICK_URL}
            title="Lichtblick"
            style={{ width: '100%', height: '100%', border: 'none' }}
          />
        </div>

        <ResizeHandle
          direction="h"
          containerRef={mainRef}
          onResize={setLichtblickPct}
          min={0.25}
          max={0.85}
        />

        <div style={{ flex: 1, minWidth: 0, overflow: 'hidden' }}>
          <QuadrupedWidget
            heldKeys={heldKeys}
            status={status}
            statusActive={statusActive}
            blocked={!isPaused}
            onPause={() => setQuadrupedPaused(true)}
          />
        </div>
      </div>

      {/* ── Footer ── */}
      <Footer />
    </div>
  )
}
