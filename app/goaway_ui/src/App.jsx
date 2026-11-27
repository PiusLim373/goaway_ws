import { useState, useEffect, useRef } from 'react'
import { useRos } from './hooks/useRos'
import { useKeyboard } from './hooks/useKeyboard'
import { useTopicActivity } from './hooks/useTopicActivity'
import { setQuadrupedPaused } from './services/rosbridge'
import { lichtblickUrl } from './services/lichtblick'
import { Header } from './components/Header'
import { Footer } from './components/Footer'
import { ResizeHandle } from './components/ui/ResizeHandle'
import { QuadrupedWidget } from './components/widgets/QuadrupedWidget'
import { MappingPage } from './components/MappingPage'

const DASHBOARD_URL = lichtblickUrl('dashboard_layout.json')

export default function App() {
  const [theme, setTheme] = useState(() => localStorage.getItem('theme') || 'dark')

  useEffect(() => {
    document.documentElement.className = theme === 'light' ? 'light' : ''
    localStorage.setItem('theme', theme)
  }, [theme])

  const { connected } = useRos()
  const { heldKeys }  = useKeyboard()
  // Not persisted, as tmms: a reload opens the dashboard. ?page=mapping opens that page (the
  // navplan tester's footer links back here).
  const [page, setPage] = useState(() => (
    new URLSearchParams(window.location.search).get('page') === 'mapping' ? 'mapping' : 'dashboard'))

  // Lichtblick's share of the width. Not persisted — same as tmms's page splits.
  const mainRef = useRef(null)
  const [lichtblickPct, setLichtblickPct] = useState(0.6)

  // Subscribed once for the whole app and passed down: the header's battery and the
  // quadruped widget read the same message, whichever page is open.
  const { active: statusActive, lastMsg: status } = useTopicActivity(
    '/quadruped_main_status', 'htxgrrt_msgs/QuadrupedMainStatus', 1000
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
      {page === 'dashboard' && (
      <div ref={mainRef} style={{ flex: 1, minHeight: 0, display: 'flex', flexDirection: 'row', overflow: 'hidden' }}>
        <div style={{ width: `${lichtblickPct * 100}%`, flexShrink: 0, overflow: 'hidden' }}>
          <iframe
            src={DASHBOARD_URL}
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
      )}

      {page === 'mapping' && (
        <MappingPage heldKeys={heldKeys} status={status} statusActive={statusActive} isPaused={isPaused} />
      )}

      {/* ── Footer ── */}
      <Footer page={page} onNavigate={setPage} />
    </div>
  )
}
