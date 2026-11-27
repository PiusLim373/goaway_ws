import { callService } from '../services/rosbridge'

const PAGES = [
  { page: 'dashboard', label: '◂ Dashboard' },
  { page: 'mapping', label: '🗺️ Mapping' },
  // Its own page (as tmms's, a second Vite entry), so a link rather than App's routing
  { page: 'navplan', label: '🧭 Navplan', href: '/navplan_tester' },
]

// Left: page nav (as tmms). Centre: ESTOP. Right: a spacer, so ESTOP stays centred.
export function Footer({ page, onNavigate }) {
  // damp: Unitree's Damp, or HTX's Damping, whichever locomotion is active
  function handleEstop() {
    callService(
      'damp',
      (res) => console.log('[ESTOP] damp result:', res),
      (err) => console.error('[ESTOP] damp error:', err)
    )
  }

  return (
    <footer
      className="flex items-center flex-shrink-0"
      style={{
        height: 58,
        background: 'var(--panel-bg)',
        borderTop: '1px solid var(--border)',
        padding: '0 16px',
      }}
    >
      <div className="flex items-center gap-2" style={{ flex: 1 }}>
        {PAGES.map(({ page: p, label, href }) => (
          <button
            key={p}
            onClick={() => (href ? window.location.assign(href) : onNavigate(p))}
            className="btn-icon"
            style={{
              padding: '8px 16px',
              fontSize: 12,
              ...(page === p && { borderColor: 'var(--accent-bright)' }),
            }}
          >
            {label}
          </button>
        ))}
      </div>

      <button
        onClick={handleEstop}
        className="flex items-center gap-2 rounded font-medium"
        style={{
          fontFamily: 'var(--font-mono)',
          fontSize: 13,
          letterSpacing: '0.08em',
          padding: '10px 48px',
          background: '#DC2626',
          color: '#ffffff',
          border: 'none',
          cursor: 'pointer',
          transition: 'background 0.1s',
        }}
        onMouseEnter={(e) => { e.currentTarget.style.background = '#B91C1C' }}
        onMouseLeave={(e) => { e.currentTarget.style.background = '#DC2626' }}
        title="Send damp command — all quadruped motors will be disabled and robot will fall"
      >
        <span style={{ fontSize: 15 }}>⚠</span>
        ESTOP — DAMP ALL MOTORS
      </button>

      <div style={{ flex: 1 }} />
    </footer>
  )
}
