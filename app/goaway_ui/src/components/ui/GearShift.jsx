// Order left→right: SIT(1) → STAND LOCK(2) → WALK(3), with UPRIGHT WALK(4) branching off
// STAND LOCK. User clicks ◀/▶ and ▼/▲ to change modes. Mode labels are display-only.
const MODES = [
  { mode: 1, label: 'SIT' },
  { mode: 2, label: 'STAND LOCK' },
  { mode: 3, label: 'WALK' },
]
const UPRIGHT = { mode: 4, label: 'UPRIGHT WALK' }

const arrowBtn = (disabled, label, onClick, size = { width: 40 }) => (
  <button
    onClick={onClick}
    disabled={disabled}
    style={{
      flexShrink: 0,
      ...size,
      fontFamily: 'var(--font-mono)',
      fontSize: 14,
      background: 'transparent',
      border: '1px solid var(--border)',
      borderRadius: 4,
      color: disabled ? 'var(--border)' : 'var(--text)',
      cursor: disabled ? 'not-allowed' : 'pointer',
      opacity: disabled ? 0.35 : 1,
      transition: 'all 0.15s',
      lineHeight: 1,
    }}
    onMouseEnter={(e) => {
      if (!disabled) {
        e.currentTarget.style.borderColor = 'var(--accent-bright)'
        e.currentTarget.style.color = 'var(--text-h)'
      }
    }}
    onMouseLeave={(e) => {
      if (!disabled) {
        e.currentTarget.style.borderColor = 'var(--border)'
        e.currentTarget.style.color = 'var(--text)'
      }
    }}
  >
    {label}
  </button>
)

const tile = (m, isActive) => (
  <div
    key={m.mode}
    style={{
      flex: 1,
      minWidth: 0,
      padding: '8px 6px',
      fontFamily: 'var(--font-mono)',
      fontSize: 12,
      letterSpacing: '0.08em',
      textTransform: 'uppercase',
      whiteSpace: 'nowrap',
      overflow: 'hidden',
      textOverflow: 'ellipsis',
      lineHeight: 1.3,
      textAlign: 'center',
      borderRadius: 4,
      border: isActive
        ? '1px solid var(--accent-bright)'
        : '1px solid var(--border)',
      background: isActive ? 'var(--accent)' : 'transparent',
      color: isActive ? '#E5E7EB' : 'var(--text-dim)',
      userSelect: 'none',
    }}
  >
    <span
      style={{
        fontSize: 9,
        opacity: 0.6,
        display: 'block',
        marginBottom: 2,
        color: isActive ? '#C4B5FD' : 'inherit',
      }}
    >
      {m.mode}
    </span>
    {m.label}
  </div>
)

// uprightAvailable: HTX only. The branch sits under STAND LOCK, the row's centre tile, in the
// same columns as the row above; its arrow is at the bottom, as ◀ ▶ are at the row's ends.
export function GearShift({ mode, onModeChange, disabled, uprightAvailable }) {
  const upright = mode === UPRIGHT.mode
  const canBranch = uprightAvailable && !disabled && (mode === 2 || upright)
  const underStandLock = (content) => (
    <div className="flex gap-1" style={{ opacity: uprightAvailable ? 1 : 0.35 }}>
      <span style={{ flexShrink: 0, width: 40 }} />
      <span style={{ flex: 1, minWidth: 0 }} />
      {content}
      <span style={{ flex: 1, minWidth: 0 }} />
      <span style={{ flexShrink: 0, width: 40 }} />
    </div>
  )
  return (
    <div className="flex flex-col gap-1" style={{ width: '100%' }}>
      <span
        style={{
          fontFamily: 'var(--font-mono)',
          fontSize: 10,
          color: 'var(--text-dim)',
          letterSpacing: '0.06em',
          marginBottom: 4,
        }}
      >
        ROBOT MODE
      </span>

      <div className="flex gap-1" style={{ alignItems: 'stretch' }}>
        {arrowBtn(disabled || upright || mode <= 1, '◀', () => onModeChange(mode - 1))}

        {MODES.map((m) => tile(m, mode === m.mode))}

        {arrowBtn(disabled || mode >= 3, '▶', () => onModeChange(mode + 1))}
      </div>

      {underStandLock(tile(UPRIGHT, upright))}
      {underStandLock(
        arrowBtn(!canBranch, upright ? '▲' : '▼', () => onModeChange(upright ? 2 : UPRIGHT.mode), {
          flex: 1,
          minWidth: 0,
          height: 24,
        })
      )}
    </div>
  )
}
