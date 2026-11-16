// Two-position switch: options[0] on the left, options[1] on the right.
// value: the active policy (undefined = unknown, knob centred). target: a switch in progress,
// shown as the knob sitting on the target and pulsing. onRequest(policy): the other side.
const TRACK_W = 64
const TRACK_H = 28
const KNOB = 22

export function LocomotionToggle({ options, value, target, enabled, onRequest }) {
  const shown = target ?? value
  const idx = options.findIndex((o) => o.policy === shown)
  const knobLeft = idx < 0 ? (TRACK_W - KNOB) / 2 - 1 : idx === 0 ? 2 : TRACK_W - KNOB - 4
  const other = options.find((o) => o.policy !== value)
  const on = idx === 1

  const labelStyle = (policy) => ({
    fontFamily: 'var(--font-mono)',
    fontSize: 12,
    letterSpacing: '0.08em',
    color: policy === shown ? 'var(--text-h)' : 'var(--text-dim)',
    transition: 'color 0.15s',
  })

  return (
    <button
      role="switch"
      aria-checked={on}
      onClick={() => other && onRequest(other.policy)}
      disabled={!enabled}
      title={enabled && other ? `Switch to ${other.label}` : undefined}
      className="flex items-center gap-3"
      style={{
        background: 'transparent',
        border: 'none',
        padding: 0,
        cursor: enabled ? 'pointer' : 'not-allowed',
        opacity: enabled || target ? 1 : 0.5,
      }}
    >
      <span style={labelStyle(options[0].policy)}>{options[0].label}</span>
      <span
        style={{
          position: 'relative',
          width: TRACK_W,
          height: TRACK_H,
          borderRadius: TRACK_H / 2,
          border: `1px solid ${on ? 'var(--accent-bright)' : 'var(--border)'}`,
          background: on ? 'var(--accent)' : 'transparent',
          transition: 'all 0.2s',
          flexShrink: 0,
        }}
      >
        <span
          className={target ? 'pulse-soft' : undefined}
          style={{
            position: 'absolute',
            top: (TRACK_H - KNOB) / 2 - 1,
            left: knobLeft,
            width: KNOB,
            height: KNOB,
            borderRadius: '50%',
            background: idx < 0 ? 'var(--border)' : on ? '#E5E7EB' : 'var(--text)',
            transition: 'left 0.2s, background 0.2s',
          }}
        />
      </span>
      <span style={labelStyle(options[1].policy)}>{options[1].label}</span>
    </button>
  )
}
