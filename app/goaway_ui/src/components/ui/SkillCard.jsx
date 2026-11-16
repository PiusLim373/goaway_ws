import { useEffect, useLayoutEffect, useRef, useState } from 'react'
import { createPortal } from 'react-dom'

const clipStyle = {
  width: '100%',
  aspectRatio: '16 / 9',
  objectFit: 'cover',
  display: 'block',
  background: 'var(--bg)',
  borderBottom: '1px solid var(--border)',
}

// A notch smaller on narrow screens, so five cards' labels fit at 1366 px.
const labelStyle = (lit) => ({
  padding: '8px 2px',
  fontFamily: 'var(--font-mono)',
  fontSize: 'clamp(11px, 0.8vw, 12px)',
  letterSpacing: '0.08em',
  textAlign: 'center',
  whiteSpace: 'nowrap',
  overflow: 'hidden',
  textOverflow: 'ellipsis',
  color: lit ? '#E5E7EB' : 'var(--text-dim)',
})

// A skill button with its clip (a muted loop, recorded in MuJoCo) above the label.
// The whole card is the button, so it's an easy target on a touch screen.
// Same colours as the old skill buttons; running = the skill currently being performed.
export function SkillCard({ label, clip, enabled, running, title, onClick }) {
  const lit = enabled || running
  return (
    <button
      onClick={onClick}
      disabled={!enabled}
      title={title}
      style={{
        flex: 1,
        minWidth: 0,
        display: 'flex',
        flexDirection: 'column',
        padding: 0,
        overflow: 'hidden',
        borderRadius: 4,
        border: `1px solid ${lit ? 'var(--accent-bright)' : 'var(--border)'}`,
        background: running ? 'var(--accent-bright)' : enabled ? 'var(--accent)' : 'transparent',
        cursor: enabled ? 'pointer' : 'not-allowed',
        opacity: lit ? 1 : 0.5,
        transition: 'all 0.15s',
      }}
    >
      <video src={clip} autoPlay loop muted playsInline style={clipStyle} />
      <span style={labelStyle(lit)}>{label}</span>
    </button>
  )
}

// A clip's caption, bottom left over it.
const captionStyle = {
  position: 'absolute',
  left: 4,
  bottom: 4,
  padding: '1px 4px',
  borderRadius: 2,
  fontFamily: 'var(--font-mono)',
  fontSize: 9,
  letterSpacing: '0.06em',
  color: '#E5E7EB',
  background: 'rgba(0, 0, 0, 0.55)',
}

// Two directions of one skill in one card (the side flips). The clip plays each side's
// clip once in turn, captioned with that side; each half below is its own button.
// A side running (an arm out, for the toggles) keeps its clip on a loop.
// sides: [{ label, clip, enabled, running, title, onClick }]
export function SkillPairCard({ label, sides }) {
  const [next, setNext] = useState(0)
  const running = sides.findIndex((s) => s.running)
  const shown = running >= 0 ? running : next
  const lit = sides.some((s) => s.enabled || s.running)
  return (
    <div
      style={{
        // Wider than one card: two labels share its width.
        flex: 1.25,
        minWidth: 0,
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden',
        borderRadius: 4,
        border: `1px solid ${lit ? 'var(--accent-bright)' : 'var(--border)'}`,
        opacity: lit ? 1 : 0.5,
        transition: 'all 0.15s',
      }}
    >
      <div style={{ position: 'relative' }}>
        <video
          src={sides[shown].clip}
          autoPlay
          loop={running >= 0}
          muted
          playsInline
          onEnded={() => setNext((shown + 1) % sides.length)}
          style={clipStyle}
        />
        <span style={captionStyle}>
          {running >= 0 && '● '}{label} {sides[shown].label}
        </span>
      </div>
      <div style={{ display: 'flex' }}>
        {sides.map((side, i) => {
          const sideLit = side.enabled || side.running
          return (
            <button
              key={side.label}
              onClick={side.onClick}
              disabled={!side.enabled}
              title={side.title}
              style={{
                ...labelStyle(sideLit),
                // Half a card: tighter, so "RIGHT ▶" fits at 1366 px.
                padding: '8px 2px',
                letterSpacing: '0.02em',
                flex: 1,
                minWidth: 0,
                border: 'none',
                borderLeft: i > 0 ? `1px solid ${lit ? 'var(--accent-bright)' : 'var(--border)'}` : 'none',
                borderRadius: 0,
                background: side.running ? 'var(--accent-bright)' : side.enabled ? 'var(--accent)' : 'transparent',
                cursor: side.enabled ? 'pointer' : 'not-allowed',
                transition: 'all 0.15s',
              }}
            >
              {side.label}
            </button>
          )
        })}
      </div>
    </div>
  )
}

// Each flip's cell (row, column) in the menu's 2 x 2 grid.
const CELLS = { front: [1, 1], back: [1, 2], left: [2, 1], right: [2, 2] }
const MENU_GAP = 6
const MENU_PAD = 8
// The card plays the flips' clips one after another, fading from one to the next.
const FADE_S = 0.5

// All the flips behind one card: tapping it opens a 2 x 2 menu of them, each with its clip on
// a loop; tapping one runs it. The menu lies over the panel below the card (in a portal, so
// the panel's overflow doesn't cut it off) and closes on a pick, a tap outside, Escape, or as
// soon as no flip can start. The card plays the clips in turn with a fade; while one flip
// runs, the card is lit, names it and keeps its clip on a loop.
// options: [{ place: 'front' | 'back' | 'left' | 'right', label, name, clip, enabled, running,
// title, onClick }]
export function FlipMenuCard({ label, options, hint }) {
  const [open, setOpen] = useState(false)
  const [box, setBox] = useState(null)  // the card and its label on screen, to place the menu
  const cardRef = useRef(null)
  const menuRef = useRef(null)
  const running = options.find((o) => o.running)
  const enabled = options.some((o) => o.enabled)
  const lit = enabled || !!running
  const [next, setNext] = useState(0)  // the clip the card plays, unless a flip runs
  const videos = useRef([])
  const pinned = options.indexOf(running)
  const shown = pinned >= 0 ? pinned : next

  // Each clip starts over as it fades in; the one fading out holds its last frame.
  useEffect(() => {
    const v = videos.current[shown]
    if (!v) return
    v.currentTime = 0
    v.play().catch(() => {})
  }, [shown])

  useEffect(() => {
    if (!enabled) setOpen(false)
  }, [enabled])

  useLayoutEffect(() => {
    if (!open) return
    const place = () => setBox({
      rect: cardRef.current.getBoundingClientRect(),
      labelHeight: cardRef.current.lastElementChild.getBoundingClientRect().height,
    })
    place()
    window.addEventListener('resize', place)
    return () => window.removeEventListener('resize', place)
  }, [open])

  useEffect(() => {
    if (!open) return
    const onPointerDown = (e) => {
      if (!cardRef.current?.contains(e.target) && !menuRef.current?.contains(e.target)) setOpen(false)
    }
    const onKeyDown = (e) => {
      if (e.key === 'Escape') setOpen(false)
    }
    document.addEventListener('pointerdown', onPointerDown)
    document.addEventListener('keydown', onKeyDown)
    return () => {
      document.removeEventListener('pointerdown', onPointerDown)
      document.removeEventListener('keydown', onKeyDown)
    }
  }, [open])

  // Tiles bigger than a card where there's room, right-aligned with the card (it's the last one
  // in the row), below it and inside the window. Only where the tiles would get much smaller
  // than a card does the menu rise over the card instead.
  let menu = null
  if (open && box) {
    const { rect, labelHeight } = box
    const side = (w) => (w - 2 * MENU_PAD - MENU_GAP) / 2
    const tileFor = (h) => (side(h) - labelHeight - 2) * 16 / 9  // a tile's width for a menu h tall
    const below = window.innerHeight - rect.bottom - 6 - 8
    let tile = Math.min(Math.max(rect.width, 200), side(window.innerWidth - 16), tileFor(below))
    let top = rect.bottom + 6
    if (tile < 0.75 * rect.width) {
      tile = Math.min(rect.width, side(window.innerWidth - 16), tileFor(window.innerHeight - 16))
      top = window.innerHeight - 8 - (2 * (tile * 9 / 16 + labelHeight + 2) + MENU_GAP + 2 * MENU_PAD)
    }
    const width = 2 * tile + MENU_GAP + 2 * MENU_PAD
    const left = Math.min(Math.max(rect.right - width, 8), window.innerWidth - width - 8)
    menu = createPortal(
      <div
        ref={menuRef}
        role="menu"
        aria-label={label}
        style={{
          position: 'fixed',
          top,
          left,
          width,
          zIndex: 55,
          padding: MENU_PAD,
          display: 'grid',
          gridTemplateColumns: `repeat(2, ${tile}px)`,
          gap: MENU_GAP,
          alignItems: 'center',
          background: 'var(--bg)',
          border: '1px solid var(--accent)',
          borderRadius: 4,
          boxShadow: '0 2px 10px rgba(0,0,0,0.5)',
        }}
      >
        {options.map((o) => {
          const [row, col] = CELLS[o.place]
          return (
            <button
              key={o.place}
              role="menuitem"
              onClick={() => {
                setOpen(false)
                o.onClick()
              }}
              disabled={!o.enabled}
              title={o.enabled ? o.title : hint}
              style={{
                gridRow: row,
                gridColumn: col,
                display: 'flex',
                flexDirection: 'column',
                padding: 0,
                overflow: 'hidden',
                borderRadius: 4,
                border: `1px solid ${o.enabled ? 'var(--accent-bright)' : 'var(--border)'}`,
                background: o.enabled ? 'var(--accent)' : 'transparent',
                cursor: o.enabled ? 'pointer' : 'not-allowed',
                opacity: o.enabled ? 1 : 0.5,
                transition: 'all 0.15s',
              }}
            >
              <video src={o.clip} autoPlay loop muted playsInline style={clipStyle} />
              <span style={labelStyle(o.enabled)}>{o.label}</span>
            </button>
          )
        })}
      </div>,
      document.body,
    )
  }

  return (
    <>
      <button
        ref={cardRef}
        onClick={() => setOpen((v) => !v)}
        disabled={!enabled}
        title={enabled ? `${label}: tap for the directions` : hint}
        aria-haspopup="menu"
        aria-expanded={open}
        style={{
          flex: 1,
          minWidth: 0,
          display: 'flex',
          flexDirection: 'column',
          padding: 0,
          overflow: 'hidden',
          borderRadius: 4,
          border: `1px solid ${lit ? 'var(--accent-bright)' : 'var(--border)'}`,
          background: running || open ? 'var(--accent-bright)' : enabled ? 'var(--accent)' : 'transparent',
          cursor: enabled ? 'pointer' : 'not-allowed',
          opacity: lit ? 1 : 0.5,
          transition: 'all 0.15s',
        }}
      >
        <div style={{ ...clipStyle, position: 'relative' }}>
          {options.map((o, i) => (
            <video
              key={o.place}
              ref={(el) => {
                videos.current[i] = el
              }}
              src={o.clip}
              muted
              playsInline
              preload="auto"
              loop={i === pinned}
              onEnded={() => {
                if (i === shown) setNext((shown + 1) % options.length)
              }}
              style={{
                position: 'absolute',
                inset: 0,
                width: '100%',
                height: '100%',
                objectFit: 'cover',
                opacity: i === shown ? 1 : 0,
                transition: `opacity ${FADE_S}s`,
              }}
            />
          ))}
          <span style={captionStyle}>{options[shown].label}</span>
        </div>
        <span style={labelStyle(lit)}>{running ? `● ${running.name}` : `${label} ▾`}</span>
      </button>
      {menu}
    </>
  )
}
