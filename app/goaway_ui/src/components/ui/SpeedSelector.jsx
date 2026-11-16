import { useRef, useState } from 'react'

// Vertical 3-stop speed slider, styled like AxisKnob (JoystickDisplay.jsx): HIGH at the top,
// LOW at the bottom. Dragging snaps to the nearest stop; it doesn't spring back.
// trackLen: match to the paired joystick size. limits: { LOW: { x, y, yaw }, ... } for the tooltip.
const LEVELS = ['LOW', 'MID', 'HIGH']   // bottom → top
const TRACK_CROSS = 10
const KNOB_R = 10
const LABEL_W = 30

export function SpeedSelector({ speed, onChange, trackLen = 80, limits }) {
  const trackRef = useRef()
  const [dragging, setDragging] = useState(false)

  const travel = trackLen - KNOB_R * 2
  const stopY = (i) => trackLen - KNOB_R - (i / (LEVELS.length - 1)) * travel
  const knobY = stopY(Math.max(0, LEVELS.indexOf(speed)))
  const trackX = LABEL_W + KNOB_R
  const svgW = LABEL_W + KNOB_R * 2

  function pick(clientY) {
    const rect = trackRef.current.getBoundingClientRect()
    const t = (trackLen - KNOB_R - (clientY - rect.top)) / travel
    const level = LEVELS[Math.round(Math.min(1, Math.max(0, t)) * (LEVELS.length - 1))]
    if (level !== speed) onChange(level)
  }

  const lim = limits?.[speed]
  const title = (lim
    ? `${speed}: forward ±${lim.x} m/s, strafe ±${lim.y} m/s, turn ±${lim.yaw} rad/s. `
    : '') + 'Drag, or press < / > (click this panel first if Lichtblick has the keyboard).'

  return (
    <div className="flex flex-col items-center gap-0.5" title={title}>
      <span
        style={{
          fontFamily: 'var(--font-mono)',
          fontSize: 9,
          color: 'var(--text-dim)',
          letterSpacing: '0.06em',
          paddingLeft: LABEL_W,
        }}
      >
        SPEED
      </span>

      <svg
        ref={trackRef}
        width={svgW}
        height={trackLen}
        viewBox={`0 0 ${svgW} ${trackLen}`}
        style={{ cursor: 'ns-resize', touchAction: 'none', userSelect: 'none', overflow: 'visible' }}
        onPointerDown={(e) => {
          e.preventDefault()
          e.currentTarget.setPointerCapture(e.pointerId)
          setDragging(true)
          pick(e.clientY)
        }}
        onPointerMove={(e) => { if (dragging) pick(e.clientY) }}
        onPointerUp={() => setDragging(false)}
      >
        {/* Track background */}
        <rect
          x={trackX - TRACK_CROSS / 2} y={0}
          width={TRACK_CROSS} height={trackLen}
          rx={3}
          fill="var(--bg)"
          stroke="var(--border)"
          strokeWidth="1"
        />
        {/* Fill bar from the bottom up to the knob */}
        <rect x={trackX - 2} y={knobY} width={4} height={trackLen - knobY} fill="var(--accent)" />
        {/* Stops: tick + label */}
        {LEVELS.map((level, i) => (
          <g key={level}>
            <line
              x1={trackX - TRACK_CROSS / 2 - 3} y1={stopY(i)}
              x2={trackX + TRACK_CROSS / 2 + 3} y2={stopY(i)}
              stroke="var(--border)"
              strokeWidth="1"
            />
            <text
              x={LABEL_W - 6} y={stopY(i)}
              textAnchor="end" dominantBaseline="middle"
              fill={level === speed ? 'var(--text-h)' : 'var(--text-dim)'}
              fontSize={9} fontFamily="var(--font-mono)"
            >
              {level}
            </text>
          </g>
        ))}
        {/* Knob */}
        <circle
          cx={trackX}
          cy={knobY}
          r={KNOB_R}
          fill={dragging ? 'var(--accent-bright)' : 'var(--accent)'}
          stroke="var(--border)"
          strokeWidth="1"
          style={{ transition: dragging ? 'none' : 'cy 0.1s' }}
        />
      </svg>

      <span
        style={{
          fontFamily: 'var(--font-mono)',
          fontSize: 9,
          color: 'var(--text-h)',
          paddingLeft: LABEL_W,
        }}
      >
        {speed}
      </span>
    </div>
  )
}
