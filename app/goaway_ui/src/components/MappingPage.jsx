import { useRef, useState } from 'react'
import { lichtblickUrl } from '../services/lichtblick'
import { setQuadrupedPaused } from '../services/rosbridge'
import { ResizeHandle } from './ui/ResizeHandle'
import { MappingWidget } from './widgets/MappingWidget'
import { QuadrupedWidget } from './widgets/QuadrupedWidget'

const MAPPING_URL = lichtblickUrl('mapping_layout.json')
// Below about this the quadruped widget's joystick row overlaps its neighbours (walk mode,
// showSkills off): it scrolls instead, and the card's auto height leaves it this much where
// the screen allows (the MAPS list keeps at least 200 px).
const QUAD_MIN_PX = 540

// Lichtblick with the live map (/mapping/map) | the MAPPING card above the quadruped widget,
// so the robot can be driven while it maps.
export function MappingPage({ heldKeys, status, statusActive, isPaused }) {
  const mainRef = useRef(null)
  const rightColRef = useRef(null)
  const [lichtblickPct, setLichtblickPct] = useState(0.6)
  // null: the card fits its tab (capped, the map list scrolls); a drag fixes the split.
  // Not persisted, as the other splits.
  const [cardPct, setCardPct] = useState(null)

  return (
    <div ref={mainRef} style={{ flex: 1, minHeight: 0, display: 'flex', flexDirection: 'row', overflow: 'hidden' }}>
      <div style={{ width: `${lichtblickPct * 100}%`, flexShrink: 0, overflow: 'hidden' }}>
        <iframe
          src={MAPPING_URL}
          title="Lichtblick mapping"
          style={{ width: '100%', height: '100%', border: 'none' }}
        />
      </div>

      <ResizeHandle direction="h" containerRef={mainRef} onResize={setLichtblickPct} min={0.25} max={0.85} />

      <div ref={rightColRef} style={{ flex: 1, minWidth: 0, overflow: 'hidden', display: 'flex', flexDirection: 'column' }}>
        <div
          style={{
            flexShrink: 0, minHeight: 0, overflow: 'hidden', display: 'flex', flexDirection: 'column',
            ...(cardPct == null
              ? { maxHeight: `min(45%, max(200px, calc(100% - ${QUAD_MIN_PX}px)))` }
              : { height: `${cardPct * 100}%` }),
          }}
        >
          <MappingWidget blocked={!isPaused} />
        </div>

        <ResizeHandle direction="v" containerRef={rightColRef} onResize={setCardPct} min={0.12} max={0.85} />

        <div style={{ flex: 1, minHeight: 0, overflowY: 'auto', overflowX: 'hidden' }}>
          <div style={{ height: '100%', minHeight: QUAD_MIN_PX }}>
            <QuadrupedWidget
              heldKeys={heldKeys}
              status={status}
              statusActive={statusActive}
              blocked={!isPaused}
              onPause={() => setQuadrupedPaused(true)}
              showSkills={false}
            />
          </div>
        </div>
      </div>
    </div>
  )
}
