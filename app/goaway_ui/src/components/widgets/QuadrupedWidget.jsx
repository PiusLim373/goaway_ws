import { Fragment, useState, useEffect, useRef, useCallback, useMemo } from 'react'
import { publishQuadrupedCmdVel, callService, subscribe } from '../../services/rosbridge'
import { useTopicActivity } from '../../hooks/useTopicActivity'
import { GearShift } from '../ui/GearShift'
import { SpeedSelector } from '../ui/SpeedSelector'
import { HeightMapLegend, LEGEND_W } from '../ui/HeightMapLegend'
import { JoystickDisplay, AxisKnob } from '../ui/JoystickDisplay'
import { WarningModal } from '../ui/WarningModal'
import { LocomotionToggle } from '../ui/LocomotionToggle'
import { FlipMenuCard, SkillCard, SkillPairCard } from '../ui/SkillCard'

// Max command per level: x forward/back and y strafe (m/s), yaw turn (rad/s).
// All inside HTX perceptive_v2's trained ranges (x ±1.0, y ±0.5, yaw ±1.0), so none get clamped.
const SPEEDS = {
  LOW:  { x: 0.3, y: 0.3, yaw: 0.5 },
  MID:  { x: 0.7, y: 0.4, yaw: 0.7 },
  HIGH: { x: 1.0, y: 0.5, yaw: 1.0 },
}
const SPEED_LEVELS = Object.keys(SPEEDS)   // LOW → HIGH
// Upright has one speed, inside its trained ranges (x, y ±0.5, yaw ±1.0).
const UPRIGHT_SPEED = { x: 0.3, y: 0.2, yaw: 0.7 }

const MODE_SERVICE_CMD = { 1: 'stand_down', 2: 'stand_up', 3: 'balance_stand', 4: 'upright_walk' }
const MODE_NEIGHBOURS = { 1: [2], 2: [1, 3, 4], 3: [2], 4: [2] }

// Real robot_mode string (from quadruped_main_status) -> GearShift's numeric mode id.
// Both locomotions report these (HTX: Damping -> damping, JointLock -> joint_lock,
// Velocity -> balance_stand). "locomotion" is a transitional state seen during *any*
// Unitree mode switch (not just entering walk), so it's deliberately not mapped here.
const MODE_FROM_STRING = {
  damping: 1, lie_down: 1, joint_lock: 2, balance_stand: 3,
  upright_walk: 4, upright_rising: 4, upright_tipped: 4, upright_arm_back: 4,
  shake_hand_left: 4, shake_hand_right: 4, wave_left: 4, wave_right: 4,
}

// Moves that own the robot until they hand back by themselves: no gears, kicks or driving.
const BUSY_MODES = {
  donkey_kick: 'DONKEY KICK',
  quick_buck: 'QUICK BUCK',
  side_flip_left: 'SIDE FLIP LEFT',
  side_flip_right: 'SIDE FLIP RIGHT',
  front_flip: 'FRONT FLIP',
  back_flip: 'BACK FLIP',
  recovery_stand: 'RECOVERY STAND',
  upright_coming_down: 'COMING DOWN',
}

const LOCOMOTIONS = [
  { policy: 'unitree', label: 'UNITREE', cmd: 'use_unitree' },
  { policy: 'htx', label: 'HTX', cmd: 'use_htx' },
]
const SWITCH_CONFIRM = {
  htx: {
    title: 'Switch to HTX locomotion',
    body: "Unitree's controller is released and the HTX policies take over in damping. The "
      + 'robot stays lying down; stand it up afterwards.',
  },
  unitree: {
    title: 'Switch to Unitree locomotion',
    body: "HTX stops sending motor commands and Unitree's controller takes over. The robot "
      + 'stays lying down.',
  },
}
// The controller answers the switch call at once and finishes in the background.
const SWITCH_TIMEOUT_MS = 30000

// HTX-only moves: they run and hand back by themselves. robot_mode reports cmd while one runs.
// Clips are muted loops in public/skills/, recorded from the deployed policies in MuJoCo.
// RECOVERY stays the first card, so it's always in the same place after a fall.
const SKILLS = [
  { cmd: 'recovery_stand', label: 'RECOVERY', clip: '/skills/recovery.mp4' },
  { cmd: 'donkey_kick', label: 'DONKEY KICK', clip: '/skills/donkey_kick.mp4' },
  { cmd: 'quick_buck', label: 'QUICK BUCK', clip: '/skills/quick_buck.mp4' },
]
// The four flips behind one card, after the others: tap it, then one of them (the arrows are
// R2 + that D-pad direction on the remote). Left/right are the robot's own sides.
const FLIPS = [
  { cmd: 'front_flip', place: 'front', label: '▲ FRONT', clip: '/skills/front_flip.mp4',
    title: 'FRONT FLIP: one turn forward, nose down' },
  { cmd: 'back_flip', place: 'back', label: '▼ BACK', clip: '/skills/back_flip.mp4',
    title: 'BACK FLIP: one turn backward, nose up' },
  { cmd: 'side_flip_left', place: 'left', label: '◀ LEFT', clip: '/skills/side_flip_left.mp4',
    title: "SIDE FLIP: rolls over the robot's own left side" },
  { cmd: 'side_flip_right', place: 'right', label: 'RIGHT ▶', clip: '/skills/side_flip_right.mp4',
    title: "SIDE FLIP: rolls over the robot's own right side" },
]
// The skills while upright, toggles: robot_mode reports cmd while that arm is out.
const SHAKE_HAND = {
  label: 'SHAKE HAND',
  sides: [
    { cmd: 'shake_hand_left', label: '◀ LEFT', clip: '/skills/shake_hand_left.mp4',
      title: "SHAKE HAND: tap to put the robot's own left paw out, tap again to put it back" },
    { cmd: 'shake_hand_right', label: 'RIGHT ▶', clip: '/skills/shake_hand_right.mp4',
      title: "SHAKE HAND: tap to put the robot's own right paw out, tap again to put it back" },
  ],
}
const HIGH_WAVE = {
  label: 'HIGH WAVE',
  sides: [
    { cmd: 'wave_left', label: '◀ LEFT', clip: '/skills/wave_left.mp4',
      title: "HIGH WAVE: tap to wave the robot's own left arm, tap again to put it back" },
    { cmd: 'wave_right', label: 'RIGHT ▶', clip: '/skills/wave_right.mp4',
      title: "HIGH WAVE: tap to wave the robot's own right arm, tap again to put it back" },
  ],
}
// robot_mode while that arm is out -> the status line's words and the side to tap again.
const ARM_OUT = {
  shake_hand_left: ['LEFT PAW OUT', '◀ LEFT'],
  shake_hand_right: ['RIGHT PAW OUT', 'RIGHT ▶'],
  wave_left: ['WAVING THE LEFT ARM', '◀ LEFT'],
  wave_right: ['WAVING THE RIGHT ARM', 'RIGHT ▶'],
}

// /lf/lowstate on the robot; ?lowstate=/lowstate in Sim2Sim, which has no lf/ topics.
const LOWSTATE_TOPIC = new URLSearchParams(window.location.search).get('lowstate') || '/lf/lowstate'

// goaway_odometry's status (JSON, 1 Hz). After a skill FAST_LIO restarts only once the robot
// stands still; until then odom runs on the legs.
const ODOM_STATUS_TOPIC = '/goaway_odometry/status'

// True while the safety teleop works (goaway_quadruped_controller holds an assisted_teleop
// goal, i.e. nav2 is up). Latched, 1 Hz heartbeat.
const SAFETY_READY_TOPIC = '/goaway_quadruped_controller/safety_ready'

// ROS convention: +x = forward, +y = left
// JoystickDisplay: x prop = forward/back, y prop = left/right
const JOY_HINTS = { up: '↑', down: '↓', left: '←', right: '→' }

// motor_state[leg * 3 + joint] — matches kJointNames in goaway_quadruped_controller.cpp
const LEGS = ['FR', 'FL', 'RR', 'RL']
const JOINTS = ['Hip', 'Thigh', 'Calf']

// Placeholder thresholds (tmms's) — tune once the Go2's joint motor limits are known.
function jointTempColor(c) {
  if (c < 50) return '#22C55E'
  if (c < 70) return '#F59E0B'
  return '#EF4444'
}

const labelStyle = {
  fontFamily: 'var(--font-mono)',
  fontSize: 10,
  color: 'var(--text-dim)',
  letterSpacing: '0.06em',
  marginBottom: 4,
}

const bannerStyle = (color) => ({
  fontFamily: 'var(--font-mono)',
  fontSize: 11,
  color,
  padding: '4px 12px',
  borderBottom: '1px solid var(--border)',
})

// SAFETY switch (same look as the locomotion switch, smaller) and its hint, a strip across the
// driving row. While a command is held back after an obstacle, the alert takes the hint's place.
const SAFETY_HINT = 'Struggling to walk on non-flat ground (ramps, stairs)? Turn safety off.'
const SW_W = 44
const SW_H = 22
const SW_KNOB = 16

function SafetyBar({ off, onToggle, obstacle }) {
  return (
    <div className="flex items-center justify-center gap-3" style={{ minWidth: 0, flexShrink: 0 }}>
      <span style={{ ...labelStyle, marginBottom: 0 }}>SAFETY</span>
      <button
        role="switch"
        aria-label="Safety"
        aria-checked={!off}
        onClick={onToggle}
        title={off ? 'Turn on: stop before obstacles the L1 sees (not behind)'
          : 'Turn off: drive without the obstacle check'}
        className="flex items-center gap-2"
        style={{ background: 'transparent', border: 'none', padding: 0, cursor: 'pointer', flexShrink: 0 }}
      >
        <span
          style={{
            position: 'relative',
            width: SW_W,
            height: SW_H,
            borderRadius: SW_H / 2,
            border: `1px solid ${off ? '#EF4444' : 'var(--accent-bright)'}`,
            background: off ? 'rgba(239, 68, 68, 0.15)' : 'var(--accent)',
            transition: 'all 0.2s',
          }}
        >
          <span
            style={{
              position: 'absolute',
              top: (SW_H - SW_KNOB) / 2 - 1,
              left: off ? 2 : SW_W - SW_KNOB - 4,
              width: SW_KNOB,
              height: SW_KNOB,
              borderRadius: '50%',
              background: off ? '#F87171' : '#E5E7EB',
              transition: 'left 0.2s, background 0.2s',
            }}
          />
        </span>
        <span
          style={{
            width: 26,
            textAlign: 'left',
            fontFamily: 'var(--font-mono)',
            fontSize: 12,
            letterSpacing: '0.08em',
            color: off ? '#F87171' : 'var(--text-h)',
            fontWeight: off ? 700 : 400,
          }}
        >
          {off ? 'OFF' : 'ON'}
        </span>
      </button>
      {obstacle ? (
        <span className="flash-warn" style={{ fontFamily: 'var(--font-mono)', fontSize: 11, color: '#EF4444', minWidth: 0 }}>
          ⚠ OBSTACLE AHEAD — LET GO TO DRIVE AGAIN
        </span>
      ) : (
        <span style={{ fontFamily: 'var(--font-mono)', fontSize: 10, lineHeight: 1.4, color: 'var(--text-dim)', minWidth: 0 }}>
          {SAFETY_HINT}
        </span>
      )}
    </div>
  )
}

// status / statusActive: quadruped_main_status, subscribed once in App.
// blocked / onPause: while unpaused, navigation owns the robot.
// showSkills false (Mapping page): no HTX skill row, the joystick gets the room.
export function QuadrupedWidget({ heldKeys, status, statusActive, blocked, onPause, showSkills = true }) {
  const [speedLevel, setSpeedLevel] = useState('LOW')
  const [modalOpen, setModalOpen]   = useState(false)
  const [pendingMode, setPendingMode] = useState(null)
  const [pendingPolicy, setPendingPolicy] = useState(null)
  const [switching, setSwitching] = useState(null)   // { target, since } while a switch runs
  const [serviceStatus, setServiceStatus] = useState({ loading: false, lastResult: null })
  const [joyDisplay, setJoyDisplay] = useState({ x: 0, y: 0, yaw: 0 })

  // isMovingRef doubles as edge-detection (read as "was moving" before each update) and as
  // the live state the 4Hz repeat-publish interval below checks.
  const isMovingRef = useRef(false)
  const desiredCmdRef = useRef({ lx: 0, ly: 0, az: 0 })

  // Throttled: lowstate is fast and the temps only need a slow refresh.
  const { active: lowStateActive, lastMsg: lowStateMsg } = useTopicActivity(
    LOWSTATE_TOPIC, 'unitree_go/LowState', 1500, 500
  )

  const { active: odomActive, lastMsg: odomMsg } = useTopicActivity(
    ODOM_STATUS_TOPIC, 'std_msgs/String', 2500
  )
  const odom = useMemo(() => {
    try { return JSON.parse(odomMsg?.data) } catch { return null }
  }, [odomMsg])
  const lioWaiting = odomActive && odom?.fastlio === 'waiting'
  const lioSettling = odomActive && odom?.fastlio === 'settling'

  const rawMode = statusActive ? status?.robot_mode : undefined
  const policy = statusActive ? status?.locomotion_policy : undefined
  const isHtx = policy === 'htx'
  const busyLabel = BUSY_MODES[rawMode]
  const htxOff = isHtx && rawMode === 'off'

  // Ground truth from the robot, not an optimistic local guess. "locomotion" and the busy
  // moves carry no gear, so hold onto the last resolved one instead of re-mapping it.
  const [resolvedMode, setResolvedMode] = useState(undefined)
  useEffect(() => {
    if (!statusActive) return
    const raw = status?.robot_mode
    if (raw === 'locomotion' || BUSY_MODES[raw]) return
    setResolvedMode(MODE_FROM_STRING[raw])
  }, [statusActive, status])

  const actualMode = statusActive ? resolvedMode : undefined
  const modeFault  = statusActive && actualMode === undefined && !busyLabel && !htxOff
  const isUpright = actualMode === 4
  const armOut = ARM_OUT[rawMode]
  const armBack = rawMode === 'upright_arm_back'
  const tipped = rawMode === 'upright_tipped'
  const rising = tipped || rawMode === 'upright_rising'
  // HTX walk without a fresh terrain scan: the policy assumes flat ground.
  const scanLost = statusActive && isHtx && !!status?.height_scan_lost
  // Joystick live: WALK, or UPRIGHT WALK with both arms in (rising too, as the robot allows).
  const isWalkMode = (actualMode === 3 || (isUpright && !armOut && !armBack)) && !busyLabel && !switching
  const speed = isUpright ? UPRIGHT_SPEED : SPEEDS[speedLevel]

  const isWalkModeRef = useRef(isWalkMode)
  useEffect(() => { isWalkModeRef.current = isWalkMode }, [isWalkMode])

  // Safety teleop: unless the operator turns it off, commands go through nav2's assisted_teleop,
  // which stops the robot when the next second of motion would hit the local costmap.
  // Always starts on: a reload or an unpause turns it back on. Upright walk skips it (its odom
  // is frozen and the L1 points at the ceiling).
  const [safetyOff, setSafetyOff] = useState(false)
  const safetyOffRef = useRef(false)
  const isUprightRef = useRef(isUpright)
  const { active: safetyReadyActive, lastMsg: safetyReadyMsg } = useTopicActivity(
    SAFETY_READY_TOPIC, 'std_msgs/Bool', 2500
  )
  const safetyReady = safetyReadyActive && !!safetyReadyMsg?.data

  // Set while a held command must be dropped until every input is released: 'obstacle' after
  // /teleop_obstacle, 'toggle' after the safety route changed mid-hold.
  const holdRef = useRef(null)
  const [obstacleHold, setObstacleHold] = useState(false)

  // The joystick pad grows with the middle section; its label and readout take ~60px, the
  // SAFETY row under it (not in upright walk) the rest.
  const padAreaRef = useRef(null)
  const safetyRowRef = useRef(null)
  const [padSize, setPadSize] = useState(200)
  const [legendFits, setLegendFits] = useState(true)
  useEffect(() => {
    const el = padAreaRef.current
    if (!el) return
    const ro = new ResizeObserver(([entry]) => {
      const { width } = entry.contentRect
      const safety = safetyRowRef.current ? safetyRowRef.current.offsetHeight + 12 : 0   // + gap-3
      const height = entry.contentRect.height - safety
      // The height map legend only shows when the joystick keeps its full size.
      const legend = LEGEND_W + 24
      const fits = width - 190 - legend >= Math.min(260, height - 60)
      setLegendFits(fits)
      setPadSize(Math.round(Math.max(80, Math.min(260, height - 60, width - 190 - (fits ? legend : 0)))))
    })
    ro.observe(el)
    return () => ro.disconnect()
  }, [isUpright])   // re-measure when the SAFETY row comes and goes

  // A switch is done when the controller reports the new policy (it keeps the old one until then).
  useEffect(() => {
    if (!switching) return
    if (policy === switching.target) {
      setSwitching(null)
      setServiceStatus({ loading: false, lastResult: `Locomotion: ${switching.target.toUpperCase()}` })
      return
    }
    const left = SWITCH_TIMEOUT_MS - (Date.now() - switching.since)
    const id = setTimeout(() => {
      setSwitching(null)
      setServiceStatus({ loading: false, lastResult: 'Switch did not complete (see the controller log)' })
    }, Math.max(left, 0))
    return () => clearTimeout(id)
  }, [switching, policy])

  // A tip puts the arm back by itself: the header says so until the next command.
  useEffect(() => {
    if (tipped) setServiceStatus({ loading: false, lastResult: 'Arm put back: the robot tipped' })
  }, [tipped])

  // Tracks the last cmd_vel the UI actually published, and on which route, so we can tell
  // whether a zero needs to be sent when walk mode is lost or a hold starts.
  const lastCmdRef = useRef({ lx: 0, ly: 0, az: 0, safe: true })
  const sendCmdVel = useCallback((lx, ly, az) => {
    const safe = !safetyOffRef.current && !isUprightRef.current
    publishQuadrupedCmdVel(lx, ly, az, safe)
    lastCmdRef.current = { lx, ly, az, safe }
  }, [])

  // Zero on the route that last carried motion, so a route change can't strand it there.
  const stopLast = useCallback(() => {
    const { lx, ly, az, safe } = lastCmdRef.current
    if (lx === 0 && ly === 0 && az === 0) return
    publishQuadrupedCmdVel(0, 0, 0, safe)
    lastCmdRef.current = { lx: 0, ly: 0, az: 0, safe }
  }, [])

  const hold = useCallback((reason) => {
    stopLast()
    holdRef.current = reason
    setObstacleHold(reason === 'obstacle')
  }, [stopLast])

  // True while held, so the caller sends nothing; letting go of every input ends the hold.
  const heldBack = useCallback((lx, ly, az) => {
    if (!holdRef.current) return false
    if (lx === 0 && ly === 0 && az === 0) {
      holdRef.current = null
      setObstacleHold(false)
    }
    return true
  }, [])

  // A command held across a route change must not carry into the other route: fresh press needed.
  const setSafety = useCallback((off) => {
    if (off === safetyOffRef.current) return
    if (isMovingRef.current) hold('toggle')
    safetyOffRef.current = off
    setSafetyOff(off)
  }, [hold])
  useEffect(() => {
    if (isUprightRef.current === isUpright) return
    if (isMovingRef.current) hold('toggle')
    isUprightRef.current = isUpright
  }, [isUpright, hold])

  // assisted_teleop flags a predicted collision; it has already zeroed its own output.
  useEffect(() => subscribe('/teleop_obstacle', 'std_msgs/Bool', (msg) => {
    if (msg.data && !safetyOffRef.current && !isUprightRef.current && isMovingRef.current
      && !holdRef.current) hold('obstacle')
  }), [hold])

  // Every teleop session starts safe: handing the robot to navigation turns safety back on.
  const wasPausedRef = useRef(undefined)
  useEffect(() => {
    if (!statusActive) return
    const paused = status?.is_paused
    if (wasPausedRef.current === true && paused === false) setSafety(false)
    wasPausedRef.current = paused
  }, [statusActive, status, setSafety])

  // Safety net: the instant walk mode is lost (known non-walk mode, a kick, a switch, or
  // unrecognized/undefined), stop any residual motion instead of leaving the last command.
  const wasWalkModeRef = useRef(isWalkMode)
  useEffect(() => {
    if (wasWalkModeRef.current && !isWalkMode) stopLast()
    wasWalkModeRef.current = isWalkMode
  }, [isWalkMode, stopLast])

  // Keyboard → publish quadruped velocity
  useEffect(() => {
    const shift = heldKeys.has('ShiftLeft') || heldKeys.has('ShiftRight')

    let lx = 0, ly = 0, az = 0

    if (heldKeys.has('ArrowUp'))   lx += speed.x
    if (heldKeys.has('ArrowDown')) lx -= speed.x

    if (!shift) {
      if (heldKeys.has('ArrowLeft'))  ly += speed.y
      if (heldKeys.has('ArrowRight')) ly -= speed.y
    } else {
      if (heldKeys.has('ArrowLeft'))  az += speed.yaw
      if (heldKeys.has('ArrowRight')) az -= speed.yaw
    }

    const isMoving = lx !== 0 || ly !== 0 || az !== 0
    const wasMoving = isMovingRef.current
    desiredCmdRef.current = { lx, ly, az }
    isMovingRef.current = isMoving
    const held = heldBack(lx, ly, az)

    if (isWalkMode && !held) {
      if (isMoving || wasMoving) {
        sendCmdVel(lx, ly, az)
      }
    }

    setJoyDisplay(isWalkMode ? { x: lx, y: ly, yaw: az } : { x: 0, y: 0, yaw: 0 })
  }, [heldKeys, isWalkMode, speed, sendCmdVel, heldBack])

  // < / > (the comma and period keys, with or without Shift) step the speed one level per press.
  const prevKeysRef = useRef(heldKeys)
  useEffect(() => {
    const pressed = (code) => heldKeys.has(code) && !prevKeysRef.current.has(code)
    const step = (d) => setSpeedLevel((l) =>
      SPEED_LEVELS[Math.min(Math.max(SPEED_LEVELS.indexOf(l) + d, 0), SPEED_LEVELS.length - 1)])
    if (pressed('Period')) step(1)
    if (pressed('Comma')) step(-1)
    prevKeysRef.current = heldKeys
  }, [heldKeys])

  // Repeat-publish at 4Hz while a key is held or the joystick is pinned at a nonzero
  // deflection — otherwise the controller has no way to tell the UI is still connected
  // between transitions, and its patience timeout would (correctly) stop the robot.
  useEffect(() => {
    const id = setInterval(() => {
      if (!isWalkModeRef.current || !isMovingRef.current) return
      if (holdRef.current) return
      const { lx, ly, az } = desiredCmdRef.current
      sendCmdVel(lx, ly, az)
    }, 250)
    return () => clearInterval(id)
  }, [sendCmdVel])

  // Drag callbacks — publish directly, independent of keyboard effect
  const handleJoyDrag = useCallback((x, y) => {
    desiredCmdRef.current = { ...desiredCmdRef.current, lx: x, ly: y }
    const { lx, ly, az } = desiredCmdRef.current
    isMovingRef.current = lx !== 0 || ly !== 0 || az !== 0
    if (!heldBack(lx, ly, az) && isWalkModeRef.current) sendCmdVel(lx, ly, az)
    setJoyDisplay(prev => ({ ...prev, x, y }))
  }, [sendCmdVel, heldBack])

  const handleYawDrag = useCallback((v) => {
    desiredCmdRef.current = { ...desiredCmdRef.current, az: v }
    const { lx, ly, az } = desiredCmdRef.current
    isMovingRef.current = lx !== 0 || ly !== 0 || az !== 0
    if (!heldBack(lx, ly, az) && isWalkModeRef.current) sendCmdVel(lx, ly, az)
    setJoyDisplay(prev => ({ ...prev, yaw: v }))
  }, [sendCmdVel, heldBack])

  function runCmd(cmd, onDone) {
    setServiceStatus({ loading: true, lastResult: null })
    callService(
      cmd,
      (res) => {
        setServiceStatus({ loading: false, lastResult: res.message ?? 'OK' })
        onDone?.(res)
      },
      (err) => setServiceStatus({ loading: false, lastResult: `Error: ${err}` })
    )
  }

  const controlsLocked = serviceStatus.loading || !!switching || !!busyLabel || htxOff

  function requestModeChange(newMode) {
    // Skip the adjacency check when the current real mode isn't known —
    // there's nothing safe to validate against, so let the operator try.
    if (actualMode !== undefined && !MODE_NEIGHBOURS[actualMode].includes(newMode)) return
    setPendingMode(newMode)
    setModalOpen(true)
  }

  // Switching only from SIT: Unitree lying down or damping, HTX Damping.
  const canSwitch = statusActive && !!policy && !controlsLocked
    && (isHtx ? rawMode === 'damping' : rawMode === 'lie_down' || rawMode === 'damping')
  const switchHint = !statusActive || !policy ? 'Locomotion unknown (no status yet)'
    : switching ? `Switching to ${switching.target.toUpperCase()}…`
    : !canSwitch ? 'Switch only in SIT'
    : null

  function requestSwitch(target) {
    if (target === policy || !canSwitch) return
    setPendingPolicy(target)
    setModalOpen(true)
  }

  function confirmModal() {
    setModalOpen(false)
    if (pendingPolicy) {
      const target = pendingPolicy
      setPendingPolicy(null)
      const { cmd } = LOCOMOTIONS.find((l) => l.policy === target)
      runCmd(cmd, (res) => {
        if (res.success) setSwitching({ target, since: Date.now() })
      })
      return
    }
    runCmd(isUpright ? 'come_down' : MODE_SERVICE_CMD[pendingMode])
    setPendingMode(null)
  }

  const { x: joystickX, y: joystickY, yaw: yawVal } = joyDisplay
  const driving = joystickX !== 0 || joystickY !== 0 || yawVal !== 0
  const kicksEnabled = isHtx && (actualMode === 2 || actualMode === 3) && !controlsLocked && !driving
  // HTX's Recovery is reachable from Damping, JointLock and Velocity (not Off or LieDown).
  const recoveryEnabled = isHtx && ['damping', 'joint_lock', 'balance_stand'].includes(rawMode)
    && !controlsLocked && !driving
  const skillState = {
    donkey_kick: { enabled: kicksEnabled, hint: 'Stand still in STAND LOCK or WALK first' },
    quick_buck: { enabled: kicksEnabled, hint: 'Stand still in STAND LOCK or WALK first' },
    side_flip_left: { enabled: kicksEnabled, hint: 'Stand still in STAND LOCK or WALK first' },
    side_flip_right: { enabled: kicksEnabled, hint: 'Stand still in STAND LOCK or WALK first' },
    front_flip: { enabled: kicksEnabled, hint: 'Stand still in STAND LOCK or WALK first' },
    back_flip: { enabled: kicksEnabled, hint: 'Stand still in STAND LOCK or WALK first' },
    recovery_stand: {
      enabled: recoveryEnabled,
      hint: 'Only from SIT (damping), STAND LOCK or WALK, not while driving',
    },
  }
  // One arm at a time; the one out is tapped again to put it back.
  const armHint = armOut ? 'Put the other arm back first'
    : armBack ? 'Wait until the arm is back in'
    : rising ? 'Wait until it is upright'
    : 'Stop driving first'
  for (const { cmd } of [...SHAKE_HAND.sides, ...HIGH_WAVE.sides]) {
    skillState[cmd] = {
      enabled: isUpright && !controlsLocked && (armOut ? rawMode === cmd : !armBack && !rising && !driving),
      hint: armHint,
    }
  }
  const armStatus = armOut ? `${armOut[0]}: tap ${armOut[1]} again to put it back.`
    : armBack ? 'Putting the arm back…'
    : tipped ? 'Tipped: the arm was put back. Standing up again…'
    : rising ? 'Standing up… the arms work once it is upright.'
    : 'Tap a side to put that arm out, tap it again to put it back. One arm at a time; '
      + 'the robot stands still with an arm out.'
  const renderSkill = ({ cmd, label, clip }) => {
    const { enabled, hint } = skillState[cmd]
    return (
      <SkillCard
        key={cmd}
        label={label}
        clip={clip}
        enabled={enabled}
        running={rawMode === cmd}
        title={enabled ? label : hint}
        onClick={() => runCmd(cmd)}
      />
    )
  }

  const renderFlips = () => (
    <FlipMenuCard
      label="FLIP"
      hint="Stand still in STAND LOCK or WALK first"
      options={FLIPS.map(({ cmd, place, label, clip, title }) => ({
        place,
        label,
        name: BUSY_MODES[cmd],
        clip,
        enabled: skillState[cmd].enabled,
        running: rawMode === cmd,
        title,
        onClick: () => runCmd(cmd),
      }))}
    />
  )

  const renderPair = ({ label, sides }) => (
    <SkillPairCard
      label={label}
      sides={sides.map(({ cmd, label: sideLabel, clip, title }) => {
        const { enabled, hint } = skillState[cmd]
        return {
          label: sideLabel,
          clip,
          enabled,
          running: rawMode === cmd,
          title: enabled ? title : hint,
          onClick: () => runCmd(cmd),
        }
      })}
    />
  )

  return (
    <>
      <WarningModal
        open={modalOpen}
        fromMode={actualMode}
        toMode={pendingMode}
        heading={pendingPolicy ? 'LOCOMOTION CHANGE WARNING' : undefined}
        title={pendingPolicy ? SWITCH_CONFIRM[pendingPolicy].title : undefined}
        body={pendingPolicy ? SWITCH_CONFIRM[pendingPolicy].body : undefined}
        onConfirm={confirmModal}
        onCancel={() => { setModalOpen(false); setPendingMode(null); setPendingPolicy(null) }}
      />

      <div className="panel flex flex-col h-full" style={{ overflow: 'hidden', position: 'relative' }}>
        {/* Title bar */}
        <div className="panel-header">
          <span>QUADRUPED CONTROL</span>
          <div className="flex items-center gap-2" style={{ overflow: 'hidden', minWidth: 0 }}>
            {serviceStatus.loading && (
              <span style={{ color: 'var(--accent-blue)', fontSize: 10, flexShrink: 0 }}>sending…</span>
            )}
            {serviceStatus.lastResult && !serviceStatus.loading && (
              <span style={{ color: 'var(--text-dim)', fontSize: 10, maxWidth: 360, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap', minWidth: 0 }}>
                {serviceStatus.lastResult}
              </span>
            )}
            <span
              style={{
                width: 6, height: 6, borderRadius: '50%', display: 'inline-block',
                flexShrink: 0,
                background: isWalkMode ? '#22C55E' : 'var(--border)',
              }}
              title={actualMode !== undefined ? `Mode ${actualMode}` : 'Mode unknown'}
            />
          </div>
        </div>

        {(modeFault || !statusActive) && (
          <div className="flash-warn" style={bannerStyle('#EF4444')}>
            {modeFault
              ? '⚠ UNKNOWN ROBOT MODE — MOVEMENT DISABLED'
              : '⚠ NO STATUS FEED — MOVEMENT DISABLED'}
          </div>
        )}

        {htxOff && (
          <div className="flash-warn" style={bannerStyle('#EF4444')}>
            ⚠ HTX NOT DRIVING — PRESS E-STOP (DAMP)
          </div>
        )}

        {safetyOff && !isUpright && !blocked && (
          <div className="flash-warn" style={bannerStyle('#EF4444')}>
            ⚠ SAFETY OFF — NO OBSTACLE CHECK
          </div>
        )}

        {!safetyOff && !safetyReady && !isUpright && !blocked && statusActive && (
          <div style={bannerStyle('#F59E0B')}>
            ⚠ SAFETY NOT READY — NAV2 NOT RUNNING, TURN SAFETY OFF TO DRIVE
          </div>
        )}

        {busyLabel && (
          <div style={bannerStyle('var(--accent-blue)')}>
            ● {busyLabel} IN PROGRESS — CONTROLS LOCKED UNTIL IT HANDS BACK
          </div>
        )}

        {tipped && (
          <div style={bannerStyle('#F59E0B')}>
            ⚠ TIPPED — ARM PUT BACK, STANDING UP AGAIN
          </div>
        )}

        {scanLost && (
          <div style={bannerStyle('#F59E0B')}>
            ⚠ HEIGHT SCAN LOST — WALKING AS IF ON FLAT GROUND, KEEP OFF STAIRS
          </div>
        )}

        {lioWaiting && (
          <div style={bannerStyle('#F59E0B')}>
            {odom.source === 'fused'
              ? '⚠ ODOM ON THE LEGS ONLY — STOP A MOMENT TO RESTART FAST_LIO'
              : '⚠ FAST_LIO OFF — STOP A MOMENT TO RESTART IT'}
          </div>
        )}

        {lioSettling && (
          <div style={bannerStyle('var(--accent-blue)')}>
            ⟳ FAST_LIO STARTING — HOLD STILL
          </div>
        )}

        {switching && (
          <div style={bannerStyle('var(--accent-blue)')}>
            ⟳ SWITCHING TO {switching.target.toUpperCase()} LOCOMOTION…
          </div>
        )}

        {/* Top and middle: everything the blocked overlay covers. Scoped to the control
            surfaces rather than the whole panel so the joint temps stay readable while nav drives. */}
        <div className="flex flex-col flex-1 min-h-0" style={{ position: 'relative' }}>
          {/* Blocked overlay — shown while navigation owns the robot.
              This is an AFFORDANCE, not a safety mechanism: goaway_quadruped_controller
              already refuses the UI's cmd_vel and its commands while unpaused. */}
          {blocked && (
            <div
              style={{
                position: 'absolute',
                inset: 0,
                zIndex: 10,
                background: 'var(--panel-bg)',
                opacity: 0.97,
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center',
                justifyContent: 'center',
                gap: 10,
                padding: 16,
                textAlign: 'center',
              }}
            >
              <div style={{ fontSize: 13, fontWeight: 600 }}>Teleop disabled</div>
              <div style={{ fontSize: 11, opacity: 0.7, lineHeight: 1.6, maxWidth: 300 }}>
                Navigation currently owns the robot. Pause it to take manual control —
                navigation's velocity is dropped while paused.
              </div>
              <button
                onClick={onPause}
                className="btn-icon"
                style={{ padding: '8px 20px', fontSize: 12 }}
              >
                ⏸ Pause Robot
              </button>
            </div>
          )}

          {/* Top: robot mode, locomotion, HTX skills */}
          <div
            className="flex flex-col gap-3 p-3 flex-shrink-0"
            style={{ borderBottom: '1px solid var(--border)' }}
          >
            <GearShift
              mode={actualMode}
              onModeChange={requestModeChange}
              disabled={controlsLocked}
              uprightAvailable={isHtx}
            />

            {/* Locomotion: Unitree's own or the HTX policies */}
            <div className="flex flex-col gap-1">
              <span style={labelStyle}>LOCOMOTION</span>
              <div className="flex items-center gap-3" style={{ minWidth: 0 }}>
                <LocomotionToggle
                  options={LOCOMOTIONS}
                  value={policy}
                  target={switching?.target}
                  enabled={canSwitch}
                  onRequest={requestSwitch}
                />
                {switchHint && (
                  <span style={{ fontFamily: 'var(--font-mono)', fontSize: 10, color: 'var(--text-dim)', minWidth: 0 }}>
                    {switchHint}
                  </span>
                )}
              </div>
            </div>

            {isHtx && showSkills && (
              <div className="flex flex-col gap-1">
                <span style={labelStyle}>HTX SKILLS</span>
                {isUpright ? (
                  <div className="flex gap-2">
                    {renderPair(SHAKE_HAND)}
                    {renderPair(HIGH_WAVE)}
                    {/* Keeps the cards at their usual size, so the joystick doesn't shrink. */}
                    <span
                      style={{
                        flex: 2.75,
                        minWidth: 0,
                        alignSelf: 'center',
                        fontFamily: 'var(--font-mono)',
                        fontSize: 10,
                        lineHeight: 1.6,
                        color: tipped ? '#F59E0B' : armOut ? '#E5E7EB' : 'var(--text-dim)',
                      }}
                    >
                      {armStatus}
                    </span>
                  </div>
                ) : (
                  <div className="flex gap-2">
                    {SKILLS.map(renderSkill)}
                    {renderFlips()}
                  </div>
                )}
              </div>
            )}
          </div>

          {/* Middle: driving — SPEED slider, TRANSLATION joystick + ROTATION knob, key hints */}
          <div className="flex flex-col flex-1 min-h-0 gap-2 p-3">
            {/* The controls, and SAFETY as their second row, centred together. Short of room
                only the controls row shrinks (and clips): SAFETY always stays visible. */}
            <div
              ref={padAreaRef}
              className="flex flex-col flex-1 min-h-0 items-center justify-center gap-3"
            >
            <div className="flex items-center justify-center gap-6" style={{ minHeight: 0, overflow: 'hidden' }}>
              {!isUpright && legendFits && <HeightMapLegend />}

              {/* Extra gap: speed is a setting, not one of the driving controls */}
              {!isUpright && (
                <div style={{ marginRight: 16 }}>
                  <SpeedSelector
                    speed={speedLevel}
                    onChange={setSpeedLevel}
                    trackLen={padSize}
                    limits={SPEEDS}
                  />
                </div>
              )}

              <JoystickDisplay
                x={joystickX}
                y={joystickY}
                label="TRANSLATION"
                hints={JOY_HINTS}
                size={padSize}
                maxX={speed.x}
                maxY={speed.y}
                onChange={isWalkMode ? handleJoyDrag : undefined}
              />

              <AxisKnob
                label="ROTATION"
                value={yawVal}
                orientation="v"
                trackLen={padSize}
                maxValue={speed.yaw}
                onChange={isWalkMode ? handleYawDrag : undefined}
              />
            </div>

              {!isUpright && (
                <div ref={safetyRowRef} style={{ maxWidth: '100%', flexShrink: 0 }}>
                  <SafetyBar off={safetyOff} onToggle={() => setSafety(!safetyOff)} obstacle={obstacleHold} />
                </div>
              )}
            </div>

            {/* Key hints */}
            <div
              className="flex items-center justify-between gap-2"
              style={{ fontFamily: 'var(--font-mono)', fontSize: 10, color: 'var(--text-dim)' }}
            >
              <span>↑↓ Forward/Back  ←→ Strafe  ⇧+←→ Yaw{!isUpright && '  < > Speed'}</span>
              <span style={{ color: isWalkMode ? '#22C55E' : '#EF4444', fontSize: 11, flexShrink: 0 }}>
                {isWalkMode
                  ? `✓ ${isUpright ? 'Upright' : 'Walk'} — joystick active`
                  : armOut || armBack ? '✗ Arm out — driving stopped' : '✗ Walk mode required'}
              </span>
            </div>
          </div>
        </div>

        {/* Bottom: 12 joint motor temperatures from lowstate */}
        <div className="p-3 flex-shrink-0" style={{ borderTop: '1px solid var(--border)' }}>
          <div
            style={{
              display: 'grid',
              gridTemplateColumns: 'auto repeat(4, 1fr)',
              columnGap: 12,
              rowGap: 2,
              fontFamily: 'var(--font-mono)',
              fontSize: 11,
            }}
          >
            <span style={{ fontSize: 10, color: 'var(--text-dim)', letterSpacing: '0.06em' }}>JOINT TEMPS</span>
            {LEGS.map((leg) => (
              <span key={leg} style={{ color: 'var(--text-dim)', textAlign: 'center' }}>{leg}</span>
            ))}
            {JOINTS.map((joint, j) => (
              <Fragment key={joint}>
                <span style={{ color: 'var(--text-dim)' }}>{joint}</span>
                {LEGS.map((leg, l) => {
                  const temp = lowStateActive ? lowStateMsg?.motor_state?.[l * 3 + j]?.temperature : undefined
                  return (
                    <span
                      key={leg}
                      style={{ textAlign: 'center', color: temp === undefined ? 'var(--text-dim)' : jointTempColor(temp) }}
                    >
                      {temp === undefined ? '--' : `${temp}°C`}
                    </span>
                  )
                })}
              </Fragment>
            ))}
          </div>
        </div>
      </div>
    </>
  )
}
