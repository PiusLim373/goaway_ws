import { Ros, Topic, Service } from 'roslib'

// https pages can't open a plain ws:// socket (browsers block it as mixed
// content), so match whatever scheme the page itself loaded over.
const WS_PROTOCOL = window.location.protocol === 'https:' ? 'wss' : 'ws'
export const WS_URL = `${WS_PROTOCOL}://${window.location.hostname}:9090`

export const ros = new Ros({ url: WS_URL })

// roslib connects exactly once, in the Ros constructor, and never retries: on a dropped socket
// it only flips isConnected and emits 'close'. Without this the singleton stays dead until the
// operator hard-refreshes.
//
// Only the socket needs rebuilding. Every Topic re-issues its own subscribe/advertise once the
// connection is back (roslib's per-Topic reconnect_on_close defaults to true), so subscribers
// and publishers throughout this module heal on their own.
//
// Fixed interval, no backoff: a rosbridge respawn is a few seconds, and a 2s retry against a
// port on the same machine costs nothing. 'error' is not handled separately — a failed connect
// emits it and then 'close', so this covers both.
let reconnectTimer = null
ros.on('close', () => {
  clearTimeout(reconnectTimer)
  reconnectTimer = setTimeout(() => ros.connect(WS_URL), 2000)
})

const CONTROLLER = '/goaway_quadruped_controller'

// Memoized publishers — one Topic instance per topic name
const _publishers = {}
function getPublisher(name, messageType) {
  if (!_publishers[name]) {
    _publishers[name] = new Topic({ ros, name, messageType })
  }
  return _publishers[name]
}

// The UI's teleop input to goaway_quadruped_controller's cmd_vel mux (used while paused).
// safe: through nav2's assisted_teleop, which stops the robot short of anything in the local
// costmap. Unsafe goes straight to the controller, unchecked.
export function publishQuadrupedCmdVel(lx, ly, az, safe) {
  const topic = safe ? '/safety_quadruped_cmd_vel_ui' : '/quadruped_cmd_vel_ui'
  getPublisher(topic, 'geometry_msgs/Twist').publish({
    linear:  { x: lx, y: ly, z: 0 },
    angular: { x: 0,  y: 0,  z: az },
  })
}

// Returns cleanup fn: call in useEffect cleanup.
// throttleMs: rosbridge sends at most one message per period (0 = every message); use it for
// fast topics such as lowstate, which would otherwise flood rosbridge's websocket.
export function subscribe(topicName, messageType, callback, throttleMs = 0) {
  const topic = new Topic({
    ros,
    name: topicName,
    messageType,
    throttle_rate: throttleMs,
    queue_length: 1,
  })
  topic.subscribe(callback)
  return () => topic.unsubscribe()
}

export function callRosService(serviceName, serviceType, request, onResult, onError, timeoutSec) {
  if (!ros.isConnected) {
    onError?.('ROS not connected')
    return
  }
  const svc = new Service({ ros, name: serviceName, serviceType })
  svc.callService(
    request,
    (result) => onResult?.(result),
    (error)  => onError?.(error),
    timeoutSec
  )
}

// goaway_quadruped_controller's ~/quadruped_cmd: damp, stand_up, stand_down, balance_stand,
// recovery_stand, donkey_kick, quick_buck, side_flip_left, side_flip_right, front_flip,
// back_flip, upright_walk, come_down, shake_hand_left, shake_hand_right, wave_left, wave_right, use_htx,
// use_unitree
export function callService(data, onResult, onError) {
  callRosService(`${CONTROLLER}/quadruped_cmd`, 'goaway_msgs/StringTrigger', { data }, onResult, onError)
}

export function setQuadrupedPaused(paused, onResult, onError) {
  callRosService(`${CONTROLLER}/quadruped_pause`, 'std_srvs/srv/SetBool',
    { data: paused }, onResult, onError)
}

// goaway_yasmin's /cancel_goal (a Trigger: it refuses during a map switch, or with nothing to
// cancel, and says why).
export function cancelNavGoal(onResult, onError) {
  callRosService('/cancel_goal', 'std_srvs/srv/Trigger', {}, onResult, onError)
}

// mapping_manager (slam_toolbox sessions). Stop saves the map first: allow it a minute.
export function startMapping(name, onResult, onError) {
  callRosService('/mapping_manager/start_mapping', 'goaway_msgs/StringTrigger', { data: name },
    onResult, onError, 30)
}

export function stopMapping(onResult, onError) {
  callRosService('/mapping_manager/stop_mapping', 'std_srvs/srv/Trigger', {}, onResult, onError, 90)
}
