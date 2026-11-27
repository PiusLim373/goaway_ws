#!/usr/bin/env python3
"""Mapping sessions with slam_toolbox (2D: /utlidar/scan + the leg odometry), after tmms's
mapping_manager (which drove FAST-LIO).

    ros2 service call /mapping_manager/start_mapping htxgrrt_msgs/srv/StringTrigger "{data: my_map}"
    ros2 service call /mapping_manager/stop_mapping std_srvs/srv/Trigger

slam_toolbox is launched idle (an unconfigured lifecycle node). start_mapping configures and
activates it; stop_mapping saves the map, then deactivates and cleans it up, so every session
starts from an empty mapper. Its live map is on /mapping/map, so map_server's /map, AMCL and the
global costmap keep the map they had.

THE MAP STORE (tmms's layout, so tmms tools and the C2 can read it)

    <maps_dir>/png/<name>.png + .yaml        2D grid for map_server / AMCL / C2 (trinary, tmms greys)
    <maps_dir>/slam/<name>.posegraph + .data slam_toolbox's pose graph, to extend or regenerate
                                             the map later (its grid is only free/occupied/unknown)
    <maps_dir>/slam/<name>.png + .yaml       the map as slam_toolbox made it (the UI's editor
                                             only rewrites png/)

AMCL keeps running during a session with its map -> odom broadcast off: slam_toolbox publishes
map -> odom instead, and two publishers would fight.

While goaway_odometry reports paused (HTX skills, UprightWalk, falls) odom is frozen and the scan
is junk, so slam_toolbox's new scans are paused too, and for resume_hold_sec after (the scan is a
1 s stack placed with the odom).

On startup (also after a respawn) a leftover session is cleaned up: slam_toolbox back to
unconfigured, AMCL's broadcast on. A launch shutdown takes slam_toolbox and AMCL down with it.
"""

import json
import os
import re
import threading
import time

import numpy as np
from PIL import Image

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from htxgrrt_msgs.msg import QuadrupedMainStatus
from htxgrrt_msgs.srv import StringTrigger
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters, SetParameters
from slam_toolbox.srv import Pause, SerializePoseGraph
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Same rule as tmms (UI, ui_backend.js, mapping_manager): also the path-traversal defence.
MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_]+$')

# tmms's greys and thresholds (ui_backend.js): map_server reads occ = (255 - pixel) / 255.
PX_OCCUPIED, PX_UNKNOWN, PX_FREE = 0, 205, 255
OCCUPIED_THRESH, FREE_THRESH = 0.65, 0.15


class MappingManagerNode(Node):

    def __init__(self):
        super().__init__('mapping_manager')

        self.declare_parameter('maps_dir', '~/.htxgrrt/maps')
        self.declare_parameter('slam_node', '/slam_toolbox')
        self.declare_parameter('amcl_node', '/amcl')   # '' leaves AMCL alone
        self.declare_parameter('map_topic', '/mapping/map')
        self.declare_parameter('odom_status_topic', '/goaway_odometry/status')
        self.declare_parameter('status_topic', '/quadruped_main_status')
        self.declare_parameter('require_paused', True)   # refuse while nav owns the robot
        self.declare_parameter('resume_hold_sec', 1.5)
        self.declare_parameter('map_wait_sec', 5.0)      # for the final map after scans stop
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('serialize_timeout_sec', 60.0)

        self._lock = threading.Lock()   # session state
        self._state = 'idle'            # idle | starting | mapping | saving
        self._map_name = ''
        self._started_at = 0.0
        self._message = ''
        self._last_saved = ''
        self._last_saved_at = 0.0
        self._map = None                # latest /mapping/map of this session
        self._map_rx = 0.0              # monotonic time it arrived
        self._odom_paused = False
        self._odom_resumed_at = 0.0
        self._scans_paused = False      # what we asked slam_toolbox for
        self._robot_paused = None       # quadruped_main_status.is_paused
        self._robot_status_rx = 0.0

        self._srv_group = MutuallyExclusiveCallbackGroup()   # start / stop never interleave
        self._aux_group = ReentrantCallbackGroup()            # clients, timers

        slam = self._param('slam_node')
        self._slam_get_state = self.create_client(
            GetState, f'{slam}/get_state', callback_group=self._aux_group)
        self._slam_change_state = self.create_client(
            ChangeState, f'{slam}/change_state', callback_group=self._aux_group)
        self._slam_get_params = self.create_client(
            GetParameters, f'{slam}/get_parameters', callback_group=self._aux_group)
        self._slam_pause = self.create_client(
            Pause, '/slam_toolbox/pause_new_measurements', callback_group=self._aux_group)
        self._slam_serialize = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map', callback_group=self._aux_group)
        amcl = self._param('amcl_node')
        self._amcl_set_params = self.create_client(
            SetParameters, f'{amcl}/set_parameters',
            callback_group=self._aux_group) if amcl else None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._status_pub = self.create_publisher(String, '~/status', latched)
        self.create_subscription(
            OccupancyGrid, self._param('map_topic'), self._map_cb, latched,
            callback_group=self._aux_group)
        self.create_subscription(
            String, self._param('odom_status_topic'), self._odom_status_cb, 10,
            callback_group=self._aux_group)
        self.create_subscription(
            QuadrupedMainStatus, self._param('status_topic'), self._robot_status_cb, 10,
            callback_group=self._aux_group)

        self.create_service(
            StringTrigger, '~/start_mapping', self._start_cb, callback_group=self._srv_group)
        self.create_service(
            Trigger, '~/stop_mapping', self._stop_cb, callback_group=self._srv_group)

        self._tick_lock = threading.Lock()   # the timer is reentrant; one tick at a time
        self.create_timer(1.0, self._tick, callback_group=self._aux_group)
        self._recovered = False
        self._publish_status()
        self.get_logger().info(f'mapping_manager ready; maps_dir={self._maps_dir()}')

    # -- helpers ---------------------------------------------------------------

    def _param(self, name):
        return self.get_parameter(name).value

    def _maps_dir(self):
        return os.path.expanduser(self._param('maps_dir'))

    def _call(self, client, request, timeout=None):
        """Blocking call from a service/timer callback (other threads deliver the reply).
        Returns the response or None."""
        timeout = float(self._param('service_timeout_sec')) if timeout is None else timeout
        if not client.wait_for_service(timeout_sec=min(timeout, 3.0)):
            return None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            future.cancel()
            return None
        return future.result()

    def _publish_status(self):
        with self._lock:
            m = self._map
            size = [round(m.info.width * m.info.resolution, 1),
                    round(m.info.height * m.info.resolution, 1)] if m is not None else None
            status = {
                'state': self._state,
                'mapping': self._state in ('starting', 'mapping', 'saving'),
                'map_name': self._map_name,
                'started_at': self._started_at,
                'scans_paused': self._scans_paused,
                'map_size': size,
                'last_saved': self._last_saved,
                'last_saved_at': self._last_saved_at,
                'message': self._message,
            }
        self._status_pub.publish(String(data=json.dumps(status)))

    # -- inputs ----------------------------------------------------------------

    def _map_cb(self, msg):
        with self._lock:
            if self._state in ('mapping', 'saving'):
                self._map = msg
                self._map_rx = time.monotonic()

    def _odom_status_cb(self, msg):
        try:
            paused = bool(json.loads(msg.data).get('paused', False))
        except ValueError:
            return
        with self._lock:
            if paused == self._odom_paused:
                return
            self._odom_paused = paused
            if not paused:
                self._odom_resumed_at = time.monotonic()
        self.get_logger().info(f'odometry {"paused" if paused else "resumed"}')

    def _robot_status_cb(self, msg):
        self._robot_paused = msg.is_paused
        self._robot_status_rx = time.monotonic()

    # -- slam_toolbox / AMCL ---------------------------------------------------

    def _slam_state(self):
        res = self._call(self._slam_get_state, GetState.Request(), timeout=3.0)
        return res.current_state.label if res is not None else None

    def _slam_transition(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        res = self._call(self._slam_change_state, req)
        return res is not None and res.success

    def _slam_to(self, target):
        """Walk slam_toolbox's lifecycle to 'active' or 'unconfigured'. Returns (ok, message)."""
        state = self._slam_state()
        if state is None:
            return False, 'slam_toolbox is not running (no get_state)'
        steps = {
            ('unconfigured', 'active'): [Transition.TRANSITION_CONFIGURE,
                                         Transition.TRANSITION_ACTIVATE],
            ('inactive', 'active'): [Transition.TRANSITION_ACTIVATE],
            ('active', 'unconfigured'): [Transition.TRANSITION_DEACTIVATE,
                                         Transition.TRANSITION_CLEANUP],
            ('inactive', 'unconfigured'): [Transition.TRANSITION_CLEANUP],
        }.get((state, target), [])
        if state != target and not steps:
            return False, f'slam_toolbox is {state}, cannot go to {target}'
        for step in steps:
            if not self._slam_transition(step):
                return False, f'slam_toolbox transition {step} failed (was {state})'
        return True, f'slam_toolbox {target}'

    def _set_scans_paused(self, paused):
        """slam_toolbox's pause is a toggle: read its parameter, toggle only if needed."""
        res = self._call(self._slam_get_params,
                         GetParameters.Request(names=['paused_new_measurements']), timeout=3.0)
        if res is None or not res.values:
            return False
        if res.values[0].bool_value != paused:
            if self._call(self._slam_pause, Pause.Request(), timeout=3.0) is None:
                return False
        with self._lock:
            self._scans_paused = paused
        return True

    def _set_amcl_tf_broadcast(self, enabled):
        """AMCL's map -> odom on/off. Best effort: no AMCL is not an error."""
        if self._amcl_set_params is None:
            return True
        req = SetParameters.Request(parameters=[
            Parameter('tf_broadcast', Parameter.Type.BOOL, enabled).to_parameter_msg()])
        res = self._call(self._amcl_set_params, req, timeout=5.0)
        state = 'on' if enabled else 'off'
        if res is None or not res.results or not res.results[0].successful:
            self.get_logger().warning(f'AMCL tf_broadcast not switched {state}')
            return False
        self.get_logger().info(f'AMCL map -> odom broadcast {state}')
        return True

    # -- map files -------------------------------------------------------------

    def _write_map(self, grid, name):
        """png/<name>.png + yaml in tmms's format, and the same pair in slam/ (the untouched
        copy; the UI's editor only rewrites png/). Returns the png path."""
        w, h = grid.info.width, grid.info.height
        data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
        data = np.flipud(data)   # image row 0 is the map's top (max y)

        tri = np.full(data.shape, PX_UNKNOWN, dtype=np.uint8)
        known = data >= 0
        tri[known & (data >= OCCUPIED_THRESH * 100)] = PX_OCCUPIED
        tri[known & (data <= FREE_THRESH * 100)] = PX_FREE

        o = grid.info.origin
        yaw = 2.0 * np.arctan2(o.orientation.z, o.orientation.w)
        origin = f'[{o.position.x:.6f}, {o.position.y:.6f}, {yaw:.6f}]'
        res = grid.info.resolution

        def save(img, path):
            tmp = path + '.tmp.png'
            Image.fromarray(img, mode='L').save(tmp)
            os.replace(tmp, path)

        def save_yaml(path, lines):
            tmp = path + '.tmp'
            with open(tmp, 'w') as f:
                f.write('\n'.join(lines) + '\n')
            os.replace(tmp, path)

        yaml = [
            f'image: {name}.png', 'mode: trinary', f'resolution: {res:.6g}', f'origin: {origin}',
            'negate: 0', f'occupied_thresh: {OCCUPIED_THRESH}', f'free_thresh: {FREE_THRESH}']
        for sub in ('slam', 'png'):   # png/ last: it is what the UI / C2 see
            d = os.path.join(self._maps_dir(), sub)
            os.makedirs(d, exist_ok=True)
            save(tri, os.path.join(d, f'{name}.png'))
            save_yaml(os.path.join(d, f'{name}.yaml'), yaml)
        return os.path.join(self._maps_dir(), 'png', f'{name}.png')

    # -- services --------------------------------------------------------------

    def _set(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self, '_' + k, v)
        self._publish_status()

    def _start_cb(self, request, response):
        name = (request.data or '').strip()
        response.success = False
        with self._lock:
            busy = self._state != 'idle'
            current = self._map_name
        if busy:
            response.message = f'a mapping session is already active ({current})'
            return response
        if not MAP_NAME_RE.match(name):
            response.message = f"invalid map name '{name}': use letters, digits and _"
            return response
        maps = self._maps_dir()
        if (os.path.exists(os.path.join(maps, 'png', f'{name}.png'))
                or os.path.exists(os.path.join(maps, 'slam', f'{name}.posegraph'))):
            response.message = f"a map named '{name}' already exists: pick another name"
            return response
        if self._param('require_paused'):
            fresh = time.monotonic() - self._robot_status_rx < 2.0
            if not fresh:
                response.message = 'no quadruped_main_status: is the controller running?'
                return response
            if not self._robot_paused:
                response.message = 'navigation owns the robot: pause it first'
                return response

        self._set(state='starting', map_name=name, started_at=time.time(), map=None,
                  message='starting slam_toolbox')
        self._set_amcl_tf_broadcast(False)
        ok, msg = self._slam_to('unconfigured')   # a fresh mapper every session
        if ok:
            ok, msg = self._slam_to('active')
        if not ok:
            self._slam_to('unconfigured')
            self._set_amcl_tf_broadcast(True)
            self._set(state='idle', map_name='', started_at=0.0, message=f'start failed: {msg}')
            response.message = f'could not start mapping: {msg}'
            return response
        with self._lock:
            self._scans_paused = False
        self._set(state='mapping', message='mapping')
        self.get_logger().info(f'mapping session "{name}" started')
        response.success = True
        response.message = f"mapping '{name}' started"
        return response

    def _stop_cb(self, _request, response):
        response.success = False
        with self._lock:
            if self._state != 'mapping':
                response.message = 'no active mapping session'
                return response
            name = self._map_name
        self._set(state='saving', message='saving the map')

        # No new scans, then the next map slam_toolbox publishes is the final one.
        paused_at = time.monotonic()
        self._set_scans_paused(True)
        end = paused_at + float(self._param('map_wait_sec'))
        while time.monotonic() < end:
            with self._lock:
                if self._map is not None and self._map_rx > paused_at + 0.05:
                    break
            time.sleep(0.1)
        with self._lock:
            grid = self._map

        errors, saved = [], []
        if grid is None:
            errors.append('no map received (did the robot move?)')
        else:
            try:
                saved.append(self._write_map(grid, name))
            except (OSError, ValueError) as exc:
                errors.append(f'could not write the map: {exc}')
            base = os.path.join(self._maps_dir(), 'slam', name)
            os.makedirs(os.path.dirname(base), exist_ok=True)
            res = self._call(self._slam_serialize, SerializePoseGraph.Request(filename=base),
                             timeout=float(self._param('serialize_timeout_sec')))
            if res is None or res.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                errors.append('pose graph not saved')

        ok, msg = self._slam_to('unconfigured')
        if not ok:
            errors.append(msg)
        self._set_amcl_tf_broadcast(True)

        if saved:
            self._set(state='idle', map_name='', started_at=0.0, last_saved=name,
                      last_saved_at=time.time(), scans_paused=False,
                      message=f'saved png/{name}.png' + (f' ({"; ".join(errors)})' if errors else ''))
        else:
            self._set(state='idle', map_name='', started_at=0.0, scans_paused=False,
                      message=f'map NOT saved: {"; ".join(errors)}')
        self.get_logger().info(f'mapping session "{name}" ended: {self._message}')
        response.success = bool(saved) and not errors
        response.message = self._message
        return response

    # -- timer -----------------------------------------------------------------

    def _tick(self):
        if not self._tick_lock.acquire(blocking=False):
            return
        try:
            self._tick_locked()
        finally:
            self._tick_lock.release()

    def _tick_locked(self):
        with self._lock:
            state = self._state
        if not self._recovered:
            if state == 'idle':
                self._recover()
                return
            self._recovered = True   # a session of ours already runs: nothing left over
        with self._lock:
            state = self._state
            odom_paused = self._odom_paused
            resumed_at = self._odom_resumed_at
            scans_paused = self._scans_paused
        if state == 'mapping':
            if self._slam_state() != 'active':
                # Died or was driven elsewhere: the map is gone.
                self._slam_to('unconfigured')
                self._set_amcl_tf_broadcast(True)
                self._set(state='idle', map_name='', started_at=0.0, scans_paused=False,
                          message='slam_toolbox stopped on its own: map NOT saved')
                self.get_logger().error('slam_toolbox is no longer active; session ended')
                return
            want = odom_paused or time.monotonic() - resumed_at < float(self._param('resume_hold_sec'))
            if want != scans_paused and self._set_scans_paused(want):
                self.get_logger().info(
                    f'slam scans {"paused (odometry paused)" if want else "resumed"}')
        self._publish_status()

    def _recover(self):
        """Once slam_toolbox answers: no session left over from a crash or respawn."""
        state = self._slam_state()
        if state is None:
            return
        if state != 'unconfigured':
            self.get_logger().warning(f'slam_toolbox was {state} at startup: cleaning up')
            self._slam_to('unconfigured')
        self._set_amcl_tf_broadcast(True)
        self._recovered = True

    def shutdown(self):
        """A session running at exit is lost (the next start's recovery cleans slam_toolbox up)."""
        with self._lock:
            active = self._state != 'idle'
        if active:
            self.get_logger().warning('shutting down with a mapping session active: map NOT saved')


def main():
    rclpy.init()
    node = MappingManagerNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
