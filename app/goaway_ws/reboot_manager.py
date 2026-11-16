#!/usr/bin/env python3
"""Host-side REST service for restarting the goaway stack from the dashboard.

Ported from tmms_ws's reboot_manager.py. Runs on the robot's host OS (not inside goaway_run)
under supervisor as [program:goaway_reboot_manager], because `supervisorctl` and `docker` are
host operations the containerised UI backend can't do. Targets, cheapest first:

  rosbridge  SIGINT rosbridge_websocket and rosapi inside goaway_run and let `ros2 launch`
             respawn them (operation.launch.py passes respawn:=true). Everything else keeps
             running.

  fastlio    SIGINT FAST_LIO and its LiDAR merger; launch respawns both. goaway_odometry carries
             odom on the legs meanwhile and links the new FAST_LIO to its pose, so odom doesn't
             jump. Refused unless the robot is still and odom isn't paused: FAST_LIO calibrates
             its IMU from its first samples, and a skill would see it start unpaused.

  goaway_ws  `supervisorctl restart quadruped:goaway_ws`: the whole ROS container, for when the
             stack is wedged. goaway_run also runs the HTX controller, which damps on exit, so
             the restart is refused while HTX holds the robot (any state but Off or Damping).

Only goaway_ws needs sudo; the others are `docker exec` alone.

Same API as tmms: GET /health, GET /status, POST /reboot/<target> -> 202, then poll /status for
the outcome; one job at a time. Binds loopback only: the UI backend proxies /api/system/* here,
and the sudo path stays off the network.
"""

import json
import logging
import os
import re
import socket
import subprocess
import threading
import time

from flask import Flask, jsonify

PORT = int(os.environ.get('GOAWAY_REBOOT_PORT', '5055'))
CONTAINER = os.environ.get('GOAWAY_CONTAINER', 'goaway_run')
SUPERVISOR_PROGRAM = os.environ.get('GOAWAY_SUPERVISOR_PROGRAM', 'quadruped:goaway_ws')
ROSBRIDGE_PORT = int(os.environ.get('GOAWAY_ROSBRIDGE_PORT', '9090'))

# supervisorctl's socket is root-only (0700) and this runs as htxgrrt, so `sudo -S` with the
# password on stdin. Overridable if the robot's password changes.
SUDO_PASSWORD = os.environ.get('GOAWAY_SUDO_PASSWORD', 'Unitree0408')

# docker exec doesn't inherit the entrypoint's exports (RMW, CycloneDDS, the overlay). It prints
# a banner, hence the redirect.
CONTAINER_ENV = 'source /home/htxgrrt/.htxgrrt/app/goaway_ws/custom.env >/dev/null 2>&1; '

# HTX states that hold no pose: stopping the controller there can't drop the robot.
SAFE_HTX_STATES = ('Off', 'Damping')

# pkill -f patterns on the installed executable paths, never a bare node name: a loose
# pattern can match the `ros2 launch` parent and take the whole stack down (see tmms).
ROSBRIDGE_PATTERN = 'lib/rosbridge_server/rosbridge_websocket'
ROSAPI_PATTERN = 'lib/rosapi/rosapi_node'
FASTLIO_PATTERN = 'lib/fast_lio/fastlio_mapping'
MERGER_PATTERN = 'lib/mapping_utils/lidar_merger_node'

GOAWAY_WS_TIMEOUT_S = 120.0
ROSBRIDGE_TIMEOUT_S = 30.0
STOP_TIMEOUT_S = 10.0           # SIGINT ignored this long: SIGKILL
NODE_RESPAWN_TIMEOUT_S = 30.0
FASTLIO_LINK_TIMEOUT_S = 60.0   # IMU init, first scans, then odometry's settle
STATE_TIMEOUT_S = 8.0     # the state topics have a 1 s heartbeat; ros2 CLI startup takes ~2 s
POLL_INTERVAL_S = 1.0
STATUS_POLL_INTERVAL_S = 3.0    # each topic read starts the ros2 CLI
PROBE_TTL_S = 2.0         # /status reuses its docker/supervisor probe this long

log = logging.getLogger('reboot_manager')

app = Flask(__name__)

# Guards _job: Flask runs threaded, and the worker thread writes it.
_lock = threading.Lock()
_job = None
_probe_cache = None


# ---------------------------------------------------------------------------------------
# subprocess helpers
# ---------------------------------------------------------------------------------------

def _run(cmd, timeout=30.0, stdin=None):
    """Run an argv list (never a shell string) and return (returncode, stdout+stderr)."""
    try:
        proc = subprocess.run(
            cmd, input=stdin, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            universal_newlines=True, timeout=timeout)
        return proc.returncode, (proc.stdout or '').strip()
    except FileNotFoundError:
        return 127, f'{cmd[0]}: not found'
    except subprocess.TimeoutExpired:
        return 124, f'timed out after {timeout:g}s: {" ".join(cmd)}'


def _sudo(args, timeout=180.0):
    """sudo -S -k: always authenticates with the password, never a cached credential."""
    return _run(['sudo', '-S', '-k', *args], timeout=timeout, stdin=SUDO_PASSWORD + '\n')


def _container_started():
    """The container's start time while it runs, else None. A new value means a real restart."""
    rc, out = _run(
        ['docker', 'inspect', '-f', '{{.State.Running}} {{.State.StartedAt}}', CONTAINER],
        timeout=10.0)
    parts = out.split()
    return parts[1] if rc == 0 and len(parts) == 2 and parts[0] == 'true' else None


def _port_listening(port, host='127.0.0.1', timeout=0.5):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _wait_for(predicate, timeout_s, interval_s=POLL_INTERVAL_S):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


def _topic_json(topic):
    """One JSON std_msgs/String from a topic inside the container, or None if no answer."""
    rc, out = _run(
        ['docker', 'exec', CONTAINER, 'bash', '-c', CONTAINER_ENV +
         f'timeout {STATE_TIMEOUT_S:g} ros2 topic echo --once {topic} std_msgs/msg/String'],
        timeout=STATE_TIMEOUT_S + 5.0)
    match = re.search(r"data: '(\{.*\})'", out)
    if rc != 0 or not match:
        return None
    try:
        return json.loads(match.group(1))
    except ValueError:
        return None


def _htx_state():
    """goaway_locomotion_node's state from /goaway_locomotion/state, or None if no answer."""
    state = _topic_json('/goaway_locomotion/state')
    return state.get('state') if state else None


def _node_pids(pattern):
    """pids inside CONTAINER whose command line matches pattern."""
    rc, out = _run(['docker', 'exec', CONTAINER, 'pgrep', '-f', pattern], timeout=10.0)
    return set(out.split()) if rc == 0 else set()   # pgrep exits 1 when nothing matches


def _stop(label, pattern, before):
    """SIGINT the node so launch respawns it; SIGKILL if it ignores that. None or an error."""
    rc, out = _run(['docker', 'exec', CONTAINER, 'pkill', '-INT', '-f', pattern], timeout=15.0)
    if rc not in (0, 1):
        return f'failed to signal {label}: {out or f"pkill exited {rc}"}'
    if not _wait_for(lambda: not (_node_pids(pattern) & before), STOP_TIMEOUT_S):
        log.warning('%s ignored SIGINT; SIGKILL', label)
        _run(['docker', 'exec', CONTAINER, 'pkill', '-KILL', '-f', pattern], timeout=15.0)
    return None


def _respawned(pattern, before):
    """A pid that wasn't there before: the old one may still be shutting down."""
    return bool(_node_pids(pattern) - before)


# ---------------------------------------------------------------------------------------
# reboot targets
# ---------------------------------------------------------------------------------------

def _reboot_rosbridge():
    """SIGINT rosbridge (and rosapi) inside the container; launch respawns them."""
    if _container_started() is None:
        return False, f'{CONTAINER} is not running: use Restart goaway_ws instead.'
    before = _node_pids(ROSBRIDGE_PATTERN)
    if not before:
        return False, f'no rosbridge_websocket process in {CONTAINER}.'

    error = _stop('rosbridge', ROSBRIDGE_PATTERN, before)
    if error:
        return False, error
    # rosapi answers the header's ros ping and has the same respawn flag; not fatal.
    rc, out = _run(
        ['docker', 'exec', CONTAINER, 'pkill', '-INT', '-f', ROSAPI_PATTERN], timeout=15.0)
    if rc not in (0, 1):
        log.warning('could not signal rosapi_node: %s', out)

    if not _wait_for(lambda: _respawned(ROSBRIDGE_PATTERN, before), NODE_RESPAWN_TIMEOUT_S):
        return False, (f'rosbridge did not come back within {NODE_RESPAWN_TIMEOUT_S:g}s: is '
                       'respawn set in operation.launch.py?')
    if not _wait_for(lambda: _port_listening(ROSBRIDGE_PORT), ROSBRIDGE_TIMEOUT_S):
        return False, f'rosbridge came back, but port {ROSBRIDGE_PORT} is not open.'
    return True, 'rosbridge restarted.'


def _reboot_fastlio():
    """SIGINT FAST_LIO and its merger; launch respawns them, odometry links the new one."""
    if _container_started() is None:
        return False, f'{CONTAINER} is not running: use Restart goaway_ws instead.'
    before = _node_pids(FASTLIO_PATTERN)
    if not before:
        return False, f'no FAST_LIO process in {CONTAINER}.'

    status = _topic_json('/goaway_odometry/status')
    if status is not None:
        if status.get('paused'):
            return False, 'odom is paused (a skill or a fall): wait until the robot is calm.'
        if not status.get('still'):
            return False, ('the robot is moving: FAST_LIO calibrates its IMU when it starts, '
                           'so stop it (standing or lying) first.')
    # No answer is let through, as for goaway_ws; then only the respawn can be checked.
    links = status.get('links', 0) if status else None
    log.info('odom status before FAST_LIO restart: %s', status or 'no answer')

    merger = _node_pids(MERGER_PATTERN)
    for label, pattern, pids in (('FAST_LIO', FASTLIO_PATTERN, before),
                                 ('the lidar merger', MERGER_PATTERN, merger)):
        if pids:
            error = _stop(label, pattern, pids)
            if error:
                return False, error
    if not _wait_for(lambda: _respawned(FASTLIO_PATTERN, before), NODE_RESPAWN_TIMEOUT_S):
        return False, f'FAST_LIO did not come back within {NODE_RESPAWN_TIMEOUT_S:g}s.'
    if merger and not _wait_for(lambda: _respawned(MERGER_PATTERN, merger),
                                NODE_RESPAWN_TIMEOUT_S):
        return False, f'the lidar merger did not come back within {NODE_RESPAWN_TIMEOUT_S:g}s.'
    if links is None:
        return True, 'FAST_LIO restarted (no odom status to confirm the link).'

    def linked():
        now = _topic_json('/goaway_odometry/status')
        return now is not None and now.get('fastlio') == 'ready' and now.get('links', 0) > links

    if not _wait_for(linked, FASTLIO_LINK_TIMEOUT_S, STATUS_POLL_INTERVAL_S):
        return False, ('FAST_LIO restarted, but odom has not linked it yet: check '
                       '/goaway_odometry/status.')
    return True, 'FAST_LIO restarted and linked to odom.'


def _reboot_goaway_ws():
    """supervisorctl restart of the whole ROS container, then wait for a new one."""
    before = _container_started()
    if before is not None:
        state = _htx_state()
        if state is not None and state not in SAFE_HTX_STATES:
            return False, (f'HTX is in {state} and would drop the robot: lie it down '
                           '(Damping) first.')
        # No answer is let through: a stack too wedged to report is what this restart is for.
        log.info('HTX state before restart: %s', state or 'no answer')

    rc, out = _sudo(['/usr/bin/supervisorctl', 'restart', SUPERVISOR_PROGRAM])
    if rc != 0:
        return False, f'supervisorctl restart failed: {out or f"exit {rc}"}'

    # The old container still running doesn't count: wait for a new start time.
    if not _wait_for(lambda: _container_started() not in (None, before), GOAWAY_WS_TIMEOUT_S):
        return False, f'{CONTAINER} did not restart within {GOAWAY_WS_TIMEOUT_S:g}s.'

    # rosbridge is what the dashboard talks to, so its port is the readiness signal.
    if not _wait_for(lambda: _port_listening(ROSBRIDGE_PORT), ROSBRIDGE_TIMEOUT_S):
        return True, f'{CONTAINER} restarted, but rosbridge is not up yet.'
    return True, 'goaway_ws restarted.'


# Menu order, cheapest first. Mirrored in ui_backend.js's REBOOT_TARGETS and the dashboard's
# REBOOT_ITEMS / REBOOT_CONFIRM: a name missing from any of them fails the button.
TARGETS = {
    'rosbridge': _reboot_rosbridge,
    'fastlio': _reboot_fastlio,
    'goaway_ws': _reboot_goaway_ws,
}


# ---------------------------------------------------------------------------------------
# job runner
# ---------------------------------------------------------------------------------------

def _worker(target):
    try:
        ok, message = TARGETS[target]()
    except Exception as err:  # noqa: BLE001 - a crashed worker must not stay "running"
        log.exception('%s reboot raised', target)
        ok, message = False, f'internal error: {err}'
    log.info('%s reboot finished: ok=%s %s', target, ok, message)
    with _lock:
        _job.update(state='done' if ok else 'failed', message=message, finished=time.time())


def _start_job(target):
    """Returns (accepted, message); one job at a time."""
    global _job
    with _lock:
        if _job is not None and _job['state'] == 'running':
            return False, f'a {_job["target"]} reboot is already running.'
        _job = {'target': target, 'state': 'running', 'message': f'restarting {target}...',
                'started': time.time(), 'finished': None}
    log.info('%s reboot requested', target)
    threading.Thread(target=_worker, args=(target,), daemon=True).start()
    return True, f'restarting {target}...'


def _probe():
    """Environment snapshot for /status, cached for PROBE_TTL_S (each one costs a sudo)."""
    global _probe_cache
    now = time.monotonic()
    cached = _probe_cache
    if cached and now - cached[0] < PROBE_TTL_S:
        return cached[1]
    # Non-zero exit when the program is merely stopped; the state word is the answer:
    # "quadruped:goaway_ws   RUNNING   pid 1234, uptime 0:05:00".
    _, out = _sudo(['/usr/bin/supervisorctl', 'status', SUPERVISOR_PROGRAM], timeout=20.0)
    parts = out.split()
    snapshot = {
        'rosbridge': {'listening': _port_listening(ROSBRIDGE_PORT)},
        'goaway_ws': {
            'container': 'running' if _container_started() else 'absent',
            'supervisor': parts[1] if len(parts) > 1 else 'unknown',
        },
    }
    _probe_cache = (now, snapshot)
    return snapshot


# ---------------------------------------------------------------------------------------
# routes
# ---------------------------------------------------------------------------------------

@app.get('/health')
def health():
    return jsonify(status='ok')


@app.get('/status')
def status():
    with _lock:
        job = dict(_job) if _job else None
    return jsonify(**_probe(), job=job)


@app.post('/reboot/<target>')
def reboot(target):
    if target not in TARGETS:
        return jsonify(error=f'unknown reboot target: {target}'), 404
    accepted, message = _start_job(target)
    if not accepted:
        return jsonify(accepted=False, error=message), 409
    return jsonify(accepted=True, message=message), 202


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO, format='[%(asctime)s] %(levelname)s %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S')
    log.info('reboot manager listening on 127.0.0.1:%d (container=%s, program=%s)',
             PORT, CONTAINER, SUPERVISOR_PROGRAM)
    # Loopback only: ui_backend.js proxies to it from the same host.
    app.run(host='127.0.0.1', port=PORT, threaded=True)
