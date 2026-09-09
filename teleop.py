#!/usr/bin/env python3
"""RACECAR Neo web teleop service: drive the car from buttons or the keyboard
in a browser, with the camera, the lidar, and the encoder in view.

Same pattern as the other lab dashboards (stdlib HTTP + rclpy) on port
8081. The browser holds a command of two axes, forward/back and left/right,
each -1, 0, or +1, from whichever buttons or keys are down at once, and
posts it on every change and every 100 ms while anything is held:

  throttle = fwd  * speed      (fwd  = +1 forward, -1 back)
  steering = turn * angle      (turn = +1 right,  -1 left; student convention)

/drive is published at a fixed rate from a timer. A command older than
cmd_timeout is treated as released, so a closed tab, a dropped link, or a
page that lost focus with a key down all coast to zero rather than driving
the last command forever. That is the one thing this dashboard adds over
the student library's rc.drive.set_speed_angle.

CAUTION: the RACECAR Neo mux gates /drive on the RB bumper and zeroes
output when /joy or the active source goes stale. The shipped yaml has
speed at 0.0 so the buttons do nothing until the slider is raised.

/drive steering is negated on publish: positive on the wire turns this car
left (physically verified on the sibling dashboards). Speed feedback comes
from /odom.
"""

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import signal
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
import cv2
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
import yaml

PORT = 8081
BASE = Path(__file__).resolve().parent
YAML_PATH = BASE / 'teleop.yaml'
LOG_DIR = BASE / 'logs'

MAX_SPEED = 1.0         # hard throttle cap either way, as drive_real.set_max_speed
PUBLISH_RATE_HZ = 15.0  # /drive rate, independent of the browser
PREVIEW_RATE_HZ = 15.0
# The live chart is 420 px wide; send an even sample of the history rather
# than every row. The log csv keeps all of them.
HIST_POINTS = 300

TUNE_KEYS = ('speed', 'angle')

_lock = threading.Lock()
_params: dict = {}
_state: dict = {'fwd': 0, 'turn': 0, 'held': False, 'timed_out': False,
                'speed_cmd': 0.0, 'steer': 0.0, 'enc_speed': 0.0,
                'scan': [], 'res': [0, 0], 'cam_fps': 0.0,
                'hist': [], 'marks': []}
_preview = b''
# The browser's command and when it last arrived. Read by the timer.
_cmd: dict = {'fwd': 0, 'turn': 0, 'stamp': 0.0}
_hist: deque = deque(maxlen=1200)
# Full rows behind the live chart. POST /logs/save snapshots this to a csv,
# so a saved log is exactly what was on the screen.
_rows: deque = deque(maxlen=1200)
# Parameter-change markers [t, label] shown on the charts. Saved into logs
# as "# mark" lines.
_marks: deque = deque(maxlen=40)
_T0 = time.monotonic()


def _now():
    return round(time.monotonic() - _T0, 2)


def _add_mark(label):
    # Coalesce rapid updates: the sliders fire while they are dragged.
    if _marks and _now() - _marks[-1][0] < 1.0 and _marks[-1][1].split(' ')[0] == label.split(' ')[0]:
        _marks[-1] = [_now(), label]
    else:
        _marks.append([_now(), label])


def clamp_speed(speed):
    """Clamp throttle to [-MAX_SPEED, MAX_SPEED]. Every speed that leaves
    this program passes through here, so /drive can never carry more than
    1.0 in either direction."""
    return max(-MAX_SPEED, min(MAX_SPEED, float(speed)))


def _clean(p):
    """Clamp every tunable into its range. Caller holds _lock."""
    p['speed'] = max(0.0, clamp_speed(p['speed']))
    p['angle'] = max(0.0, min(1.0, float(p['angle'])))
    p['cmd_timeout'] = max(0.1, float(p.get('cmd_timeout', 0.5)))


def load_params():
    with _lock:
        before = json.dumps(_params, sort_keys=True)
        _params.update(yaml.safe_load(YAML_PATH.read_text()))
        _clean(_params)
        if before != '{}' and before != json.dumps(_params, sort_keys=True):
            _add_mark('yaml load')


def save_params():
    """Rewrite the numeric values in place so the yaml comments survive."""
    scalar = re.compile(r'^([A-Za-z_]\w*):([ \t]*)[-\d.eE+]+(.*)$')
    out = []
    with _lock:
        for raw in YAML_PATH.read_text().splitlines(keepends=True):
            m = scalar.match(raw.rstrip('\n'))
            if m:
                key, gap, tail = m.groups()
                v = _params.get(key)
                if isinstance(v, (int, float)):
                    out.append(f'{key}:{gap}{v}{tail}\n')
                    continue
            out.append(raw)
    YAML_PATH.write_text(''.join(out))


def update_params(data):
    """Apply a posted subset of TUNE_KEYS. Returns the change labels."""
    with _lock:
        before = dict(_params)
        for k in TUNE_KEYS:
            if k in data:
                _params[k] = float(data[k])
        _clean(_params)
        changed = [f'{k} {_params[k]:g}' for k in TUNE_KEYS if before.get(k) != _params[k]]
        if changed:
            _add_mark(', '.join(changed))
    return changed


def _axis(v):
    """Squash whatever the browser sent to -1, 0, or +1."""
    v = int(v)
    return (v > 0) - (v < 0)


def set_cmd(data, now=None):
    """Take the browser's command: {'fwd': -1|0|1, 'turn': -1|0|1}. Both
    axes together are one command, so forward and left arrive as one post
    and are applied at the same instant."""
    with _lock:
        _cmd['fwd'] = _axis(data.get('fwd', 0))
        _cmd['turn'] = _axis(data.get('turn', 0))
        _cmd['stamp'] = time.monotonic() if now is None else now


class TeleopNode(Node):
    def __init__(self):
        super().__init__('webteleop')
        with _lock:
            cam_topic = _params['camera_topic']
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.create_subscription(Image, cam_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        # /drive at a fixed rate whether or not the browser is talking, so
        # the mux never sees a gap and a silent browser reads as released.
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._actuate)

        self._enc = 0.0
        self._speed_cmd = 0.0
        self._steer_cmd = 0.0
        self._res = (0, 0)
        self._last_preview = 0.0
        self._frame_stamps = []

    def _odom_cb(self, msg):
        self._enc = msg.twist.twist.linear.x

    def _scan_cb(self, msg):
        # 0 = nose, positive = right (student convention); every third ray
        # is plenty for a 420 px view.
        scan = [[round(-math.degrees(msg.angle_min + i * msg.angle_increment), 1),
                 round(msg.ranges[i], 3)]
                for i in range(0, len(msg.ranges), 3)
                if msg.range_min < msg.ranges[i] < msg.range_max]
        with _lock:
            _state['scan'] = scan

    def _image_cb(self, msg):
        global _preview
        now = time.monotonic()
        self._frame_stamps.append(now)
        self._frame_stamps = [t for t in self._frame_stamps if now - t < 2.0]
        with _lock:
            _state['cam_fps'] = round(len(self._frame_stamps) / 2.0, 1)
        if now - self._last_preview < 1.0 / PREVIEW_RATE_HZ:
            return
        self._last_preview = now
        if msg.encoding == 'jpeg':
            arr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        else:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if arr is None:
            return
        h, w = arr.shape[:2]
        self._res = (w, h)
        with _lock:
            p = dict(_params)
        pw = int(p['preview_width'])
        small = cv2.resize(arr, (pw, max(1, round(pw * h / w))))
        quality = [cv2.IMWRITE_JPEG_QUALITY, int(p['preview_quality'])]
        with _lock:
            _preview = cv2.imencode('.jpg', small, quality)[1].tobytes()
            _state['res'] = [w, h]

    def _actuate(self, now=None):
        now = time.monotonic() if now is None else now
        with _lock:
            p = dict(_params)
            fwd, turn, stamp = _cmd['fwd'], _cmd['turn'], _cmd['stamp']
        held = fwd != 0 or turn != 0
        timed_out = held and now - stamp > p['cmd_timeout']
        if timed_out:
            fwd = turn = 0
        self._speed_cmd = clamp_speed(fwd * p['speed'])
        self._steer_cmd = max(-1.0, min(1.0, turn * p['angle']))

        out = AckermannDriveStamped()
        out.drive.speed = self._speed_cmd
        out.drive.steering_angle = float(-self._steer_cmd)  # wire positive = left on this car
        self._pub.publish(out)

        t = _now()
        _rows.append(f'{t},{fwd},{turn},{self._speed_cmd:.3f},{self._steer_cmd:.3f},{self._enc:.3f}\n')
        _hist.append([t, round(self._enc, 3), round(self._speed_cmd, 3)])
        with _lock:
            _state.update({'fwd': fwd, 'turn': turn, 'held': held and not timed_out,
                           'timed_out': timed_out,
                           'speed_cmd': round(self._speed_cmd, 3),
                           'steer': round(self._steer_cmd, 3),
                           'enc_speed': round(self._enc, 3),
                           'hist': _thin(_hist), 'marks': list(_marks)})


def _thin(hist):
    """Even sample of `hist` down to HIST_POINTS, newest row always kept."""
    rows = list(hist)
    stride = max(1, len(rows) // HIST_POINTS)
    if stride == 1:
        return rows
    sent = rows[::stride]
    if sent[-1] is not rows[-1]:
        sent.append(rows[-1])
    return sent


def _log_number(name):
    """Log7.csv -> 7, so lists sort numerically. Anything else sorts first."""
    m = re.match(r'Log(\d+)\.csv$', name)
    return int(m.group(1)) if m else 0


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def do_GET(self):
        if self.path == '/':
            self._send((BASE / 'teleop.html').read_bytes(), 'text/html; charset=utf-8')
        elif self.path.startswith('/frame'):
            with _lock:
                data = _preview
            if data:
                self._send(data, 'image/jpeg', cache='no-store')
            else:
                self.send_error(503)
        elif self.path == '/state':
            with _lock:
                body = json.dumps({**_state, 'params': _params, 'marks': list(_marks)})
            self._send(body.encode(), 'application/json')
        elif self.path == '/logs':
            names = sorted((f.name for f in LOG_DIR.glob('*.csv')), key=_log_number)
            self._send(json.dumps(names).encode(), 'application/json')
        elif self.path.startswith('/logs/'):
            f = LOG_DIR / Path(self.path).name
            if f.is_file():
                self._send(f.read_bytes(), 'text/csv')
            else:
                self.send_error(404)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/cmd':
            set_cmd(self._body())
        elif self.path == '/params':
            update_params(self._body())
        elif self.path == '/logs/save':
            LOG_DIR.mkdir(exist_ok=True)
            latest = max((_log_number(f.name) for f in LOG_DIR.glob('Log*.csv')), default=0)
            name = f'Log{latest + 1}.csv'
            rows = list(_rows)
            tmin = float(rows[0].split(',')[0]) if rows else 0.0
            with open(LOG_DIR / name, 'w') as f:
                f.write('t,fwd,turn,speed_cmd,steer,speed_ms\n')
                f.writelines(f'# mark,{t},{label}\n' for t, label in list(_marks) if t >= tmin)
                f.writelines(rows)
            self._send(name.encode(), 'text/plain')
            return
        elif self.path == '/save':
            save_params()
        elif self.path == '/load':
            load_params()
        else:
            self.send_error(404)
            return
        self._send(b'ok', 'text/plain')

    def _body(self):
        return json.loads(self.rfile.read(int(self.headers['Content-Length'])))

    def _send(self, body, ctype, cache=None):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        if cache:
            self.send_header('Cache-Control', cache)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


def _spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main():
    load_params()
    rclpy.init()
    node = TeleopNode()
    spin = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin.start()
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)

    # rclpy.init() installs SIGINT/SIGTERM handlers that shut the ROS context
    # down but leave serve_forever() blocked, so the process outlives the
    # signal until systemd's TimeoutStopSec expires and SIGKILLs it. Take the
    # signals back. shutdown() has to run off the serving thread or it
    # deadlocks waiting for the loop it is called from.
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    print(f'Web teleop dashboard on http://0.0.0.0:{PORT}')
    server.serve_forever()

    # Unwind the ROS wait before the interpreter tears the context down;
    # otherwise the spin thread aborts the process from C++.
    rclpy.shutdown()
    spin.join(timeout=2)


if __name__ == '__main__':
    main()
