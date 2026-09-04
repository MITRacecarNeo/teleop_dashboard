"""Command, timeout, and actuation tests for teleop.py.

Runs without a ROS graph: Node.__init__ and the create_* factories are
stubbed, so the real TeleopNode.__init__ still sets every field and the
timer callback is driven by hand with an explicit clock.

    source /opt/ros/humble/setup.bash
    source /home/racecar/ros2_ws/install/setup.bash
    pytest -q
"""

import importlib.util
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image, LaserScan

BASE = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('teleop', BASE / 'teleop.py')
tp = importlib.util.module_from_spec(_spec)
sys.modules['teleop'] = tp
_spec.loader.exec_module(tp)

SHIPPED = None  # yaml as committed, restored after any test that writes it


class Recorder:
    """Stands in for the /drive publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(autouse=True)
def params():
    """Fresh parameters per test; restore the file if a test saved over it."""
    global SHIPPED
    if SHIPPED is None:
        SHIPPED = (BASE / 'teleop.yaml').read_text()
    tp._params.clear()
    tp._marks.clear()
    tp._rows.clear()
    tp._hist.clear()
    tp._cmd.update(fwd=0, turn=0, stamp=0.0)
    tp.load_params()
    tp._params.update(speed=0.5, angle=0.8)
    yield tp._params
    if (BASE / 'teleop.yaml').read_text() != SHIPPED:
        (BASE / 'teleop.yaml').write_text(SHIPPED)


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(tp.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(tp.TeleopNode, 'create_publisher',
                        lambda self, *a, **k: Recorder(), raising=False)
    monkeypatch.setattr(tp.TeleopNode, 'create_subscription',
                        lambda self, *a, **k: None, raising=False)
    monkeypatch.setattr(tp.TeleopNode, 'create_timer',
                        lambda self, *a, **k: None, raising=False)
    return tp.TeleopNode()


def drive(node, fwd, turn, at=100.0, tick=None):
    """Post a command at time `at`, then run the timer at `tick` (default: same instant)."""
    tp.set_cmd({'fwd': fwd, 'turn': turn}, now=at)
    node._actuate(now=at if tick is None else tick)
    return node._pub.msgs[-1]


# ---- parameters ----

def test_speed_is_capped_both_ways():
    assert tp.clamp_speed(5.0) == tp.MAX_SPEED
    assert tp.clamp_speed(-5.0) == -tp.MAX_SPEED


def test_shipped_yaml_cannot_drive():
    import yaml
    assert yaml.safe_load(SHIPPED)['speed'] == 0.0


def test_save_keeps_comments(params):
    params.update(speed=0.35, angle=0.6)
    tp.save_params()
    text = (BASE / 'teleop.yaml').read_text()
    assert 'the buttons do' in text  # comments survive
    tp._params.clear()
    tp.load_params()
    assert tp._params['speed'] == 0.35
    assert tp._params['angle'] == 0.6
    assert tp._params['camera_topic'] == '/camera/color'


def test_update_clamps(params):
    tp.update_params({'speed': 4.0, 'angle': -1.0})
    assert params['speed'] == tp.MAX_SPEED
    assert params['angle'] == 0.0
    assert tp.update_params({'speed': 1.0}) == []
    assert tp.update_params({'speed': 0.2}) == ['speed 0.2']


# ---- command mapping ----

def test_forward_publishes_the_speed_slider(node, params):
    msg = drive(node, 1, 0)
    assert msg.drive.speed == pytest.approx(0.5)
    assert msg.drive.steering_angle == 0.0
    assert tp._state['fwd'] == 1 and tp._state['held']


def test_back_is_negative_speed(node):
    assert drive(node, -1, 0).drive.speed == pytest.approx(-0.5)


def test_left_turns_left_on_the_wire(node):
    # Student convention: left is negative; the wire is negated, so left is
    # positive on /drive (physically verified on the sibling dashboards).
    assert drive(node, 0, -1).drive.steering_angle == pytest.approx(0.8)
    assert drive(node, 0, 1).drive.steering_angle == pytest.approx(-0.8)


def test_forward_and_left_at_once(node):
    msg = drive(node, 1, -1)
    assert msg.drive.speed == pytest.approx(0.5)
    assert msg.drive.steering_angle == pytest.approx(0.8)
    assert (tp._state['fwd'], tp._state['turn']) == (1, -1)


def test_axes_are_squashed_to_unit(node):
    tp.set_cmd({'fwd': 7, 'turn': -3})
    assert (tp._cmd['fwd'], tp._cmd['turn']) == (1, -1)
    tp.set_cmd({})
    assert (tp._cmd['fwd'], tp._cmd['turn']) == (0, 0)


def test_release_stops(node):
    drive(node, 1, 1)
    msg = drive(node, 0, 0)
    assert msg.drive.speed == 0.0 and msg.drive.steering_angle == 0.0
    assert not tp._state['held']


def test_speed_zero_means_no_motion_but_steering(node, params):
    params['speed'] = 0.0
    msg = drive(node, 1, 1)
    assert msg.drive.speed == 0.0
    assert msg.drive.steering_angle == pytest.approx(-0.8)


# ---- timeout ----

def test_stale_command_drives_zero(node, params):
    params['cmd_timeout'] = 0.5
    msg = drive(node, 1, -1, at=100.0, tick=100.4)
    assert msg.drive.speed == pytest.approx(0.5)
    assert not tp._state['timed_out']
    node._actuate(now=100.6)
    msg = node._pub.msgs[-1]
    assert msg.drive.speed == 0.0 and msg.drive.steering_angle == 0.0
    assert tp._state['timed_out']
    assert not tp._state['held']


def test_a_fresh_command_clears_the_timeout(node, params):
    params['cmd_timeout'] = 0.5
    drive(node, 1, 0, at=100.0, tick=101.0)
    assert tp._state['timed_out']
    drive(node, 1, 0, at=101.0)
    assert not tp._state['timed_out']
    assert node._pub.msgs[-1].drive.speed == pytest.approx(0.5)


def test_a_released_command_never_reads_as_timed_out(node):
    drive(node, 0, 0, at=100.0, tick=200.0)
    assert not tp._state['timed_out']


# ---- sensors ----

def test_scan_is_thinned_and_in_student_degrees(node):
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_increment = 2 * math.pi / 360
    msg.range_min, msg.range_max = 0.05, 12.0
    msg.ranges = [2.0] * 360
    node._scan_cb(msg)
    scan = tp._state['scan']
    assert len(scan) == 120
    assert scan[0] == [180.0, 2.0]  # raw -pi is the tail, student +180


def test_camera_preview_is_encoded(node):
    img = np.full((480, 640, 3), 90, np.uint8)
    msg = Image()
    msg.encoding = 'jpeg'
    msg.height, msg.width = 480, 640
    msg.data = cv2.imencode('.jpg', img)[1].tobytes()
    node._last_preview = 0.0
    node._image_cb(msg)
    assert tp._preview[:2] == b'\xff\xd8'
    assert tp._state['res'] == [640, 480]


def test_rows_and_history_record_the_command(node):
    node._enc = 1.25
    drive(node, 1, -1)
    assert tp._rows[-1].strip().split(',')[1:] == ['1', '-1', '0.500', '-0.800', '1.250']
    assert tp._hist[-1][1:] == [1.25, 0.5]


def test_thin_keeps_the_newest_row():
    rows = [[i, 0.0, 0.0] for i in range(1000)]
    sent = tp._thin(rows)
    assert len(sent) < len(rows)
    assert sent[-1] is rows[-1]
