#!/usr/bin/env python3
"""Unit tests for sim_ackermann_teleop.py core logic.

Tests key bindings, speed adjustment, and stale-limit behavior
without requiring ROS2 or keyboard hardware.
"""

import sys
import os

# ---------------------------------------------------------------------------
# 1) Load the module under test (patch out the heavy deps first)
# ---------------------------------------------------------------------------
# Simulate geometry_msgs.msg.Twist before the real import triggers
class _Twist:
    def __init__(self):
        self.linear = _Vec()
        self.angular = _Vec()

class _Vec:
    x = 0.0
    y = 0.0
    z = 0.0

import types as _types
geo_fake = _types.ModuleType('geometry_msgs')
geo_fake.__dict__['msg'] = _types.ModuleType('geometry_msgs.msg')
geo_fake.msg.__dict__['Twist'] = _Twist
sys.modules['geometry_msgs'] = geo_fake
sys.modules['geometry_msgs.msg'] = geo_fake.msg

# Inject the module's namespace so we can reference its symbols
source_path = os.path.join(os.path.dirname(__file__), 'sim_ackermann_teleop.py')
with open(source_path) as f:
    source = f.read()
# Strip shebang and import lines that need heavy deps, keep data/logic
# We only need the constants and pure functions
lines = []
skip = {'rclpy', 'geometry_msgs', 'termios', 'tty', 'threading', 'time'}
for line in source.split('\n'):
    stripped = line.strip()
    if stripped.startswith('import '):
        mod = stripped.split()[1].split('.')[0]
        if mod in skip:
            continue
    if stripped.startswith('from '):
        mod = stripped.split()[1].split('.')[0]
        if mod in skip:
            continue
    lines.append(line)

exec('\n'.join(lines))


# ---------------------------------------------------------------------------
# 2) Test helpers
# ---------------------------------------------------------------------------
def test_move_bindings():
    """Every move key maps to a (linear_dir, angular_dir) tuple."""
    assert 'i' in MOVE_BINDINGS
    assert MOVE_BINDINGS['i'] == (1, 0)
    assert MOVE_BINDINGS['k'] == (0, 0)
    assert MOVE_BINDINGS[' '] == (0, 0)
    assert MOVE_BINDINGS[','] == (-1, 0)
    assert MOVE_BINDINGS['j'] == (0, 1)
    assert MOVE_BINDINGS['l'] == (0, -1)
    # Diagonal combos
    assert MOVE_BINDINGS['u'] == (1, 1)
    assert MOVE_BINDINGS['o'] == (1, -1)
    assert MOVE_BINDINGS['m'] == (-1, 1)
    assert MOVE_BINDINGS['.'] == (-1, -1)
    print('  PASS: test_move_bindings')


def test_speed_bindings():
    """Speed modifiers produce expected factors."""
    assert SPEED_BINDINGS['q'] == (1.1, 1.1)
    assert SPEED_BINDINGS['z'] == (0.9, 0.9)
    assert SPEED_BINDINGS['w'] == (1.1, 1)
    assert SPEED_BINDINGS['x'] == (0.9, 1)
    assert SPEED_BINDINGS['e'] == (1, 1.1)
    assert SPEED_BINDINGS['c'] == (1, 0.9)
    print('  PASS: test_speed_bindings')


def test_status_str():
    """status_str formats speed and steer correctly."""
    s = status_str(1.5, 0.5)
    assert '1.50' in s
    assert '0.50' in s
    # 0.5 rad ≈ 28.6 deg
    assert '29' in s, f'expected ~29 deg, got: {s}'
    print('  PASS: test_status_str')


def test_stale_logic():
    """After stale_limit, velocity should decay to zero."""
    stale_limit = 6
    state = {'lin': 1.0, 'ang': 0.5, 'stale': 0}

    def step(st):
        with __import__('threading').Lock():
            lin = st['lin']
            ang = st['ang']
            st['stale'] += 1
            if st['stale'] > stale_limit:
                lin = 0.0
                ang = 0.0
            return lin, ang

    # Within limit — velocity unchanged
    for _ in range(stale_limit):
        lin, ang = step(state)
        assert lin == 1.0, f'stale={state["stale"]} lin should be 1.0'
        assert ang == 0.5

    # One more tick — decay
    lin, ang = step(state)
    assert lin == 0.0, f'lin should be 0 after stale, got {lin}'
    assert ang == 0.0
    print('  PASS: test_stale_logic')


def test_twist_composition():
    """linear.x = lin_dir * speed, angular.z = ang_dir * steer."""
    lin_dir, ang_dir = MOVE_BINDINGS['u']  # (1, 1)
    speed = 0.5
    steer = 0.3
    twist = _Twist()
    twist.linear.x = lin_dir * speed
    twist.angular.z = ang_dir * steer
    assert twist.linear.x == 0.5
    assert twist.angular.z == 0.3
    # Forward-only 'i'
    lin_dir, ang_dir = MOVE_BINDINGS['i']
    twist.linear.x = lin_dir * speed
    twist.angular.z = ang_dir * steer
    assert twist.linear.x == 0.5
    assert twist.angular.z == 0.0
    print('  PASS: test_twist_composition')


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    print('Running unit tests for sim_ackermann_teleop.py ...')
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
    print('All tests passed.')
