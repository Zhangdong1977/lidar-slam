#!/usr/bin/env python3
"""Ackermann keyboard teleop publishing /steering_angle and /velocity (Float64).

Uses select-based non-blocking keyboard reads.
"""

import select
import sys
import termios
import tty

import rclpy
from std_msgs.msg import Float64


MSG = """
Ackermann Keyboard Teleop
---------------------------
  u    i    o
  j    k    l
  m    ,    .

i / ,  : forward / backward
j / l  : steer left / right
u / o  : forward+left / forward+right
m / .  : backward+left / backward+right
k / SPACE : brake

q/z : increase/decrease both speed & steer 10%
w/x : increase/decrease speed 10%
e/c : increase/decrease steer 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
    'i': (1, 0),
    ',': (-1, 0),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    'o': (1, -1),
    'm': (-1, 1),
    '.': (-1, -1),
    'k': (0, 0),
    ' ': (0, 0),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def status_str(speed, steer):
    return f'speed: {speed:.2f} m/s  steer: {steer:.2f} rad ({steer * 57.3:.0f} deg)'


def main():
    rclpy.init()
    node = rclpy.create_node('ackermann_teleop')

    speed = node.declare_parameter('speed', 1.0).value
    steer = node.declare_parameter('steer', 0.3).value

    pub_steer = node.create_publisher(Float64, '/steering_angle', 10)
    pub_vel = node.create_publisher(Float64, '/velocity', 10)

    lin = 0.0
    ang = 0.0
    status = 0
    stale_count = 0
    stale_limit = 8

    print(MSG)
    print(status_str(speed, steer), flush=True)

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while rclpy.ok():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                key = sys.stdin.read(1)
                if key in MOVE_BINDINGS:
                    lin = MOVE_BINDINGS[key][0]
                    ang = MOVE_BINDINGS[key][1]
                    stale_count = 0
                elif key in SPEED_BINDINGS:
                    speed *= SPEED_BINDINGS[key][0]
                    steer *= SPEED_BINDINGS[key][1]
                    sys.stdout.write(status_str(speed, steer) + '\r\n')
                    if status == 14:
                        sys.stdout.write(MSG.replace('\n', '\r\n') + '\r\n')
                    status = (status + 1) % 15
                    stale_count = 0
                elif key == '\x03':
                    break
                else:
                    lin = 0.0
                    ang = 0.0
            else:
                stale_count += 1
                if stale_count >= stale_limit:
                    lin = 0.0
                    ang = 0.0

            msg_s = Float64()
            msg_s.data = ang * steer
            msg_v = Float64()
            msg_v.data = lin * speed
            pub_steer.publish(msg_s)
            pub_vel.publish(msg_v)
            rclpy.spin_once(node, timeout_sec=0.001)

    except Exception as e:
        sys.stdout.write(str(e) + '\r\n')
    finally:
        msg_s = Float64()
        msg_s.data = 0.0
        msg_v = Float64()
        msg_v.data = 0.0
        pub_steer.publish(msg_s)
        pub_vel.publish(msg_v)
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
