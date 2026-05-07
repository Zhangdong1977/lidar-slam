#!/usr/bin/env python3
"""Keyboard teleop that publishes zero velocity when no key is pressed."""

import sys
import select
import termios
import tty

import geometry_msgs.msg
import rclpy


MSG = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def vels(speed, turn):
    return f'currently:\tspeed {speed:.2f}\tturn {turn:.2f}'


def main():
    # Save terminal settings for restoration
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    speed = node.declare_parameter('speed', 0.5).value
    turn = node.declare_parameter('turn', 1.0).value

    pub = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
    twist = geometry_msgs.msg.Twist()

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0
    stale_count = 0
    stale_limit = 8

    print(MSG)
    print(vels(speed, turn))

    try:
        while rclpy.ok():
            # Non-blocking key read with short poll
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                key = sys.stdin.read(1)
                if key in MOVE_BINDINGS:
                    x = MOVE_BINDINGS[key][0]
                    y = MOVE_BINDINGS[key][1]
                    z = MOVE_BINDINGS[key][2]
                    th = MOVE_BINDINGS[key][3]
                    stale_count = 0
                elif key in SPEED_BINDINGS:
                    speed *= SPEED_BINDINGS[key][0]
                    turn *= SPEED_BINDINGS[key][1]
                    print(vels(speed, turn))
                    if status == 14:
                        print(MSG)
                    status = (status + 1) % 15
                    stale_count = 0
                elif key == '\x03':
                    break
                else:
                    # Unknown key: stop
                    x = 0.0
                    y = 0.0
                    z = 0.0
                    th = 0.0
            else:
                # No key available — reset to zero after a short holdoff
                # The holdoff lets you tap keys without immediate zero-crossing
                stale_count += 1
                if stale_count >= stale_limit:
                    x = 0.0
                    y = 0.0
                    z = 0.0
                    th = 0.0

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.z = th * turn
            pub.publish(twist)

            # Spin once to process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.001)

    except Exception as e:
        print(e)
    finally:
        # Publish zero and cleanup
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
