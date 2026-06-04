#!/usr/bin/env python3
"""Block until a specified ROS2 topic has at least N publishers, then exit.

Exit codes: 0 = ready, 1 = timeout.
Modeled on the existing wait_for_tf.py readiness guard pattern.
"""

import sys

import rclpy
from rclpy.node import Node


class WaitForTopic(Node):
    def __init__(self):
        super().__init__('wait_for_topic')

        self.declare_parameter('topic_name', '')
        self.declare_parameter('min_publishers', 1)
        self.declare_parameter('timeout', 60.0)
        self.declare_parameter('check_period', 0.5)
        self.declare_parameter('exit_on_timeout', True)

        self._topic_name = self.get_parameter('topic_name').value
        self._min_publishers = self.get_parameter('min_publishers').value
        timeout = self.get_parameter('timeout').value
        period = self.get_parameter('check_period').value
        exit_on_timeout = self.get_parameter('exit_on_timeout').value

        if not self._topic_name:
            self.get_logger().error('topic_name parameter is required')
            sys.exit(1)

        self.get_logger().info(
            f'Waiting for topic {self._topic_name} '
            f'(min_publishers={self._min_publishers}, timeout={timeout}s)'
        )

        # Poll in a loop (same pattern as wait_for_tf.py)
        import time
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            info = self.count_publishers(self._topic_name)
            if info >= self._min_publishers:
                self.get_logger().info(
                    f'Topic {self._topic_name} is ready '
                    f'({info} publishers)'
                )
                sys.exit(0)

            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'Timeout waiting for topic {self._topic_name} '
                    f'({info}/{self._min_publishers} publishers after {timeout}s)'
                )
                if exit_on_timeout:
                    sys.exit(1)
                deadline = time.monotonic() + timeout

            rclpy.spin_once(self, timeout_sec=period)

        sys.exit(1)


def main():
    rclpy.init(args=sys.argv)
    node = WaitForTopic()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
