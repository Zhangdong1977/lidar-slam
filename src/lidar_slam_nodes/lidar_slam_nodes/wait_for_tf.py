#!/usr/bin/env python3
"""Block until a target TF frame chain is available, then exit."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


class WaitForTF(Node):
    def __init__(self):
        super().__init__('wait_for_tf')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'body_link')
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('check_period', 0.5)
        self.declare_parameter('exit_on_timeout', True)

        target = self.get_parameter('target_frame').value
        source = self.get_parameter('source_frame').value
        timeout = self.get_parameter('timeout').value
        period = self.get_parameter('check_period').value
        exit_on_timeout = self.get_parameter('exit_on_timeout').value

        self.get_logger().info(f'Waiting for TF: {source} -> {target} (timeout={timeout}s)')

        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        deadline = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok():
            now = self.get_clock().now()
            if now > deadline:
                self.get_logger().error(
                    f'Timeout waiting for TF: {source} -> {target}'
                )
                if exit_on_timeout:
                    sys.exit(1)
                deadline = now + Duration(seconds=timeout)

            if self._tf_buffer.can_transform(
                target, source, rclpy.time.Time(), Duration(seconds=0.5)
            ):
                self.get_logger().info(f'TF {source} -> {target} is available')
                sys.exit(0)

            rclpy.spin_once(self, timeout_sec=period)

        sys.exit(1)


def main():
    rclpy.init(args=sys.argv)
    node = WaitForTF()
    node.destroy_node()
    rclpy.shutdown()
