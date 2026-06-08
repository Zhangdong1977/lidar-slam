#!/usr/bin/env python3
"""Block until a specified ROS2 topic is ready, then exit.

Exit codes: 0 = ready, 1 = timeout.
Modeled on the existing wait_for_tf.py readiness guard pattern.
"""

import importlib
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message


class WaitForTopic(Node):
    def __init__(self):
        super().__init__('wait_for_topic')

        self.declare_parameter('topic_name', '')
        self.declare_parameter('topic_type', '')
        self.declare_parameter('min_publishers', 1)
        self.declare_parameter('min_messages', 1)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_durability', 'volatile')
        self.declare_parameter('timeout', 60.0)
        self.declare_parameter('check_period', 0.5)
        self.declare_parameter('exit_on_timeout', True)

        self._topic_name = self.get_parameter('topic_name').value
        self._topic_type = self.get_parameter('topic_type').value
        self._min_publishers = self.get_parameter('min_publishers').value
        self._min_messages = self.get_parameter('min_messages').value
        self._qos_depth = self.get_parameter('qos_depth').value
        self._qos_reliability = self.get_parameter('qos_reliability').value
        self._qos_durability = self.get_parameter('qos_durability').value
        timeout = self.get_parameter('timeout').value
        period = self.get_parameter('check_period').value
        exit_on_timeout = self.get_parameter('exit_on_timeout').value
        self._message_count = 0

        if not self._topic_name:
            self.get_logger().error('topic_name parameter is required')
            sys.exit(1)

        if self._topic_type:
            self._wait_for_messages(timeout, period, exit_on_timeout)
        else:
            self._wait_for_publishers(timeout, period, exit_on_timeout)

    def _wait_for_publishers(self, timeout, period, exit_on_timeout):
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

    def _wait_for_messages(self, timeout, period, exit_on_timeout):
        try:
            msg_cls = get_message(self._topic_type)
        except (AttributeError, ModuleNotFoundError, ValueError):
            msg_cls = self._import_message_type(self._topic_type)

        qos = self._make_qos_profile()
        self.create_subscription(
            msg_cls, self._topic_name, self._message_callback, qos)

        self.get_logger().info(
            f'Waiting for messages on {self._topic_name} '
            f'(type={self._topic_type}, min_messages={self._min_messages}, '
            f'qos={self._qos_reliability}/{self._qos_durability}, '
            f'timeout={timeout}s)'
        )

        import time
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            if self._message_count >= self._min_messages:
                self.get_logger().info(
                    f'Topic {self._topic_name} is ready '
                    f'({self._message_count} messages received)'
                )
                sys.exit(0)

            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'Timeout waiting for messages on {self._topic_name} '
                    f'({self._message_count}/{self._min_messages} messages '
                    f'after {timeout}s)'
                )
                if exit_on_timeout:
                    sys.exit(1)
                deadline = time.monotonic() + timeout

            rclpy.spin_once(self, timeout_sec=period)

        sys.exit(1)

    def _message_callback(self, _msg):
        self._message_count += 1

    def _make_qos_profile(self):
        reliability = str(self._qos_reliability).lower()
        durability = str(self._qos_durability).lower()

        reliability_policy = {
            'reliable': ReliabilityPolicy.RELIABLE,
            'best_effort': ReliabilityPolicy.BEST_EFFORT,
            'besteffort': ReliabilityPolicy.BEST_EFFORT,
        }.get(reliability)
        durability_policy = {
            'volatile': DurabilityPolicy.VOLATILE,
            'transient_local': DurabilityPolicy.TRANSIENT_LOCAL,
            'transientlocal': DurabilityPolicy.TRANSIENT_LOCAL,
        }.get(durability)

        if reliability_policy is None:
            raise ValueError(
                'qos_reliability must be reliable or best_effort')
        if durability_policy is None:
            raise ValueError(
                'qos_durability must be volatile or transient_local')

        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(self._qos_depth),
            reliability=reliability_policy,
            durability=durability_policy,
        )

    @staticmethod
    def _import_message_type(type_name):
        parts = type_name.split('/')
        if len(parts) != 3 or parts[1] != 'msg':
            raise ValueError(
                'topic_type must be in package/msg/Message form')
        module = importlib.import_module(f'{parts[0]}.msg')
        return getattr(module, parts[2])


def main():
    rclpy.init(args=sys.argv)
    node = WaitForTopic()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
