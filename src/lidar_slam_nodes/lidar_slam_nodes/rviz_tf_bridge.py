#!/usr/bin/env python3
"""Relay one namespaced vehicle TF tree into RViz-friendly global topics."""

import copy
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class RvizTFBridge(Node):
    def __init__(self):
        super().__init__('rviz_tf_bridge')
        self.declare_parameter('namespace', 'gazebo_1')
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('scan_in', 'scan')
        self.declare_parameter('scan_out', '')

        namespace = str(self.get_parameter('namespace').value).strip('/')
        if not namespace:
            raise RuntimeError('namespace parameter must not be empty')
        self._namespace = namespace
        self._fixed_frame = str(self.get_parameter('fixed_frame').value).strip('/')
        self._scan_in = str(self.get_parameter('scan_in').value).strip('/')

        scan_out = str(self.get_parameter('scan_out').value).strip('/')
        if not scan_out:
            scan_out = f'rviz/{namespace}/scan'
        self._scan_out = scan_out

        tf_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._tf_pub = self.create_publisher(TFMessage, '/tf', tf_pub_qos)
        self._tf_static_pub = self.create_publisher(TFMessage, '/tf_static', tf_static_qos)
        self._scan_pub = self.create_publisher(
            LaserScan, f'/{self._scan_out}', qos_profile_sensor_data)

        self.create_subscription(
            TFMessage, f'/{namespace}/tf', self._tf_callback, tf_sub_qos)
        self.create_subscription(
            TFMessage, f'/{namespace}/tf_static', self._tf_static_callback, tf_static_qos)
        self.create_subscription(
            LaserScan, f'/{namespace}/{self._scan_in}', self._scan_callback,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Relaying /{namespace}/tf to /tf with frame prefix "{namespace}/"; '
            f'/{namespace}/{self._scan_in} -> /{self._scan_out}')

    def _rviz_frame(self, frame_id):
        frame_id = str(frame_id).strip('/')
        if not frame_id:
            return frame_id
        if frame_id == self._fixed_frame:
            return frame_id
        if frame_id.startswith(f'{self._namespace}/'):
            return frame_id
        return f'{self._namespace}/{frame_id}'

    def _rewrite_tf_message(self, msg):
        out = TFMessage()
        for transform in msg.transforms:
            rewritten = copy.deepcopy(transform)
            rewritten.header.frame_id = self._rviz_frame(transform.header.frame_id)
            rewritten.child_frame_id = self._rviz_frame(transform.child_frame_id)
            out.transforms.append(rewritten)
        return out

    def _tf_callback(self, msg):
        out = self._rewrite_tf_message(msg)
        if out.transforms:
            self._tf_pub.publish(out)

    def _tf_static_callback(self, msg):
        out = self._rewrite_tf_message(msg)
        if out.transforms:
            self._tf_static_pub.publish(out)

    def _scan_callback(self, msg):
        out = copy.deepcopy(msg)
        out.header.frame_id = self._rviz_frame(msg.header.frame_id)
        self._scan_pub.publish(out)


def main():
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = RvizTFBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
