#!/usr/bin/env python3
"""Replace NaN lidar readings with inf so SLAM ignores rays with no return."""

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRangeFilter(Node):
    def __init__(self):
        super().__init__('scan_range_filter')
        self._sub = self.create_subscription(
            LaserScan, '/scan_raw', self._callback, 10)
        self._pub = self.create_publisher(LaserScan, '/scan', 10)

    def _callback(self, msg: LaserScan):
        for i, r in enumerate(msg.ranges):
            if math.isnan(r):
                msg.ranges[i] = float('inf')
        self._pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    node = ScanRangeFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
