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
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = [float('inf') if math.isnan(r) else r for r in msg.ranges]
        out.intensities = list(msg.intensities)
        # Strip Gazebo model namespace from frame_id:
        #   "ackermann_robot/body_link/lidar" -> "body_link/lidar"
        if '/' in msg.header.frame_id:
            out.header.frame_id = msg.header.frame_id.split('/', 1)[1]
        self._pub.publish(out)


def main():
    rclpy.init(args=sys.argv)
    node = ScanRangeFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
