#!/usr/bin/env python3
"""Watch explore_lite status and save map on exploration completion."""

import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from explore_lite_msgs.msg import ExploreStatus

MAX_RETRIES = 5
RETRY_DELAY = 5.0  # seconds


class MapSaverWatcher(Node):
    def __init__(self):
        super().__init__('map_saver_watcher')
        self._saved = False
        self._map_dir = os.environ.get(
            'LIDAR_SLAM_ROOT', '/home/hello/lidar-slam',
        )
        self._map_name = os.environ.get(
            'EXPLORE_MAP_NAME', 'auto_exploration_map',
        )
        self.create_subscription(
            ExploreStatus,
            '/explore/status',
            self._status_cb,
            10,
        )

    def _status_cb(self, msg: ExploreStatus):
        # Allow re-saving if exploration resumes after a previous completion.
        if msg.status == ExploreStatus.EXPLORATION_IN_PROGRESS:
            self._saved = False
            return
        if self._saved:
            return
        if msg.status != ExploreStatus.EXPLORATION_COMPLETE:
            return
        self._saved = True
        self.get_logger().info('Exploration complete — saving map…')
        map_path = os.path.join(self._map_dir, 'maps', self._map_name)
        for attempt in range(1, MAX_RETRIES + 1):
            try:
                subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                     '-f', map_path, '--ros-args', '-p', 'save_map_timeout:=10000.0'],
                    check=True, timeout=90.0,
                )
                self.get_logger().info(f'Map saved to {map_path}')
                return
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
                self.get_logger().warn(
                    f'map_saver_cli attempt {attempt}/{MAX_RETRIES} failed: {e}'
                )
                if attempt < MAX_RETRIES:
                    self.get_logger().info(f'Retrying in {RETRY_DELAY}s...')
                    time.sleep(RETRY_DELAY)
                else:
                    self.get_logger().error(
                        f'map_saver_cli failed after {MAX_RETRIES} attempts'
                    )


def main():
    rclpy.init(args=sys.argv)
    node = MapSaverWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
