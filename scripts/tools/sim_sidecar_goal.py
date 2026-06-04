#!/usr/bin/env python3
"""模拟 openTCS Sidecar: 发布路由图 + 发送路由导航目标.

完整模拟 Sidecar 行为:
  1. 发布 GeoJSON 路由图到 route_graph_json (Transient Local)
  2. 等待 route_server 加载路由图
  3. 调用 ComputeAndTrackRoute action 按路由图导航
  4. 监控导航进度并输出结果

用法:
  python3 scripts/tools/sim_sidecar_goal.py
  ros2 run lidar_slam_nodes sim_sidecar_goal
"""

import json
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav2_msgs.action import ComputeAndTrackRoute
from std_msgs.msg import String

# 3x2 网格路由图 (factory floor, 坐标均为 map frame, 单位米)
#
#   Node3(0,5)---Node4(5,5)---Node5(10,5)
#       |            |            |
#   Node0(0,0)---Node1(5,0)---Node2(10,0)
#
ROUTE_GRAPH = {
    'type': 'FeatureCollection',
    'features': [
        # ---- Nodes ----
        {'type': 'Feature',
         'properties': {'id': 0, 'frame': 'map'},
         'geometry': {'type': 'Point', 'coordinates': [0.0, 0.0]}},
        {'type': 'Feature',
         'properties': {'id': 1, 'frame': 'map'},
         'geometry': {'type': 'Point', 'coordinates': [5.0, 0.0]}},
        {'type': 'Feature',
         'properties': {'id': 2, 'frame': 'map'},
         'geometry': {'type': 'Point', 'coordinates': [10.0, 0.0]}},
        {'type': 'Feature',
         'properties': {'id': 3, 'frame': 'map'},
         'geometry': {'type': 'Point', 'coordinates': [0.0, 5.0]}},
        {'type': 'Feature',
         'properties': {'id': 4, 'frame': 'map'},
         'geometry': {'type': 'Point', 'coordinates': [5.0, 5.0]}},
        {'type': 'Feature',
         'properties': {'id': 5, 'frame': 'map'},
         'geometry': {'type': 'Point', 'coordinates': [10.0, 5.0]}},
        # ---- Edges (bidirectional: 每个方向一条) ----
        # 0 ↔ 1
        {'type': 'Feature',
         'properties': {'id': 100, 'startid': 0, 'endid': 1},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[0.0, 0.0], [5.0, 0.0]]]}},
        {'type': 'Feature',
         'properties': {'id': 101, 'startid': 1, 'endid': 0},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[5.0, 0.0], [0.0, 0.0]]]}},
        # 1 ↔ 2
        {'type': 'Feature',
         'properties': {'id': 102, 'startid': 1, 'endid': 2},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[5.0, 0.0], [10.0, 0.0]]]}},
        {'type': 'Feature',
         'properties': {'id': 103, 'startid': 2, 'endid': 1},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[10.0, 0.0], [5.0, 0.0]]]}},
        # 3 ↔ 4
        {'type': 'Feature',
         'properties': {'id': 104, 'startid': 3, 'endid': 4},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[0.0, 5.0], [5.0, 5.0]]]}},
        {'type': 'Feature',
         'properties': {'id': 105, 'startid': 4, 'endid': 3},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[5.0, 5.0], [0.0, 5.0]]]}},
        # 4 ↔ 5
        {'type': 'Feature',
         'properties': {'id': 106, 'startid': 4, 'endid': 5},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[5.0, 5.0], [10.0, 5.0]]]}},
        {'type': 'Feature',
         'properties': {'id': 107, 'startid': 5, 'endid': 4},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[10.0, 5.0], [5.0, 5.0]]]}},
        # 0 ↔ 3
        {'type': 'Feature',
         'properties': {'id': 108, 'startid': 0, 'endid': 3},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[0.0, 0.0], [0.0, 5.0]]]}},
        {'type': 'Feature',
         'properties': {'id': 109, 'startid': 3, 'endid': 0},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[0.0, 5.0], [0.0, 0.0]]]}},
        # 1 ↔ 4
        {'type': 'Feature',
         'properties': {'id': 110, 'startid': 1, 'endid': 4},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[5.0, 0.0], [5.0, 5.0]]]}},
        {'type': 'Feature',
         'properties': {'id': 111, 'startid': 4, 'endid': 1},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[5.0, 5.0], [5.0, 0.0]]]}},
        # 2 ↔ 5
        {'type': 'Feature',
         'properties': {'id': 112, 'startid': 2, 'endid': 5},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[10.0, 0.0], [10.0, 5.0]]]}},
        {'type': 'Feature',
         'properties': {'id': 113, 'startid': 5, 'endid': 2},
         'geometry': {'type': 'MultiLineString',
                      'coordinates': [[[10.0, 5.0], [10.0, 0.0]]]}},
    ],
}

# Navigation: start node → goal node
START_NODE_ID = 0   # Depot (0, 0)
GOAL_NODE_ID = 5    # NE-Station (10, 5)

ACTION_NAME = 'route_server/compute_and_track_route'


class SidecarSimulator(Node):

    def __init__(self):
        super().__init__('sidecar_simulator')

        self.declare_parameter('skip_graph_publish', False)
        self.declare_parameter('start_node', START_NODE_ID)
        self.declare_parameter('goal_node', GOAL_NODE_ID)
        self.declare_parameter('nav_timeout', 120.0)
        self.declare_parameter('namespace', '')
        self.declare_parameter('route_graph_topic', 'route_graph_json')
        self.declare_parameter('action_name', ACTION_NAME)

        self._skip_graph = self.get_parameter('skip_graph_publish').value
        self._start_id = self.get_parameter('start_node').value
        self._goal_id = self.get_parameter('goal_node').value
        self._timeout = self.get_parameter('nav_timeout').value
        namespace = self.get_parameter('namespace').value.strip('/')
        graph_topic = self.get_parameter('route_graph_topic').value
        action_name = self.get_parameter('action_name').value
        self._graph_topic = self._resolve_name(namespace, graph_topic)
        self._action_name = self._resolve_name(namespace, action_name)

        # Route graph publisher (Transient Local QoS)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._graph_pub = self.create_publisher(String, self._graph_topic, qos)

        # ComputeAndTrackRoute action client
        self._action_client = ActionClient(
            self, ComputeAndTrackRoute, self._action_name)

        # State machine
        self._step = 0
        self._goal_handle = None
        self._done = False
        self._start_time = 0.0

        # Kick off after a short delay
        self.create_timer(2.0, self._tick)

    @staticmethod
    def _resolve_name(namespace: str, name: str) -> str:
        clean_name = name.strip('/')
        if namespace:
            return f'/{namespace}/{clean_name}'
        return f'/{clean_name}'

    # -----------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------

    def _tick(self):
        if self._done:
            return

        if self._step == 0:
            self._publish_graph()
            self._step = 1
        elif self._step == 1:
            self._wait_and_navigate()
        elif self._step == 2:
            self._check_timeout()

    def _publish_graph(self):
        if self._skip_graph:
            self.get_logger().info('跳过路由图发布 (skip_graph_publish=true)')
            return

        msg = String()
        msg.data = json.dumps(ROUTE_GRAPH)
        self._graph_pub.publish(msg)

        n_nodes = sum(
            1 for f in ROUTE_GRAPH['features']
            if f['geometry']['type'] == 'Point')
        n_edges = sum(
            1 for f in ROUTE_GRAPH['features']
            if f['geometry']['type'] in ('LineString', 'MultiLineString'))
        self.get_logger().info(
            f'已发布路由图到 {self._graph_topic}: {n_nodes} 节点, {n_edges} 边')

    def _wait_and_navigate(self):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                '等待 route_server action 就绪...', throttle_duration_sec=5.0)
            return

        self.get_logger().info(
            f'route_server 已就绪，发送路由导航目标: '
            f'Node{self._start_id} → Node{self._goal_id}')

        goal = ComputeAndTrackRoute.Goal()
        goal.start_id = self._start_id
        goal.goal_id = self._goal_id
        goal.use_start = False
        goal.use_poses = False

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

        self._step = 2
        self._start_time = time.monotonic()

    def _check_timeout(self):
        elapsed = time.monotonic() - self._start_time
        if elapsed > self._timeout:
            self.get_logger().error(
                f'导航超时 ({self._timeout:.0f}s)，取消目标')
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._finish('TIMEOUT')

    # -----------------------------------------------------------------
    # Action callbacks
    # -----------------------------------------------------------------

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'路由导航目标被拒绝 (error code: {goal_handle.status})')
            self._finish('REJECTED')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('路由导航目标已接受，开始执行...')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'导航中: 节点 {fb.last_node_id} → {fb.next_node_id}, '
            f'边 {fb.current_edge_id}, '
            f'重路由={fb.rerouted}, '
            f'路径点数={len(fb.path.poses)}')

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            duration = result.execution_duration
            secs = duration.sec + duration.nanosec * 1e-9
            self.get_logger().info(
                f'导航成功! 耗时 {secs:.1f}s')
            self._finish('SUCCEEDED')
        elif status == 5:  # CANCELED
            self.get_logger().warn('导航已取消')
            self._finish('CANCELED')
        elif status == 6:  # ABORTED
            self.get_logger().error(
                f'导航失败 (error_code={result.error_code})')
            self._finish('ABORTED')
        else:
            self.get_logger().error(
                f'导航未知状态: {status}')
            self._finish(f'UNKNOWN({status})')

    # -----------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------

    def _finish(self, result: str):
        self._done = True
        self.get_logger().info(f'===== Sidecar 模拟结束: {result} =====')
        # Schedule shutdown after a short delay to flush logs
        self.create_timer(1.0, lambda: self._shutdown())

    def _shutdown(self):
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = SidecarSimulator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
