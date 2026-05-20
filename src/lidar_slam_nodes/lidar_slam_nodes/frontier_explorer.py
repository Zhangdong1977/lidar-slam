#!/usr/bin/env python3
"""Frontier-based autonomous exploration for Ackermann robot.

Subscribes to /map from slam_toolbox, detects frontier regions (boundary
between known-free and unknown space), and dispatches NavigateToPose goals
to Nav2. The robot explores autonomously until no reachable frontiers remain.
"""

import math
import os
import enum
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException


class State(enum.Enum):
    IDLE = 0
    SELECTING = 1
    NAVIGATING = 2
    COMPLETED = 3


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.declare_parameter('frontier_min_size', 20)
        self.declare_parameter('size_weight', 1.0)
        self.declare_parameter('distance_weight', 0.5)
        self.declare_parameter('heading_weight', 2.0)
        self.declare_parameter('max_goal_distance', 15.0)
        self.declare_parameter('min_goal_distance', 1.0)
        self.declare_parameter('max_heading_diff', 2.5)
        self.declare_parameter('explore_rate', 0.5)
        self.declare_parameter('completion_check_count', 10)
        self.declare_parameter('planning_retry_count', 5)
        self.declare_parameter('max_global_retries', 1)
        self.declare_parameter('min_free_cells', 500)
        self.declare_parameter('stuck_timeout', 180.0)
        self.declare_parameter('stuck_distance', 0.5)
        self.declare_parameter('goal_obstacle_clearance', 0.6)
        self.declare_parameter('initial_warmup_seconds', 10.0)
        self.declare_parameter('nav2_wait_timeout', 120.0)
        self.declare_parameter('blacklist_radius', 5.0)
        self.declare_parameter('stuck_position_count', 3)
        self.declare_parameter('long_range_enabled', True)
        self.declare_parameter('long_range_min_cluster_size', 50)
        self.declare_parameter('max_unknown_ratio', 0.15)

        self.frontier_min_size = self.get_parameter('frontier_min_size').value
        self.size_weight = self.get_parameter('size_weight').value
        self.distance_weight = self.get_parameter('distance_weight').value
        self.heading_weight = self.get_parameter('heading_weight').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.max_heading_diff = self.get_parameter('max_heading_diff').value
        self.planning_retry_count = self.get_parameter('planning_retry_count').value
        self.min_free_cells = self.get_parameter('min_free_cells').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_distance = self.get_parameter('stuck_distance').value
        self.goal_obstacle_clearance = self.get_parameter('goal_obstacle_clearance').value
        self.max_global_retries = self.get_parameter('max_global_retries').value
        self.initial_warmup_seconds = self.get_parameter('initial_warmup_seconds').value
        self.nav2_wait_timeout = self.get_parameter('nav2_wait_timeout').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.stuck_position_count = self.get_parameter('stuck_position_count').value
        self.long_range_enabled = self.get_parameter('long_range_enabled').value
        self.long_range_min_cluster_size = self.get_parameter('long_range_min_cluster_size').value
        self.max_unknown_ratio = self.get_parameter('max_unknown_ratio').value
        completion_check_count = self.get_parameter('completion_check_count').value
        explore_rate = self.get_parameter('explore_rate').value

        # State
        self.current_map = None
        self.state = State.IDLE
        self.consecutive_empty = 0
        self.completion_threshold = completion_check_count
        self.tried_centroids = []
        self.failed_centroids = []
        self.global_retry_count = 0
        self.current_goal_accepted = False

        # Stuck detection
        self.current_goal_handle = None
        self.last_progress_pose = None
        self.last_progress_time = None

        # Position-level stuck detection
        self.stuck_position = None
        self.stuck_at_position_count = 0
        self._long_range_goals = []
        self._repositioning = False

        # Startup tracking
        self._start_time = self.get_clock().now()
        self._nav2_ready = False
        self._nav2_ready_checked = False

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Timer
        self.create_timer(1.0 / explore_rate, self.explore_step)

        self.get_logger().info(
            f'Frontier explorer started, warmup={self.initial_warmup_seconds}s, '
            f'waiting for Nav2 and stable map...')

    def map_callback(self, msg):
        self.current_map = msg

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'body_link', rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return tx, ty, yaw
        except TransformException:
            return None

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def detect_frontiers(self, grid, height, width):
        free = (grid == 0)
        unknown = (grid == -1)

        # A free cell is a frontier if it has at least one unknown 4-neighbor
        frontier = np.zeros_like(free, dtype=bool)
        # Check each direction via shifted arrays
        if np.any(unknown):
            # Up: unknown cell below free cell
            frontier[:-1, :] |= (free[:-1, :] & unknown[1:, :])
            # Down
            frontier[1:, :] |= (free[1:, :] & unknown[:-1, :])
            # Left
            frontier[:, :-1] |= (free[:, :-1] & unknown[:, 1:])
            # Right
            frontier[:, 1:] |= (free[:, 1:] & unknown[:, :-1])

        return frontier

    def cluster_frontiers(self, frontier_mask):
        labeled = np.zeros_like(frontier_mask, dtype=np.int32)
        clusters = []
        cluster_id = 0

        ys, xs = np.where(frontier_mask)
        frontier_set = set(zip(ys.tolist(), xs.tolist()))

        for y, x in zip(ys.tolist(), xs.tolist()):
            if labeled[y, x] != 0 or (y, x) not in frontier_set:
                continue
            cluster_id += 1
            queue = deque([(y, x)])
            cells = []
            while queue:
                cy, cx = queue.popleft()
                if labeled[cy, cx] != 0:
                    continue
                labeled[cy, cx] = cluster_id
                cells.append((cy, cx))
                for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    ny, nx = cy + dy, cx + dx
                    if (0 <= ny < frontier_mask.shape[0] and
                            0 <= nx < frontier_mask.shape[1] and
                            labeled[ny, nx] == 0 and
                            (ny, nx) in frontier_set):
                        queue.append((ny, nx))
            if len(cells) >= self.frontier_min_size:
                clusters.append(cells)

        return clusters

    def cell_to_world(self, row, col, info):
        wx = info.origin.position.x + (col + 0.5) * info.resolution
        wy = info.origin.position.y + (row + 0.5) * info.resolution
        return wx, wy

    def world_to_cell(self, wx, wy, info):
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        return row, col

    def is_free(self, row, col, grid, height, width):
        if 0 <= row < height and 0 <= col < width:
            return grid[row, col] == 0
        return False

    def has_obstacle_clearance(self, row, col, grid, height, width, clearance_m, resolution):
        clearance_cells = int(math.ceil(clearance_m / resolution))
        r2 = clearance_cells * clearance_cells
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                if dy * dy + dx * dx > r2:
                    continue
                nr, nc = row + dy, col + dx
                if 0 <= nr < height and 0 <= nc < width:
                    if grid[nr, nc] > 0:
                        return False
        return True

    def snap_to_free(self, wx, wy, grid, info, radius=5):
        row, col = self.world_to_cell(wx, wy, info)
        height, width = grid.shape
        if self.is_free(row, col, grid, height, width):
            return wx, wy
        for r in range(1, radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    nr, nc = row + dy, col + dx
                    if self.is_free(nr, nc, grid, height, width):
                        return self.cell_to_world(nr, nc, info)
        return None

    def is_ackermann_feasible(self, gx, gy, rx, ry, ryaw):
        dx = gx - rx
        dy = gy - ry
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < self.min_goal_distance or dist > self.max_goal_distance:
            return False
        goal_dir = math.atan2(dy, dx)
        heading_diff = abs(self.normalize_angle(goal_dir - ryaw))
        # Reject goals that require large heading change -- Ackermann cannot
        # efficiently reverse to reach targets behind it.
        if heading_diff > self.max_heading_diff:
            return False
        return True

    def find_frontiers(self):
        if self.current_map is None:
            return []

        msg = self.current_map
        data = np.array(msg.data, dtype=np.int8)
        free_count = np.count_nonzero(data == 0)
        if free_count < self.min_free_cells:
            return []

        height, width = msg.info.height, msg.info.width
        grid = data.reshape((height, width))

        frontier_mask = self.detect_frontiers(grid, height, width)
        if not np.any(frontier_mask):
            return []

        clusters = self.cluster_frontiers(frontier_mask)

        pose = self.get_robot_pose()
        if pose is None:
            return []
        rx, ry, ryaw = pose

        scored = []
        for cells in clusters:
            wxs, wys = [], []
            for (r, c) in cells:
                w = self.cell_to_world(r, c, msg.info)
                wxs.append(w[0])
                wys.append(w[1])
            cx = sum(wxs) / len(wxs)
            cy = sum(wys) / len(wys)

            # Snap centroid to nearest free cell
            snapped = self.snap_to_free(cx, cy, grid, msg.info, radius=10)
            if snapped is None:
                continue
            sx, sy = snapped

            # Reject goals too close to known obstacles
            srow, scol = self.world_to_cell(sx, sy, msg.info)
            if not self.has_obstacle_clearance(
                    srow, scol, grid, height, width,
                    self.goal_obstacle_clearance, msg.info.resolution):
                continue

            dist = math.sqrt((sx - rx) ** 2 + (sy - ry) ** 2)
            goal_dir = math.atan2(sy - ry, sx - rx)
            heading_diff = abs(self.normalize_angle(goal_dir - ryaw))
            heading_cost = heading_diff / math.pi
            score = (len(cells) * self.size_weight
                     - dist * self.distance_weight
                     - heading_cost * self.heading_weight)
            scored.append((score, sx, sy, len(cells)))

        scored.sort(key=lambda x: x[0], reverse=True)

        # Two-pass filtering: strict (heading-feasible) then relaxed (any heading)
        strict_goals = []
        relaxed_goals = []
        for score, gx, gy, size in scored:
            # Distance check (always apply)
            dist = math.sqrt((gx - rx) ** 2 + (gy - ry) ** 2)
            if dist < self.min_goal_distance or dist > self.max_goal_distance:
                continue
            # Blacklist checks (always apply)
            already_tried = False
            for tx, ty in self.tried_centroids:
                if math.sqrt((gx - tx) ** 2 + (gy - ty) ** 2) < self.blacklist_radius:
                    already_tried = True
                    break
            if already_tried:
                continue
            already_failed = False
            for fx, fy in self.failed_centroids:
                if math.sqrt((gx - fx) ** 2 + (gy - fy) ** 2) < self.blacklist_radius:
                    already_failed = True
                    break
            if already_failed:
                continue

            # Heading check: strict vs relaxed bucket
            goal_dir = math.atan2(gy - ry, gx - rx)
            heading_diff = abs(self.normalize_angle(goal_dir - ryaw))
            if heading_diff <= self.max_heading_diff:
                strict_goals.append((score, gx, gy, size))
            else:
                # Re-score without heading penalty for fair ranking
                fallback_score = (size * self.size_weight
                                 - dist * self.distance_weight)
                relaxed_goals.append((fallback_score, gx, gy, size))

        if strict_goals:
            return strict_goals
        if relaxed_goals:
            self.get_logger().info(
                f'No heading-feasible frontiers, relaxing heading constraint '
                f'for {len(relaxed_goals)} behind-robot frontiers')
            return relaxed_goals

        # Phase 2: long-range fallback — find distant frontier clusters
        # and generate intermediate waypoints toward them
        if self.long_range_enabled:
            long_range = self._long_range_fallback(clusters, msg, rx, ry)
            if long_range:
                return long_range

        return []

    def _long_range_fallback(self, clusters, msg, rx, ry):
        """Find large frontier clusters beyond max_goal_distance.

        Generates an intermediate waypoint at max_goal_distance in the direction
        of the largest distant clusters, enabling the robot to backtrack toward
        unexplored areas. Tries up to 3 clusters in descending size order.
        """
        grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))

        large_clusters = sorted(
            [c for c in clusters
             if len(c) >= self.long_range_min_cluster_size],
            key=len, reverse=True)
        if not large_clusters:
            return []

        for cluster in large_clusters[:3]:
            wxs = [self.cell_to_world(r, c, msg.info)[0] for r, c in cluster]
            wys = [self.cell_to_world(r, c, msg.info)[1] for r, c in cluster]
            target_x = sum(wxs) / len(wxs)
            target_y = sum(wys) / len(wys)

            dist = math.sqrt((target_x - rx) ** 2 + (target_y - ry) ** 2)
            if dist <= self.max_goal_distance:
                continue

            direction = math.atan2(target_y - ry, target_x - rx)
            wp_x = rx + self.max_goal_distance * math.cos(direction)
            wp_y = ry + self.max_goal_distance * math.sin(direction)

            snapped = self.snap_to_free(wp_x, wp_y, grid, msg.info, radius=20)
            if snapped is None:
                continue
            wp_x, wp_y = snapped

            wp_row, wp_col = self.world_to_cell(wp_x, wp_y, msg.info)
            if not self.has_obstacle_clearance(
                    wp_row, wp_col, grid, msg.info.height, msg.info.width,
                    self.goal_obstacle_clearance, msg.info.resolution):
                continue

            self.get_logger().info(
                f'Long-range fallback: cluster at ({target_x:.1f}, {target_y:.1f}) '
                f'[size={len(cluster)}, dist={dist:.1f}m], '
                f'waypoint ({wp_x:.1f}, {wp_y:.1f})')
            return [(0.0, wp_x, wp_y, len(cluster))]

        return []

    def _reposition_to_frontiers(self):
        """Navigate toward the weighted center of all remaining frontier clusters.

        Called when no goals are available but unknown ratio is still high.
        Moving the robot to a central location may reveal new frontiers.
        Returns True if a repositioning goal was sent.
        """
        if self.current_map is None:
            return False

        msg = self.current_map
        data = np.array(msg.data, dtype=np.int8)
        height, width = msg.info.height, msg.info.width
        grid = data.reshape((height, width))

        frontier_mask = self.detect_frontiers(grid, height, width)
        if not np.any(frontier_mask):
            return False

        clusters = self.cluster_frontiers(frontier_mask)
        if not clusters:
            return False

        # Compute size-weighted centroid of all frontier clusters
        total_weight = 0
        weighted_x = 0.0
        weighted_y = 0.0
        for cells in clusters:
            w = len(cells)
            wxs = [self.cell_to_world(r, c, msg.info)[0] for r, c in cells]
            wys = [self.cell_to_world(r, c, msg.info)[1] for r, c in cells]
            weighted_x += sum(wxs) / len(wxs) * w
            weighted_y += sum(wys) / len(wys) * w
            total_weight += w

        if total_weight == 0:
            return False

        target_x = weighted_x / total_weight
        target_y = weighted_y / total_weight

        # Snap to free cell
        snapped = self.snap_to_free(target_x, target_y, grid, msg.info, radius=20)
        if snapped is None:
            return False
        target_x, target_y = snapped

        # Check obstacle clearance
        trow, tcol = self.world_to_cell(target_x, target_y, msg.info)
        if not self.has_obstacle_clearance(
                trow, tcol, grid, height, width,
                self.goal_obstacle_clearance, msg.info.resolution):
            return False

        self.get_logger().info(
            f'Repositioning to frontier center ({target_x:.1f}, {target_y:.1f}) '
            f'[{len(clusters)} clusters, {total_weight} frontier cells]')
        self._repositioning = True
        self.tried_centroids.append((target_x, target_y))
        self.state = State.NAVIGATING
        self.send_goal(target_x, target_y)
        return True

    def publish_markers(self, goals, selected_idx=-1):
        markers = MarkerArray()

        # Clear old markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if not goals:
            self.marker_pub.publish(markers)
            return

        for i, (score, gx, gy, size) in enumerate(goals):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = gx
            m.pose.position.y = gy
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3

            if i == selected_idx:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
            elif i < 3:
                m.color.r = 1.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 0.8
            else:
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.5
            markers.markers.append(m)

        # Arrow for selected goal
        if 0 <= selected_idx < len(goals):
            _, gx, gy, _ = goals[selected_idx]
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'selected_goal'
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            pose = self.get_robot_pose()
            if pose:
                arrow.points.append(Point(x=pose[0], y=pose[1], z=0.3))
            arrow.points.append(Point(x=gx, y=gy, z=0.3))
            arrow.scale.x = 0.08
            arrow.scale.y = 0.15
            arrow.color.r = 0.0
            arrow.color.g = 1.0
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            markers.markers.append(arrow)

        self.marker_pub.publish(markers)

    def send_goal(self, gx, gy):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy

        pose = self.get_robot_pose()
        if pose:
            yaw = math.atan2(gy - pose[1], gx - pose[0])
        else:
            yaw = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending goal: ({gx:.2f}, {gy:.2f}, {math.degrees(yaw):.1f} deg)')

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(
                'Goal rejected by Nav2 (action server may still be activating)')
            if self.tried_centroids:
                self.tried_centroids.pop()
            self._nav2_ready = False
            self.current_goal_accepted = False
            self.state = State.IDLE
            return
        self.get_logger().info('Goal accepted')
        self.current_goal_accepted = True
        self.current_goal_handle = handle
        # Reset stuck tracking for new goal
        self.last_progress_pose = None
        self.last_progress_time = None
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        self.current_goal_accepted = False
        self.current_goal_handle = None
        self.last_progress_pose = None
        self.last_progress_time = None
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached successfully')
            self.tried_centroids.clear()
            self.failed_centroids.clear()
            self.global_retry_count = 0
            self.stuck_position = None
            self.stuck_at_position_count = 0
            if self._repositioning:
                self._repositioning = False
                self.get_logger().info(
                    'Repositioning complete, re-scanning for frontiers')
        else:
            self.get_logger().warn(f'Goal failed with status {result.status}')
            self._repositioning = False
            if self.tried_centroids:
                failed_goal = self.tried_centroids.pop()
                self.failed_centroids.append(failed_goal)
                fx, fy = failed_goal
                self.get_logger().info(
                    f'Blacklisted failed frontier ({fx:.2f}, {fy:.2f}), '
                    f'total blacklisted: {len(self.failed_centroids)}')
            # Track position-level stuck
            pose = self.get_robot_pose()
            if pose:
                self._update_stuck_position(pose)
        self.state = State.IDLE

    def feedback_callback(self, feedback_msg):
        pass

    def _update_stuck_position(self, pose):
        if pose is None:
            return
        if self.stuck_position is not None:
            dx = pose[0] - self.stuck_position[0]
            dy = pose[1] - self.stuck_position[1]
            if math.sqrt(dx * dx + dy * dy) > 2.0:
                self.stuck_position = None
                self.stuck_at_position_count = 0
                return
        self.stuck_position = (pose[0], pose[1])
        self.stuck_at_position_count += 1
        self.get_logger().warn(
            f'Stuck at position ({pose[0]:.2f}, {pose[1]:.2f}), '
            f'count={self.stuck_at_position_count}/{self.stuck_position_count}')

    def _check_stuck(self):
        pose = self.get_robot_pose()
        if pose is None:
            return

        now = self.get_clock().now()

        if self.last_progress_pose is None:
            self.last_progress_pose = pose
            self.last_progress_time = now
            return

        elapsed = (now - self.last_progress_time).nanoseconds / 1e9
        if elapsed < self.stuck_timeout:
            return

        dx = pose[0] - self.last_progress_pose[0]
        dy = pose[1] - self.last_progress_pose[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.stuck_distance:
            self.get_logger().warn(
                f'Robot stuck! Moved only {dist:.3f}m in {elapsed:.1f}s, '
                f'canceling goal')
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            self.current_goal_accepted = False
            self.last_progress_pose = None
            self.last_progress_time = None
            if self.tried_centroids:
                failed_goal = self.tried_centroids.pop()
                self.failed_centroids.append(failed_goal)
            # Track position-level stuck
            self._update_stuck_position(pose)
            self.state = State.IDLE
        else:
            self.last_progress_pose = pose
            self.last_progress_time = now

    def explore_step(self):
        if self.state == State.COMPLETED:
            return

        # Initial warmup: wait for map to accumulate meaningful data
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed < self.initial_warmup_seconds:
            return

        if self.state == State.NAVIGATING:
            self._check_stuck()
            return

        if self.current_map is None:
            return

        # Ensure Nav2 is ready before attempting exploration
        if not self._nav2_ready:
            if not self._nav2_ready_checked:
                self.get_logger().info(
                    f'Waiting for Nav2 action server (timeout={self.nav2_wait_timeout}s)...')
                self._nav2_ready_checked = True
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                self._nav2_ready = True
                self.get_logger().info('Nav2 action server is now available')
            elif elapsed > self.nav2_wait_timeout:
                self.get_logger().error(
                    'Nav2 action server not available within timeout, aborting')
                self.state = State.COMPLETED
                return
            else:
                return

        # Check position-level stuck BEFORE frontier search — if the robot
        # is physically stuck, finding frontiers is useless; reposition instead.
        if self.stuck_at_position_count >= self.stuck_position_count:
            self.get_logger().warn(
                f'Stuck at same position for {self.stuck_at_position_count} '
                f'consecutive goals, attempting recovery')
            self.stuck_at_position_count = 0
            self.stuck_position = None
            self.failed_centroids.clear()
            self.tried_centroids.clear()
            self.global_retry_count += 1
            if self.global_retry_count > self.max_global_retries:
                # Before giving up, check if coverage is still insufficient
                if self._unknown_ratio() > self.max_unknown_ratio:
                    self.get_logger().warn(
                        f'Stuck but coverage insufficient '
                        f'({self._unknown_ratio():.1%} unknown), '
                        f'resetting for repositioning attempt')
                    self.global_retry_count = 0
                    if self._reposition_to_frontiers():
                        return
                    # reposition failed, fall through to declare complete
                else:
                    self.get_logger().warn(
                        'All retries exhausted at stuck position, '
                        'declaring exploration complete')
                self.state = State.COMPLETED
                self.get_logger().info('=== EXPLORATION COMPLETE ===')
                self.publish_markers([])
                self.save_map()
                return
            # Try repositioning to frontier center to escape stuck position
            if self._reposition_to_frontiers():
                return
            return

        goals = self.find_frontiers()
        self.publish_markers(goals, selected_idx=0 if goals else -1)

        if not goals:
            self.consecutive_empty += 1
            self.get_logger().info(
                f'No frontiers found ({self.consecutive_empty}/{self.completion_threshold})')
            if self.consecutive_empty >= self.completion_threshold:
                # Coverage check: refuse to complete if too much unknown area remains
                if self._unknown_ratio() > self.max_unknown_ratio:
                    ratio = self._unknown_ratio()
                    self.get_logger().warn(
                        f'Map coverage insufficient: {ratio:.1%} unknown '
                        f'(threshold {self.max_unknown_ratio:.0%}), '
                        f'resetting completion counter')
                    self.consecutive_empty = 0
                    # Clear blacklist to give all frontiers another chance
                    self.failed_centroids.clear()
                    self.tried_centroids.clear()
                    # Try repositioning toward remaining frontiers
                    if self._reposition_to_frontiers():
                        return
                    return
                self.state = State.COMPLETED
                self.get_logger().info('=== EXPLORATION COMPLETE ===')
                self.publish_markers([])
                # Save map automatically
                self.save_map()
            return

        self.consecutive_empty = 0

        # Try best goals in order
        for i, (score, gx, gy, size) in enumerate(goals):
            self.get_logger().info(
                f'Attempting frontier #{i}: ({gx:.2f}, {gy:.2f}), '
                f'size={size}, score={score:.1f}')
            self.tried_centroids.append((gx, gy))
            self.state = State.NAVIGATING
            self.send_goal(gx, gy)
            return

        # All frontiers filtered out (tried or failed)
        if self.global_retry_count < self.max_global_retries:
            self.global_retry_count += 1
            self.get_logger().warn(
                f'All frontiers exhausted, clearing blacklist for retry '
                f'({self.global_retry_count}/{self.max_global_retries})')
            self.failed_centroids.clear()
            self.tried_centroids.clear()
            self.state = State.IDLE
        else:
            self.get_logger().warn(
                'All frontiers exhausted after full retry, '
                'declaring exploration complete')
            self.state = State.COMPLETED
            self.get_logger().info('=== EXPLORATION COMPLETE ===')
            self.publish_markers([])
            self.save_map()

    def _unknown_ratio(self):
        """Return the ratio of reachable unknown cells via flood-fill from robot."""
        if self.current_map is None:
            return 1.0
        data = np.array(self.current_map.data, dtype=np.int8)
        height = self.current_map.info.height
        width = self.current_map.info.width
        grid = data.reshape((height, width))

        pose = self.get_robot_pose()
        if pose is None:
            return 1.0
        rx, ry = pose[0], pose[1]
        col = int((rx - self.current_map.info.origin.position.x)
                  / self.current_map.info.resolution)
        row = int((ry - self.current_map.info.origin.position.y)
                  / self.current_map.info.resolution)

        if not (0 <= row < height and 0 <= col < width
                and grid[row, col] == 0):
            return 1.0

        visited = np.zeros((height, width), dtype=bool)
        queue = deque([(row, col)])
        visited[row, col] = True
        reachable_free = 0
        reachable_unknown = 0

        while queue:
            cy, cx = queue.popleft()
            reachable_free += 1
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                ny, nx = cy + dy, cx + dx
                if not (0 <= ny < height and 0 <= nx < width):
                    continue
                if visited[ny, nx]:
                    continue
                visited[ny, nx] = True
                if grid[ny, nx] == 0:
                    queue.append((ny, nx))
                elif grid[ny, nx] == -1:
                    reachable_unknown += 1

        total = reachable_free + reachable_unknown
        if total == 0:
            return 1.0
        return reachable_unknown / total

    def save_map(self):
        import subprocess
        project_dir = os.environ.get('LIDAR_SLAM_ROOT', '/home/hello/lidar-slam')
        map_path = os.path.join(project_dir, 'maps', 'auto_exploration_map')
        self.get_logger().info(f'Saving map to {map_path}')
        try:
            subprocess.Popen(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', map_path, '--ros-args', '-p', 'use_sim_time:=true'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to save map: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
