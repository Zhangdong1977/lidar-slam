#!/usr/bin/env python3
"""Vehicle state node for jvs-opentcs Sidecar integration.

Publishes full vehicle runtime state for the jvs-opentcs-ros2-sidecar to consume.
Sidecar subscribes to these ROS2 topics and exposes HTTP REST API to openTCS/JVS.

Data flow:
  Sidecar --/goal_pose--> this node --NavigateToPose--> Nav2
  Nav2 TF ----> this node --/amcl_pose----> Sidecar (10Hz)
                        \--/robot_state--> Sidecar (1Hz, JSON with 29+ fields)
                        \--/battery_state-> Sidecar (1Hz, simulated)
"""

import json
import hashlib
import math
import os
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, String
import tf2_ros

try:
    from nav2_collision_monitor.msg import CollisionMonitorState as CollisionMonitorStateMsg
    _HAS_CM_MSG = True
except ImportError:
    _HAS_CM_MSG = False


class VehicleState:
    IDLE = 'IDLE'
    WORKING = 'WORKING'
    CHARGING = 'CHARGING'
    PAUSED = 'PAUSED'
    ERROR = 'ERROR'
    OFFLINE = 'OFFLINE'
    MANUAL = 'MANUAL'


class DispatchStatus:
    UNKNOWN = 'UNKNOWN'
    ACCEPTED = 'ACCEPTED'
    EXECUTING = 'EXECUTING'
    SUCCEEDED = 'SUCCEEDED'
    CANCELING = 'CANCELING'
    CANCELED = 'CANCELED'
    ABORTED = 'ABORTED'
    BLOCKED = 'BLOCKED'
    REJECTED = 'REJECTED'


class LocalizationStatus:
    OK = 'OK'
    DEGRADED = 'DEGRADED'
    LOST = 'LOST'
    INITIALIZING = 'INITIALIZING'


# Nav2 action_msgs GoalStatus codes
GOAL_STATUS_UNKNOWN = 0
GOAL_STATUS_ACCEPTED = 1
GOAL_STATUS_EXECUTING = 2
GOAL_STATUS_CANCELING = 3
GOAL_STATUS_SUCCEEDED = 4
GOAL_STATUS_CANCELED = 5
GOAL_STATUS_ABORTED = 6


class OpentcsVehicleNode(LifecycleNode):
    def __init__(self):
        super().__init__('opentcs_vehicle_node')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        # --- Declare parameters ---
        self.declare_parameter('vehicle_name', 'ackermann_robot')
        vehicle_name = self.get_parameter('vehicle_name').value

        # access_identity falls back to vehicle_name when not explicitly set,
        # so multi-vehicle deployments only need to override vehicle_name.
        self.declare_parameter('access_identity', '')
        self.declare_parameter('namespace', '')
        self.declare_parameter('domain_id', 42)
        self.declare_parameter('base_frame', 'body_link')
        self.declare_parameter('map_frame', 'map')
        # Sidecar-facing topics: dynamically prefixed with vehicle_name
        self.declare_parameter('amcl_pose_topic', f'/{vehicle_name}/amcl_pose')
        self.declare_parameter('goal_pose_topic', f'/{vehicle_name}/goal_pose')
        self.declare_parameter('robot_state_topic', f'/{vehicle_name}/robot_state')
        self.declare_parameter('battery_state_topic', f'/{vehicle_name}/battery_state')
        # Nav2/internal topics: relative names resolve under namespace
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('pose_publish_rate', 10.0)
        self.declare_parameter('status_sample_ms', 1000)
        self.declare_parameter('heartbeat_timeout_ms', 30000)
        self.declare_parameter('cancel_timeout_ms', 5000)
        self.declare_parameter('max_speed', 1.4)
        self.declare_parameter('battery_sim_enabled', True)
        self.declare_parameter('battery_sim_start_percent', 95.0)
        self.declare_parameter('battery_sim_drain_rate', 5.0)
        self.declare_parameter('goal_order_id_parse', True)
        self.declare_parameter('emergency_stop_topic', '')
        self.declare_parameter('safety_stop_topic', '')
        self.declare_parameter('obstacle_detection_mode', 'collision_monitor')
        self.declare_parameter('obstacle_scan_threshold', 0.5)
        self.declare_parameter('obstacle_scan_angle_window', 1.047)
        self.declare_parameter('battery_real_topic', 'battery_state_real')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('amcl_subscribe_topic', 'amcl_pose')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('collision_monitor_state_topic', 'collision_monitor_state')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('global_costmap_topic', 'global_costmap/costmap')
        self.declare_parameter('position_report_topic', '')
        self.declare_parameter('map_yaml_file', '')
        self.declare_parameter('error_auto_recover_ms', 10000)
        self.declare_parameter('goal_bounds_check_enabled', True)
        self.declare_parameter('goal_bounds_tolerance', 0.5)
        self.declare_parameter('goal_reject_unknown_cost', True)

        # Ackermann heading alignment parameters
        self.declare_parameter('goal_heading_alignment_enabled', True)
        self.declare_parameter('goal_heading_alignment_tolerance', 0.15)
        self.declare_parameter('alignment_forward_speed', 0.3)
        self.declare_parameter('alignment_reverse_speed', 0.25)
        self.declare_parameter('alignment_angular_gain', 2.0)
        self.declare_parameter('alignment_max_omega', 0.8)
        self.declare_parameter('alignment_forward_dist', 0.8)
        self.declare_parameter('alignment_reverse_dist', 0.6)
        self.declare_parameter('alignment_max_iterations', 5)
        self.declare_parameter('alignment_stop_duration', 0.5)
        self.declare_parameter('alignment_timeout', 30.0)

        # Load parameters
        self._vehicle_name = self.get_parameter('vehicle_name').value
        self._access_identity = (
            self.get_parameter('access_identity').value or self._vehicle_name)
        self._base_frame = self.get_parameter('base_frame').value
        self._map_frame = self.get_parameter('map_frame').value
        pose_rate = self.get_parameter('pose_publish_rate').value
        status_ms = self.get_parameter('status_sample_ms').value
        battery_sim = self.get_parameter('battery_sim_enabled').value
        battery_start = self.get_parameter('battery_sim_start_percent').value
        battery_drain = self.get_parameter('battery_sim_drain_rate').value
        cancel_timeout_s = self.get_parameter('cancel_timeout_ms').value / 1000.0
        self._order_id_parse = self.get_parameter('goal_order_id_parse').value

        # --- Internal state ---
        self._state = VehicleState.IDLE
        self._dispatch_status = DispatchStatus.UNKNOWN
        self._localization_status = LocalizationStatus.INITIALIZING
        self._localization_score = 0.0
        self._fault_code = ''
        self._fault_message = ''

        # Goal tracking
        self._goal_handle = None
        self._send_goal_future = None
        self._current_goal_id = ''
        self._current_order_id = ''
        self._distance_remaining = 0.0
        self._estimated_time_remaining = 0.0
        self._cancel_timeout_s = cancel_timeout_s
        self._goal_generation = 0
        self._current_goal_yaw = 0.0

        # Heading alignment state machine
        self._alignment_active = False
        self._alignment_phase = 'IDLE'
        self._alignment_goal_yaw = 0.0
        self._alignment_start_x = 0.0
        self._alignment_start_y = 0.0
        self._alignment_iteration = 0
        self._alignment_stop_start = 0.0
        self._alignment_start_time = 0.0

        # Pose from TF
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._pose_quat_z = 0.0
        self._pose_quat_w = 1.0
        self._last_tf_time = 0.0

        # Velocity from /odom
        self._linear_velocity = 0.0
        self._angular_velocity = 0.0

        # AMCL covariance for localization quality
        self._amcl_cov_x = 0.25
        self._amcl_cov_y = 0.25
        self._amcl_cov_yaw = 0.15

        # Battery simulation
        self._battery_sim = battery_sim
        self._battery_start = battery_start
        self._battery_drain = battery_drain
        self._battery_percent = battery_start
        self._battery_charging = False
        self._start_time = time.monotonic()

        # Safety state
        self._emergency_stop = False
        self._safety_stop = False
        self._obstacle_detected = False

        # Position reporting
        self._current_position = ''
        self._last_node_id = ''
        self._next_position = ''

        # Heartbeat tracking
        self._last_goal_time = 0.0

        # ERROR auto-recovery
        self._error_auto_recover_s = self.get_parameter('error_auto_recover_ms').value / 1000.0
        self._error_enter_time = 0.0

        # 全局代价地图缓存（目标点边界校验）
        self._costmap_info = None
        self._costmap_data = None

        # Map checksum
        self._map_checksum = self._compute_map_checksum(
            self.get_parameter('map_yaml_file').value)

        # Store rate params for on_activate
        self._pose_rate = pose_rate
        self._status_ms = status_ms

        # Timer handles (for cleanup in on_deactivate)
        self._timers = []

        self.get_logger().info(
            f'opentcs_vehicle_node configured: vehicle={self._vehicle_name}, '
            f'access_identity={self._access_identity}, '
            f'base_frame={self._base_frame}, map_frame={self._map_frame}, '
            f'pose_rate={pose_rate}Hz, state_rate={1000/status_ms:.1f}Hz, '
            f'battery_sim={battery_sim}'
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- Publishers ---
        amcl_topic = self.get_parameter('amcl_pose_topic').value
        state_topic = self.get_parameter('robot_state_topic').value
        battery_topic = self.get_parameter('battery_state_topic').value

        self._amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, amcl_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self._battery_pub = self.create_publisher(
            BatteryState, battery_topic, 10)
        self._cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        # --- Subscribers ---
        goal_topic = self.get_parameter('goal_pose_topic').value
        self.create_subscription(PoseStamped, goal_topic, self._goal_cb, 10)
        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        amcl_sub_topic = self.get_parameter('amcl_subscribe_topic').value
        self.create_subscription(
            PoseWithCovarianceStamped, amcl_sub_topic,
            self._amcl_covariance_cb, 10)

        # Safety subscriptions (conditional)
        estop_topic = self.get_parameter('emergency_stop_topic').value
        if estop_topic:
            self.create_subscription(Bool, estop_topic, self._estop_cb, 10)
        sstop_topic = self.get_parameter('safety_stop_topic').value
        if sstop_topic:
            self.create_subscription(Bool, sstop_topic, self._safety_stop_cb, 10)

        # Obstacle detection subscription (conditional)
        obs_mode = self.get_parameter('obstacle_detection_mode').value
        if obs_mode == 'collision_monitor':
            scan_topic = self.get_parameter('scan_topic').value
            cm_topic = self.get_parameter('collision_monitor_state_topic').value
            if _HAS_CM_MSG:
                self.create_subscription(CollisionMonitorStateMsg,
                                         cm_topic,
                                         self._collision_monitor_cb, 10)
            else:
                self.get_logger().warn(
                    'nav2_collision_monitor.msg unavailable, falling back to scan mode')
                self.create_subscription(LaserScan, scan_topic,
                                         self._scan_obstacle_cb, 10)
        elif obs_mode == 'scan':
            scan_topic = self.get_parameter('scan_topic').value
            self.create_subscription(LaserScan, scan_topic, self._scan_obstacle_cb, 10)

        # Real battery subscription (when sim disabled)
        if not self._battery_sim:
            battery_real_topic = self.get_parameter('battery_real_topic').value
            self._battery_real_msg = None
            self.create_subscription(BatteryState, battery_real_topic,
                                     self._battery_real_cb, 10)

        # Position report subscription (optional, from sidecar)
        pos_report_topic = self.get_parameter('position_report_topic').value
        if pos_report_topic:
            self.create_subscription(String, pos_report_topic,
                                     self._position_report_cb, 10)

        # 全局代价地图订阅（目标点边界校验）
        costmap_topic = self.get_parameter('global_costmap_topic').value
        self.create_subscription(
            OccupancyGrid, costmap_topic,
            self._global_costmap_cb, 1)

        # --- Action client ---
        action_name = self.get_parameter('nav_action_name').value
        self._action_client = ActionClient(self, NavigateToPose, action_name)

        # --- Timers ---
        self._timers = [
            self.create_timer(1.0 / self._pose_rate, self._publish_pose),
            self.create_timer(self._status_ms / 1000.0, self._publish_robot_state),
            self.create_timer(1.0, self._publish_battery),
            self.create_timer(0.5, self._check_localization),
            self.create_timer(0.05, self._alignment_step),
        ]

        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        # Destroy all timers
        for timer in self._timers:
            self.destroy_timer(timer)
        self._timers = []

        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # =========================================================================
    # Subscriptions
    # =========================================================================

    def _global_costmap_cb(self, msg: OccupancyGrid):
        self._costmap_info = msg.info
        self._costmap_data = msg.data

    def _validate_goal_in_costmap(self, x: float, y: float) -> tuple:
        """检查目标点是否在全局代价地图有效范围内。"""
        if not self.get_parameter('goal_bounds_check_enabled').value:
            return True, ''

        if self._costmap_info is None:
            self.get_logger().warn(
                '全局代价地图缓存尚未建立，跳过边界检查',
                throttle_duration_sec=30.0)
            return True, ''

        info = self._costmap_info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution
        map_max_x = origin_x + info.width * resolution
        map_max_y = origin_y + info.height * resolution

        tolerance = self.get_parameter('goal_bounds_tolerance').value

        if (x < origin_x + tolerance or x > map_max_x - tolerance or
                y < origin_y + tolerance or y > map_max_y - tolerance):
            reason = (f'目标点 ({x:.2f}, {y:.2f}) 超出地图边界 '
                      f'x=[{origin_x:.2f}, {map_max_x:.2f}], '
                      f'y=[{origin_y:.2f}, {map_max_y:.2f}], '
                      f'容忍度={tolerance:.2f}m')
            return False, reason

        if self.get_parameter('goal_reject_unknown_cost').value and self._costmap_data:
            cell_x = int((x - origin_x) / resolution)
            cell_y = int((y - origin_y) / resolution)
            cell_x = max(0, min(cell_x, info.width - 1))
            cell_y = max(0, min(cell_y, info.height - 1))
            idx = cell_y * info.width + cell_x
            if idx < len(self._costmap_data):
                cost = self._costmap_data[idx]
                if cost < 0:
                    reason = (f'目标点 ({x:.2f}, {y:.2f}) 位于未知区域 '
                              f'(cost={cost})')
                    return False, reason
                if cost >= 90:
                    reason = (f'目标点 ({x:.2f}, {y:.2f}) 位于障碍区域 '
                              f'(cost={cost})')
                    return False, reason

        return True, ''

    def _odom_cb(self, msg: Odometry):
        self._linear_velocity = msg.twist.twist.linear.x
        self._angular_velocity = msg.twist.twist.angular.z

    def _amcl_covariance_cb(self, msg: PoseWithCovarianceStamped):
        # Extract covariance diagonal for localization quality
        c = msg.pose.covariance
        self._amcl_cov_x = c[0]    # x variance
        self._amcl_cov_y = c[7]    # y variance
        self._amcl_cov_yaw = c[35] # yaw variance

    def _estop_cb(self, msg: Bool):
        self._emergency_stop = msg.data

    def _safety_stop_cb(self, msg: Bool):
        self._safety_stop = msg.data

    def _collision_monitor_cb(self, msg):
        # CollisionMonitorState.msg: state field is "CLEAR"/"APPROACH"/"STOP"
        self._obstacle_detected = (msg.state not in ('CLEAR', ''))

    def _scan_obstacle_cb(self, msg: LaserScan):
        threshold = self.get_parameter('obstacle_scan_threshold').value
        half_window = self.get_parameter('obstacle_scan_angle_window').value / 2.0
        self._obstacle_detected = any(
            msg.range_min < r < threshold
            for i, r in enumerate(msg.ranges)
            if abs(msg.angle_min + i * msg.angle_increment) <= half_window
        )

    def _position_report_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'nodeId' in data:
                self._last_node_id = data['nodeId']
            if 'currentPosition' in data:
                self._current_position = data['currentPosition']
            if 'nextPosition' in data:
                self._next_position = data['nextPosition']
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn(f'Invalid position report: {msg.data}')

    @staticmethod
    def _compute_map_checksum(map_yaml_path: str) -> str:
        if not map_yaml_path or not os.path.exists(map_yaml_path):
            return ''
        pgm_file = None
        try:
            with open(map_yaml_path, 'r') as f:
                for line in f:
                    if line.startswith('image:'):
                        pgm_file = line.split(':', 1)[1].strip().strip('"').strip("'")
                        break
        except OSError:
            return ''
        if not pgm_file:
            return ''
        pgm_path = os.path.join(os.path.dirname(map_yaml_path), pgm_file)
        if not os.path.exists(pgm_path):
            return ''
        sha = hashlib.sha256()
        with open(pgm_path, 'rb') as f:
            for chunk in iter(lambda: f.read(8192), b''):
                sha.update(chunk)
        return sha.hexdigest()[:16]

    # =========================================================================
    # Goal management (Nav2 action)
    # =========================================================================

    def _goal_cb(self, msg: PoseStamped):
        # Cancel any active heading alignment
        if self._alignment_active:
            self.get_logger().info('Canceling alignment due to new goal')
            self._alignment_active = False
            self._alignment_phase = 'IDLE'
            self._publish_cmd_vel(0.0, 0.0)

        # Save goal yaw for post-navigation heading alignment
        q = msg.pose.orientation
        self._current_goal_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.get_logger().info(
            f'Goal received on {self.get_parameter("goal_pose_topic").value}: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), '
            f'current_state={self._state}, dispatch={self._dispatch_status}')

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server not available')
            self._set_dispatch(DispatchStatus.REJECTED)
            self._set_fault('NAV2_UNAVAILABLE', 'NavigateToPose action server not available')
            return

        # 目标点地图边界校验
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        is_valid, reason = self._validate_goal_in_costmap(goal_x, goal_y)
        if not is_valid:
            self.get_logger().warn(reason)
            self._set_dispatch(DispatchStatus.REJECTED)
            self._set_fault('GOAL_OUT_OF_BOUNDS', reason)
            return

        # Parse orderId from frame_id (format: "map/orderId=TO-xxx")
        if self._order_id_parse:
            raw_frame = msg.header.frame_id or 'map'
            if '/orderId=' in raw_frame:
                self._current_order_id = raw_frame.split('/orderId=')[1].split('/')[0]
                msg.header.frame_id = raw_frame.split('/orderId=')[0]
            else:
                self._current_order_id = ''
        else:
            self._current_order_id = ''

        self._last_goal_time = time.monotonic()
        self._goal_generation += 1

        # Cancel previous goal if active
        if self._goal_handle is not None:
            self.get_logger().info('Canceling previous goal before accepting new one')
            self._cancel_goal_internal()

        # Generate goal ID
        self._current_goal_id = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1000000
        if self._current_goal_id == 0:
            self._current_goal_id = int(time.time() * 1000)

        # State transitions
        self._set_state(VehicleState.WORKING)
        self._set_dispatch(DispatchStatus.ACCEPTED)
        self._clear_fault()

        goal = NavigateToPose.Goal()
        goal.pose = msg

        self.get_logger().info(
            f'Sending goal {self._current_goal_id}: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        self._send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        self._send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'send_goal_async failed: {e}')
            self._set_dispatch(DispatchStatus.REJECTED)
            self._set_fault('SEND_GOAL_FAILED', str(e))
            self._set_state(VehicleState.ERROR)
            self._goal_handle = None
            return
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal {self._current_goal_id} rejected by Nav2')
            self._set_dispatch(DispatchStatus.REJECTED)
            self._set_fault('GOAL_REJECTED', 'Nav2 rejected the navigation goal')
            self._set_state(VehicleState.IDLE)
            self._goal_handle = None
            return

        self._goal_handle = goal_handle
        self._set_dispatch(DispatchStatus.EXECUTING)
        self.get_logger().info(f'Goal {self._current_goal_id} accepted, EXECUTING')
        gen = self._goal_generation
        goal_handle.get_result_async().add_done_callback(
            lambda future: self._result_cb(future, gen)
        )

    def _feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._distance_remaining = feedback.distance_remaining
        etr = feedback.estimated_time_remaining
        self._estimated_time_remaining = etr.sec + etr.nanosec * 1e-9

    def _result_cb(self, future, generation):
        if generation != self._goal_generation:
            return

        result = future.result()
        status = result.status
        self.get_logger().info(f'Goal {self._current_goal_id} finished with status: {status}')

        # Nav2 is done; clear handle and metrics regardless of outcome
        self._goal_handle = None
        self._distance_remaining = 0.0
        self._estimated_time_remaining = 0.0

        if status == GOAL_STATUS_SUCCEEDED:
            # Check if heading alignment is needed
            if self._should_align_heading():
                self._start_alignment()
                return  # Will report success after alignment completes
            self._set_dispatch(DispatchStatus.SUCCEEDED)
            self._set_state(VehicleState.IDLE)
            self._current_order_id = ''
            self._current_goal_id = ''
        elif status == GOAL_STATUS_CANCELED:
            self._set_dispatch(DispatchStatus.CANCELED)
            self._set_state(VehicleState.IDLE)
            self._current_order_id = ''
            self._current_goal_id = ''
        elif status == GOAL_STATUS_ABORTED:
            self._set_dispatch(DispatchStatus.ABORTED)
            self._set_fault('NAV2_ABORTED', f'Nav2 aborted goal with status {status}')
            self._set_state(VehicleState.ERROR)
            self._current_order_id = ''
            self._current_goal_id = ''
        else:
            self._set_dispatch(DispatchStatus.ABORTED)
            self._set_fault('NAV2_UNKNOWN', f'Nav2 finished with unknown status {status}')
            self._set_state(VehicleState.ERROR)
            self._current_order_id = ''
            self._current_goal_id = ''

    def _cancel_goal_internal(self):
        if self._goal_handle is not None:
            self._set_dispatch(DispatchStatus.CANCELING)
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            self._current_goal_id = ''
            self._current_order_id = ''

    def cancel_current_goal(self):
        self._cancel_goal_internal()

    # =========================================================================
    # Ackermann heading alignment (multi-point turn)
    # =========================================================================

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _should_align_heading(self):
        if not self.get_parameter('goal_heading_alignment_enabled').value:
            return False
        heading_error = abs(self._normalize_angle(
            self._current_goal_yaw - self._pose_yaw))
        tolerance = self.get_parameter('goal_heading_alignment_tolerance').value
        return heading_error > tolerance

    def _start_alignment(self):
        self._alignment_active = True
        self._alignment_phase = 'FORWARD'
        self._alignment_goal_yaw = self._current_goal_yaw
        self._alignment_start_x = self._pose_x
        self._alignment_start_y = self._pose_y
        self._alignment_iteration = 0
        self._alignment_start_time = time.monotonic()
        self._set_dispatch(DispatchStatus.EXECUTING)
        heading_err = self._normalize_angle(
            self._alignment_goal_yaw - self._pose_yaw)
        self.get_logger().info(
            f'Starting heading alignment: current_yaw='
            f'{math.degrees(self._pose_yaw):.1f}°, '
            f'goal_yaw={math.degrees(self._alignment_goal_yaw):.1f}°, '
            f'error={math.degrees(heading_err):.1f}°')

    def _finish_alignment(self, success):
        self._alignment_active = False
        self._alignment_phase = 'IDLE'
        self._publish_cmd_vel(0.0, 0.0)
        heading_err = self._normalize_angle(
            self._alignment_goal_yaw - self._pose_yaw)
        if success:
            self.get_logger().info(
                f'Heading alignment succeeded: yaw='
                f'{math.degrees(self._pose_yaw):.1f}°, '
                f'remaining_error={math.degrees(heading_err):.1f}°, '
                f'iterations={self._alignment_iteration}')
        else:
            self.get_logger().warn(
                f'Heading alignment gave up: yaw='
                f'{math.degrees(self._pose_yaw):.1f}°, '
                f'remaining_error={math.degrees(heading_err):.1f}°, '
                f'iterations={self._alignment_iteration}')
        self._set_dispatch(DispatchStatus.SUCCEEDED)
        self._set_state(VehicleState.IDLE)
        self._current_order_id = ''
        self._current_goal_id = ''

    def _alignment_step(self):
        if not self._alignment_active:
            return

        now = time.monotonic()
        timeout = self.get_parameter('alignment_timeout').value
        if now - self._alignment_start_time > timeout:
            self._finish_alignment(False)
            return

        heading_error = self._normalize_angle(
            self._alignment_goal_yaw - self._pose_yaw)
        tolerance = self.get_parameter('goal_heading_alignment_tolerance').value

        if abs(heading_error) < tolerance:
            self._finish_alignment(True)
            return

        max_iter = self.get_parameter('alignment_max_iterations').value
        if self._alignment_iteration >= max_iter:
            self._finish_alignment(False)
            return

        # P-control for angular velocity
        gain = self.get_parameter('alignment_angular_gain').value
        max_omega = self.get_parameter('alignment_max_omega').value
        omega = max(-max_omega, min(max_omega, gain * heading_error))

        if self._alignment_phase == 'FORWARD':
            dist = math.hypot(self._pose_x - self._alignment_start_x,
                              self._pose_y - self._alignment_start_y)
            fwd_dist = self.get_parameter('alignment_forward_dist').value
            if dist >= fwd_dist:
                self._publish_cmd_vel(0.0, 0.0)
                self._alignment_phase = 'STOP1'
                self._alignment_stop_start = now
            else:
                fwd_speed = self.get_parameter('alignment_forward_speed').value
                self._publish_cmd_vel(fwd_speed, omega)

        elif self._alignment_phase == 'STOP1':
            stop_dur = self.get_parameter('alignment_stop_duration').value
            if now - self._alignment_stop_start > stop_dur:
                self._alignment_phase = 'REVERSE'
                self._alignment_start_x = self._pose_x
                self._alignment_start_y = self._pose_y

        elif self._alignment_phase == 'REVERSE':
            dist = math.hypot(self._pose_x - self._alignment_start_x,
                              self._pose_y - self._alignment_start_y)
            rev_dist = self.get_parameter('alignment_reverse_dist').value
            if dist >= rev_dist:
                self._publish_cmd_vel(0.0, 0.0)
                self._alignment_phase = 'STOP2'
                self._alignment_stop_start = now
            else:
                rev_speed = self.get_parameter('alignment_reverse_speed').value
                self._publish_cmd_vel(-rev_speed, omega)

        elif self._alignment_phase == 'STOP2':
            stop_dur = self.get_parameter('alignment_stop_duration').value
            if now - self._alignment_stop_start > stop_dur:
                self._alignment_iteration += 1
                self._alignment_phase = 'FORWARD'
                self._alignment_start_x = self._pose_x
                self._alignment_start_y = self._pose_y

    def _publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    # =========================================================================
    # State management
    # =========================================================================

    def _set_state(self, new_state):
        if self._state != new_state:
            self.get_logger().info(f'State: {self._state} -> {new_state}')
            self._state = new_state
            if new_state == VehicleState.ERROR:
                self._error_enter_time = time.monotonic()

    def _set_dispatch(self, new_status):
        if self._dispatch_status != new_status:
            self.get_logger().info(f'Dispatch: {self._dispatch_status} -> {new_status}')
            self._dispatch_status = new_status

    def _set_fault(self, code, message):
        self._fault_code = code
        self._fault_message = message

    def _clear_fault(self):
        if self._fault_code:
            self._fault_code = ''
            self._fault_message = ''

    # =========================================================================
    # Localization quality
    # =========================================================================

    def _check_localization(self):
        now = time.monotonic()

        # ERROR auto-recovery: return to IDLE after timeout
        if (self._state == VehicleState.ERROR
                and self._error_auto_recover_s > 0
                and self._error_enter_time > 0
                and now - self._error_enter_time > self._error_auto_recover_s):
            self.get_logger().info(
                f'Auto-recovering from ERROR after {self._error_auto_recover_s:.0f}s '
                f'(fault was: {self._fault_code}: {self._fault_message})')
            self._set_state(VehicleState.IDLE)
            self._set_dispatch(DispatchStatus.UNKNOWN)
            self._clear_fault()

        tf_age = now - self._last_tf_time

        if self._last_tf_time == 0.0 or tf_age > 2.0:
            self._localization_status = LocalizationStatus.INITIALIZING
            self._localization_score = 0.0
            return

        cov_x = self._amcl_cov_x
        cov_y = self._amcl_cov_y
        cov_yaw = self._amcl_cov_yaw

        if cov_x > 1.0 or cov_y > 1.0 or cov_yaw > 0.5:
            self._localization_status = LocalizationStatus.LOST
            self._localization_score = 0.1
        elif cov_x > 0.25 or cov_y > 0.25 or cov_yaw > 0.15:
            self._localization_status = LocalizationStatus.DEGRADED
            self._localization_score = 0.5
        else:
            self._localization_status = LocalizationStatus.OK
            self._localization_score = max(0.8, 1.0 - (cov_x + cov_y + cov_yaw) * 2)

        # Heartbeat check
        if self._last_goal_time > 0:
            heartbeat_s = self.get_parameter('heartbeat_timeout_ms').value / 1000.0
            elapsed = time.monotonic() - self._last_goal_time
            if elapsed > heartbeat_s:
                self.get_logger().warn(
                    f'Sidecar heartbeat timeout: no goal received in {elapsed:.0f}s',
                    throttle_duration_sec=60.0)

    # =========================================================================
    # Publishers
    # =========================================================================

    def _publish_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return

        self._last_tf_time = time.monotonic()

        self._pose_x = t.transform.translation.x
        self._pose_y = t.transform.translation.y
        q = t.transform.rotation
        self._pose_quat_z = q.z
        self._pose_quat_w = q.w
        # Extract yaw from quaternion
        self._pose_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = self._pose_x
        msg.pose.pose.position.y = self._pose_y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation = q
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = self._amcl_cov_x
        msg.pose.covariance[7] = self._amcl_cov_y
        msg.pose.covariance[35] = self._amcl_cov_yaw

        self._amcl_pub.publish(msg)

    def _publish_robot_state(self):
        timestamp = int(time.time())

        state_dict = {
            'vehicleName': self._vehicle_name,
            'vehicleId': self._vehicle_name,
            'accessIdentity': self._access_identity,
            'timestamp': timestamp,
            'frameId': self._map_frame,
            'x': round(self._pose_x, 4),
            'y': round(self._pose_y, 4),
            'yaw': round(self._pose_yaw, 6),
            'quaternionZ': round(self._pose_quat_z, 6),
            'quaternionW': round(self._pose_quat_w, 6),
            'linearVelocity': round(self._linear_velocity, 4),
            'angularVelocity': round(self._angular_velocity, 4),
            'battery': round(self._battery_percent, 1),
            'charging': self._battery_charging,
            'state': self._state,
            'dispatchStatus': self._dispatch_status,
            'localizationStatus': self._localization_status,
            'localizationScore': round(self._localization_score, 2),
            'emergencyStop': self._emergency_stop,
            'safetyStop': self._safety_stop,
            'obstacleDetected': self._obstacle_detected,
            'faultCode': self._fault_code,
            'faultMessage': self._fault_message,
            'currentTransportOrder': self._current_order_id,
            'goalId': str(self._current_goal_id),
            'distanceRemaining': round(self._distance_remaining, 2),
            'estimatedTimeRemaining': round(self._estimated_time_remaining, 1),
            'currentPosition': self._current_position,
            'lastNodeId': self._last_node_id,
            'nextPosition': self._next_position,
            'mapChecksum': self._map_checksum,
            'maxSpeed': self.get_parameter('max_speed').value,
        }

        msg = String()
        msg.data = json.dumps(state_dict, ensure_ascii=False)
        self._state_pub.publish(msg)

    def _battery_real_cb(self, msg: BatteryState):
        self._battery_real_msg = msg
        self._battery_percent = msg.percentage
        self._battery_charging = (
            msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        )

    def _publish_battery(self):
        if not self._battery_sim:
            if self._battery_real_msg is not None:
                self._battery_pub.publish(self._battery_real_msg)
            else:
                self.get_logger().warn(
                    'Battery sim disabled but no real battery data received',
                    throttle_duration_sec=30.0)
            return

        elapsed_hours = (time.monotonic() - self._start_time) / 3600.0
        self._battery_percent = max(
            0.0,
            self._battery_start - self._battery_drain * elapsed_hours
        )

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = 48.0 * (self._battery_percent / 100.0)
        msg.percentage = self._battery_percent
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.present = True

        self._battery_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpentcsVehicleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
