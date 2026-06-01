"""Material action server node with PyQt signal bridge.

Hosts LoadMaterials and UnloadMaterials action servers.
Uses ActionBridge (QObject) to emit pyqtSignal when goals arrive,
allowing the Qt GUI to update from the main thread.

Key design:
  - GoalState atomic state machine prevents cancel/submit/timeout races.
  - Per-goal timeout read from ROS2 parameters.
  - Cancel path uses goal_handle.canceled() (not succeed()).
"""

import threading
import time
from enum import Enum

from PyQt5 import QtCore

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from jvs_agv_material_msgs.action import LoadMaterials, UnloadMaterials
from jvs_agv_material_msgs.msg import MaterialActual


class ActionType(Enum):
    LOAD = 'LOAD'
    UNLOAD = 'UNLOAD'


class GoalState(Enum):
    """Atomic lifecycle for a single goal execution.

    Only one transition can win — protected by _goal_lock.
    """
    IDLE = 'IDLE'               # No goal active
    ACTIVE = 'ACTIVE'           # Goal accepted, waiting for GUI/timeout
    CANCELLING = 'CANCELLING'   # Cancel requested
    SUBMITTING = 'SUBMITTING'   # GUI submitted result
    TIMED_OUT = 'TIMED_OUT'     # Timeout expired
    COMPLETED = 'COMPLETED'     # Result returned to ROS2


class GoalData:
    """Holds deserialized goal information for the GUI."""

    def __init__(self, action_type: ActionType, goal_handle):
        self.action_type = action_type
        self.goal_handle = goal_handle
        goal = goal_handle.request
        self.job_id = goal.job_id
        self.request_id = goal.request_id
        self.order_id = goal.order_id
        self.order_no = goal.order_no
        self.step_no = goal.step_no
        self.vehicle_name = goal.vehicle_name
        self.point_id = goal.point_id
        self.location_id = goal.location_id
        self.materials = []
        for m in goal.materials:
            self.materials.append({
                'material_code': m.material_code,
                'material_name': m.material_name,
                'quantity': m.quantity,
                'unit': m.unit,
                'batch_no': m.batch_no,
                'container_code': m.container_code,
                'inventory_id': m.inventory_id,
            })
        self.trace_id = goal.trace_id


class ActionBridge(QtCore.QObject):
    """Bridge between ROS2 executor thread and Qt main thread.

    All signals are emitted from the executor thread and delivered
    to slots on the Qt main thread via queued connections.
    """

    # (action_type_str, goal_data_dict)
    goal_received = QtCore.pyqtSignal(str, dict)
    # (phase, progress, message)
    feedback_updated = QtCore.pyqtSignal(str, float, str)
    # cancelled
    action_cancelled = QtCore.pyqtSignal()
    # action completed — whether succeeded, failed, cancelled, or timed out
    action_finished = QtCore.pyqtSignal(str, str)


class MaterialActionServer(Node):
    """ROS2 node hosting LoadMaterials and UnloadMaterials action servers."""

    def __init__(self):
        super().__init__('material_action_server')
        self._bridge = ActionBridge()
        self._cb_group = ReentrantCallbackGroup()

        # ---- Goal state machine (protected by _goal_lock) ----
        self._goal_lock = threading.Lock()
        self._current_goal: GoalData = None
        self._goal_state = GoalState.IDLE
        self._goal_event = threading.Event()  # generic wake signal

        # Result data set by GUI (under _goal_lock)
        self._result_status = ''
        self._result_message = ''
        self._result_materials = []  # list of (checked: bool, material: dict)

        # ---- Parameters ----
        self.declare_parameter('vehicle_name', 'ackermann_robot')
        self._vehicle_name = self.get_parameter('vehicle_name').value

        self.declare_parameter('action_timeout_ms', 300000)
        self.declare_parameter('load_timeout_ms', -1)    # -1 = use action_timeout_ms
        self.declare_parameter('unload_timeout_ms', -1)   # -1 = use action_timeout_ms

        self._default_timeout_ms = self.get_parameter('action_timeout_ms').value
        self._load_timeout_ms = self.get_parameter('load_timeout_ms').value
        self._unload_timeout_ms = self.get_parameter('unload_timeout_ms').value

        # ---- Action servers ----
        self._load_server = ActionServer(
            self, LoadMaterials, 'load_materials',
            execute_callback=self._execute_load,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self._unload_server = ActionServer(
            self, UnloadMaterials, 'unload_materials',
            execute_callback=self._execute_unload,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            'Material action server started (vehicle=%s, timeout=%dms)'
            % (self._vehicle_name, self._default_timeout_ms))

    @property
    def bridge(self) -> ActionBridge:
        return self._bridge

    # ---- ROS2 action callbacks ----

    def _goal_callback(self, goal_request):
        """Validate incoming goal. Accept or reject."""
        if not goal_request.job_id:
            self.get_logger().warn('Rejecting goal: empty job_id')
            return GoalResponse.REJECT
        if not goal_request.materials or len(goal_request.materials) == 0:
            self.get_logger().warn('Rejecting goal: empty materials')
            return GoalResponse.REJECT
        # vehicle_name validation (warn but don't reject for flexibility)
        if (goal_request.vehicle_name
                and goal_request.vehicle_name != self._vehicle_name):
            self.get_logger().warn(
                'Goal vehicle_name=%s does not match local=%s'
                % (goal_request.vehicle_name, self._vehicle_name))
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Handle cancel request with atomic state check."""
        with self._goal_lock:
            if self._goal_state != GoalState.ACTIVE:
                self.get_logger().info(
                    'Cancel rejected: goal state=%s' % self._goal_state.value)
                return CancelResponse.REJECT
            self._goal_state = GoalState.CANCELLING
        self._goal_event.set()
        self._bridge.action_cancelled.emit()
        self.get_logger().info('Cancel accepted')
        return CancelResponse.ACCEPT

    def _execute_load(self, goal_handle):
        return self._execute(goal_handle, ActionType.LOAD, LoadMaterials)

    def _execute_unload(self, goal_handle):
        return self._execute(goal_handle, ActionType.UNLOAD, UnloadMaterials)

    def _execute(self, goal_handle, action_type: ActionType, action_cls):
        """Core execution: notify GUI, wait for operator/timeout/cancel, return result."""
        # ---- 1. Initialize state ----
        with self._goal_lock:
            self._current_goal = GoalData(action_type, goal_handle)
            self._goal_state = GoalState.ACTIVE
            self._goal_event.clear()
            self._result_status = ''
            self._result_message = ''
            self._result_materials = []

        goal_start_time = time.monotonic()

        # ---- 2. Resolve timeout ----
        timeout_ms = self._default_timeout_ms
        if action_type == ActionType.LOAD and self._load_timeout_ms > 0:
            timeout_ms = self._load_timeout_ms
        elif action_type == ActionType.UNLOAD and self._unload_timeout_ms > 0:
            timeout_ms = self._unload_timeout_ms
        timeout_sec = timeout_ms / 1000.0

        # ---- 3. Notify GUI ----
        goal_data = self._current_goal
        goal_dict = {
            'action_type': action_type.value,
            'job_id': goal_data.job_id,
            'request_id': goal_data.request_id,
            'order_id': goal_data.order_id,
            'order_no': goal_data.order_no,
            'step_no': goal_data.step_no,
            'vehicle_name': goal_data.vehicle_name,
            'point_id': goal_data.point_id,
            'location_id': goal_data.location_id,
            'materials': goal_data.materials,
            'trace_id': goal_data.trace_id,
        }
        self._bridge.goal_received.emit(action_type.value, goal_dict)

        # ---- 4. Initial feedback phases (P1) ----
        self._publish_feedback(goal_handle, 'ACCEPTED', 0.0,
            'Goal accepted, vehicle=%s' % goal_data.vehicle_name)
        self._publish_feedback(goal_handle, 'CHECKING', 0.05,
            'Checking %d material items' % len(goal_data.materials))
        self._publish_feedback(goal_handle, 'MOVING_TO_STATION', 0.1,
            'Navigating to %s/%s' % (goal_data.point_id, goal_data.location_id))
        self._publish_feedback(goal_handle, 'MOVING_ACTUATOR', 0.15,
            'Actuator positioning (%s)' % action_type.value)
        self._publish_feedback(goal_handle,
            'PICKING' if action_type == ActionType.LOAD else 'PLACING',
            0.2, 'Waiting for operator confirmation')

        # ---- 5. Poll loop with timeout ----
        poll_interval = 0.5  # seconds
        while True:
            # Check timeout
            elapsed = time.monotonic() - goal_start_time
            remaining = timeout_sec - elapsed
            if remaining <= 0:
                with self._goal_lock:
                    if self._goal_state == GoalState.ACTIVE:
                        self._goal_state = GoalState.TIMED_OUT
                self._goal_event.set()
                break

            # Wait for any wake signal
            wait_time = min(poll_interval, remaining + 0.05)
            self._goal_event.wait(timeout=wait_time)

            with self._goal_lock:
                state = self._goal_state

            if state in (GoalState.CANCELLING, GoalState.SUBMITTING,
                         GoalState.TIMED_OUT):
                break
            # else: spurious wake or still ACTIVE, loop again

        # ---- 6. Branch on terminal state ----
        with self._goal_lock:
            state = self._goal_state

        if state == GoalState.CANCELLING:
            return self._handle_cancel(goal_handle, action_type, action_cls,
                                       goal_start_time)
        elif state == GoalState.TIMED_OUT:
            return self._handle_timeout(goal_handle, action_type, action_cls,
                                        goal_start_time)
        else:
            return self._handle_submit(goal_handle, action_type, action_cls)

    # ---- Terminal state handlers ----

    def _handle_cancel(self, goal_handle, action_type, action_cls,
                       start_time: float):
        """Build CANCELLED result."""
        self._publish_feedback(goal_handle, 'COMPLETING', 1.0, 'Cancelled')
        result = action_cls.Result()
        result.success = False
        result.status = 'CANCELLED'
        result.message = self._result_message or 'Cancelled by operator'
        result.error_code = ''
        result.current_load = self._build_zero_actual()
        if action_type == ActionType.LOAD:
            result.actual_loaded = list(result.current_load)
        else:
            result.actual_unloaded = list(result.current_load)
        goal_handle.canceled()
        self.get_logger().info('Action %s cancelled' % action_type.value)
        self._bridge.action_finished.emit('CANCELLED', result.message)
        self._cleanup_goal()
        return result

    def _handle_timeout(self, goal_handle, action_type, action_cls,
                        start_time: float):
        """Build TIMEOUT result."""
        elapsed = time.monotonic() - start_time
        self._publish_feedback(goal_handle, 'COMPLETING', 1.0,
            'Timeout after %.1fs' % elapsed)
        result = action_cls.Result()
        result.success = False
        result.status = 'TIMEOUT'
        result.message = 'Action timed out (%.1fs)' % elapsed
        result.error_code = 'MATERIAL_ACTION_TIMEOUT'
        result.current_load = self._build_zero_actual()
        if action_type == ActionType.LOAD:
            result.actual_loaded = list(result.current_load)
        else:
            result.actual_unloaded = list(result.current_load)
        goal_handle.succeed()  # terminal state, we return the result
        self.get_logger().warn('Action %s timed out (%.1fs)'
                               % (action_type.value, elapsed))
        self._bridge.action_finished.emit('TIMEOUT', result.message)
        self._cleanup_goal()
        return result

    def _handle_submit(self, goal_handle, action_type, action_cls):
        """Build SUCCEEDED/PARTIAL/FAILED result from GUI state."""
        self._publish_feedback(goal_handle, 'VERIFYING', 0.9,
                               'Verifying results')
        self._publish_feedback(goal_handle, 'COMPLETING', 1.0, 'Completing')

        status = self._result_status
        message = self._result_message

        result = action_cls.Result()
        result.message = message
        result.error_code = ''

        actual_list = self._build_actual_from_materials()

        if status == 'FAILED':
            result.success = False
            result.status = 'FAILED'
            result.error_code = ('LOAD_FAILED' if action_type == ActionType.LOAD
                                 else 'UNLOAD_FAILED')
            for ma in actual_list:
                ma.status = 'FAILED'
                ma.actual_quantity = 0.0
        elif status == 'SUCCEEDED':
            result.success = True
            result.status = 'SUCCEEDED'
        elif status == 'PARTIAL':
            result.success = True
            result.status = 'PARTIAL'
        else:
            result.success = False
            result.status = status

        if action_type == ActionType.LOAD:
            result.actual_loaded = actual_list
        else:
            result.actual_unloaded = actual_list

        result.current_load = list(actual_list)

        goal_handle.succeed()
        self.get_logger().info(
            'Action %s completed: status=%s' % (action_type.value, result.status))
        self._bridge.action_finished.emit(result.status, result.message)
        self._cleanup_goal()
        return result

    # ---- Helpers ----

    def _publish_feedback(self, goal_handle, phase, progress, message):
        """Publish feedback on the goal handle."""
        try:
            fb = goal_handle.create_feedback()
            fb.phase = phase
            fb.progress = progress
            fb.message = message
            goal_handle.publish_feedback(fb)
        except Exception:
            pass
        self._bridge.feedback_updated.emit(phase, progress, message)

    def _build_actual_from_materials(self):
        """Build MaterialActual list from GUI submit data."""
        actual_list = []
        for checked, mat in self._result_materials:
            ma = MaterialActual()
            ma.material_code = mat.get('material_code', '')
            ma.material_name = mat.get('material_name', '')
            ma.actual_quantity = mat.get('quantity', 0.0) if checked else 0.0
            ma.unit = mat.get('unit', '')
            ma.batch_no = mat.get('batch_no', '')
            ma.container_code = mat.get('container_code', '')
            ma.inventory_id = mat.get('inventory_id', '')
            ma.status = 'SUCCEEDED' if checked else 'FAILED'
            ma.message = ''
            actual_list.append(ma)
        return actual_list

    def _build_zero_actual(self):
        """Build MaterialActual list with all zeros (for cancel/timeout)."""
        actual_list = []
        if self._current_goal is None:
            return actual_list
        for mat in self._current_goal.materials:
            ma = MaterialActual()
            ma.material_code = mat.get('material_code', '')
            ma.material_name = mat.get('material_name', '')
            ma.actual_quantity = 0.0
            ma.unit = mat.get('unit', '')
            ma.batch_no = mat.get('batch_no', '')
            ma.container_code = mat.get('container_code', '')
            ma.inventory_id = mat.get('inventory_id', '')
            ma.status = 'FAILED'
            ma.message = ''
            actual_list.append(ma)
        return actual_list

    def _cleanup_goal(self):
        with self._goal_lock:
            self._current_goal = None
            self._goal_state = GoalState.COMPLETED

    # ---- Called by GUI (main thread) ----

    def submit_result(self, status: str, message: str,
                      materials: list) -> bool:
        """Submit result from GUI. Called from Qt main thread.

        Args:
            status: 'SUCCEEDED', 'PARTIAL', or 'FAILED'
            message: Free-text message from operator
            materials: list of (checked: bool, material_dict: dict)

        Returns:
            True if accepted (goal was ACTIVE), False if ignored.
        """
        with self._goal_lock:
            if self._goal_state != GoalState.ACTIVE:
                self.get_logger().warn(
                    'submit_result ignored: goal state=%s'
                    % self._goal_state.value)
                return False
            self._result_status = status
            self._result_message = message
            self._result_materials = materials
            self._goal_state = GoalState.SUBMITTING
        self._goal_event.set()
        return True

    def cancel_result(self) -> bool:
        """Called when operator clicks Cancel in GUI.

        Returns:
            True if accepted (goal was ACTIVE), False if ignored.
        """
        with self._goal_lock:
            if self._goal_state != GoalState.ACTIVE:
                self.get_logger().warn(
                    'cancel_result ignored: goal state=%s'
                    % self._goal_state.value)
                return False
            self._result_message = 'Cancelled by operator'
            self._goal_state = GoalState.CANCELLING
        self._goal_event.set()
        return True
