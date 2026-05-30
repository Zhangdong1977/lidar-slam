"""Material action server node with PyQt signal bridge.

Hosts LoadMaterials and UnloadMaterials action servers.
Uses ActionBridge (QObject) to emit pyqtSignal when goals arrive,
allowing the Qt GUI to update from the main thread.
"""

import threading
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
    # action completed — whether succeeded, failed, or cancelled
    action_finished = QtCore.pyqtSignal(str, str)


class MaterialActionServer(Node):
    """ROS2 node hosting LoadMaterials and UnloadMaterials action servers."""

    def __init__(self):
        super().__init__('material_action_server')
        self._bridge = ActionBridge()
        self._cb_group = ReentrantCallbackGroup()

        # Current goal state
        self._goal_lock = threading.Lock()
        self._current_goal: GoalData = None
        self._result_event = threading.Event()
        self._cancel_requested = False

        # Result set by GUI
        self._result_status = ''
        self._result_message = ''
        self._result_materials = []  # list of (checked: bool, material: dict)

        # Vehicle name for validation
        self.declare_parameter('vehicle_name', 'ackermann_robot')
        self._vehicle_name = self.get_parameter('vehicle_name').value

        # Action servers
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
            'Material action server started (vehicle=%s)' % self._vehicle_name)

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
        """Handle cancel request."""
        self.get_logger().info('Cancel requested for goal')
        self._cancel_requested = True
        self._bridge.action_cancelled.emit()
        return CancelResponse.ACCEPT

    def _execute_load(self, goal_handle):
        return self._execute(goal_handle, ActionType.LOAD, LoadMaterials)

    def _execute_unload(self, goal_handle):
        return self._execute(goal_handle, ActionType.UNLOAD, UnloadMaterials)

    def _execute(self, goal_handle, action_type: ActionType, action_cls):
        """Core execution: notify GUI, wait for operator, return result."""
        with self._goal_lock:
            self._current_goal = GoalData(action_type, goal_handle)
            self._result_event.clear()
            self._cancel_requested = False
            self._result_status = ''
            self._result_message = ''
            self._result_materials = []

        # Notify GUI
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

        # Feedback: accepted
        self._publish_feedback(goal_handle, 'ACCEPTED', 0.0, 'Goal accepted')
        self._publish_feedback(goal_handle, 'CHECKING', 0.1, 'Checking materials')

        # Wait for operator (GUI button click) or cancel
        while not self._result_event.is_set():
            if self._cancel_requested:
                self._publish_feedback(goal_handle, 'COMPLETING', 1.0, 'Cancelled')
                result = action_cls.Result()
                result.success = False
                result.status = 'CANCELLED'
                result.message = self._result_message or 'Cancelled by operator'
                goal_handle.succeed()
                self._bridge.action_finished.emit('CANCELLED', result.message)
                self._cleanup_goal()
                return result
            self._result_event.wait(timeout=0.2)

        # Build result from GUI state
        self._publish_feedback(goal_handle, 'VERIFYING', 0.9, 'Verifying results')
        self._publish_feedback(goal_handle, 'COMPLETING', 1.0, 'Completing')

        status = self._result_status
        message = self._result_message

        result = action_cls.Result()
        result.message = message
        result.error_code = ''

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

        if status == 'FAILED':
            result.success = False
            result.status = 'FAILED'
            result.error_code = 'LOAD_FAILED' if action_type == ActionType.LOAD else 'UNLOAD_FAILED'
            # Override: all materials FAILED
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

        # Set the correct field name depending on action type
        if action_type == ActionType.LOAD:
            result.actual_loaded = actual_list
        else:
            result.actual_unloaded = actual_list

        # current_load same as actual for simulation
        result.current_load = list(actual_list)

        goal_handle.succeed()
        self.get_logger().info(
            'Action %s completed: status=%s' % (action_type.value, result.status))
        self._bridge.action_finished.emit(result.status, result.message)
        self._cleanup_goal()
        return result

    def _publish_feedback(self, goal_handle, phase, progress, message):
        """Publish feedback on the goal handle."""
        # Workaround: access the action type's Feedback via the goal_handle
        try:
            fb = goal_handle.create_feedback()
            fb.phase = phase
            fb.progress = progress
            fb.message = message
            goal_handle.publish_feedback(fb)
        except Exception:
            pass
        self._bridge.feedback_updated.emit(phase, progress, message)

    def _cleanup_goal(self):
        with self._goal_lock:
            self._current_goal = None

    # ---- Called by GUI (main thread) ----

    def submit_result(self, status: str, message: str,
                      materials: list):
        """Submit result from GUI. Called from Qt main thread.

        Args:
            status: 'SUCCEEDED', 'PARTIAL', 'FAILED', or 'CANCELLED'
            message: Free-text message from operator
            materials: list of (checked: bool, material_dict: dict)
        """
        self._result_status = status
        self._result_message = message
        self._result_materials = materials
        self._result_event.set()

    def cancel_result(self):
        """Called when operator clicks Cancel in GUI."""
        self._cancel_requested = True
        self._result_message = 'Cancelled by operator'
        self._result_event.set()
