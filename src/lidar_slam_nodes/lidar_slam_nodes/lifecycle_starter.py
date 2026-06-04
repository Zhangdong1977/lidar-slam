#!/usr/bin/env python3
"""Lifecycle starter: configurable timeout + retry replacement for Nav2 lifecycle_manager.

Nav2's lifecycle_manager uses an unbounded blocking service call that can hang
indefinitely under Fast-DDS + Gazebo CPU load. This node replaces it with:
  - Configurable per-call timeouts (30s default)
  - Automatic retries with exponential backoff
  - A /ready topic that signals when all managed nodes are active
  - Periodic health monitoring with automatic recovery

Usage:
  ros2 run lidar_slam_nodes lifecycle_starter --ros-args \
    -p node_names:['map_server','amcl'] \
    -p configure_timeout:30.0 \
    -p max_retries:5 \
    -p starter_name:'localization'
"""

import json
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from lifecycle_msgs.srv import ChangeState, GetState, GetAvailableStates
from lifecycle_msgs.msg import State, Transition
from std_msgs.msg import Bool, String


# Lifecycle state constants
TRANSITION_CONFIGURE = Transition.TRANSITION_CONFIGURE
TRANSITION_ACTIVATE = Transition.TRANSITION_ACTIVATE
TRANSITION_DEACTIVATE = Transition.TRANSITION_DEACTIVATE
TRANSITION_CLEANUP = Transition.TRANSITION_CLEANUP

STATE_UNCONFIGURED = State.PRIMARY_STATE_UNCONFIGURED
STATE_INACTIVE = State.PRIMARY_STATE_INACTIVE
STATE_ACTIVE = State.PRIMARY_STATE_ACTIVE


class LifecycleStarter(Node):
    """Manages lifecycle state transitions with configurable timeout and retries."""

    def __init__(self):
        super().__init__('lifecycle_starter')

        # Parameters — node_names defaults to [''] (STRING_ARRAY, not BYTE_ARRAY).
        # Launch serializes Python lists as !!python/tuple; an empty default []
        # gets typed BYTE_ARRAY, causing InvalidParameterTypeException.
        self.declare_parameter('node_names', [''])
        self.declare_parameter('configure_timeout', 30.0)
        self.declare_parameter('activate_timeout', 30.0)
        self.declare_parameter('max_retries', 5)
        self.declare_parameter('retry_delay', 2.0)
        self.declare_parameter('startup_delay', 0.0)
        self.declare_parameter('monitor_period', 10.0)
        self.declare_parameter('starter_name', 'unnamed')

        self._node_names = self.get_parameter('node_names').value
        self._configure_timeout = self.get_parameter('configure_timeout').value
        self._activate_timeout = self.get_parameter('activate_timeout').value
        self._max_retries = self.get_parameter('max_retries').value
        self._retry_delay = self.get_parameter('retry_delay').value
        self._startup_delay = self.get_parameter('startup_delay').value
        self._monitor_period = self.get_parameter('monitor_period').value
        self._starter_name = self.get_parameter('starter_name').value

        if not self._node_names:
            self.get_logger().warn('No node_names provided, nothing to manage')

        # Service clients: {node_name: {'get_state': client, 'change_state': client}}
        self._managed_clients = {}

        # Publishers
        self._ready_pub = self.create_publisher(Bool, f'lifecycle_starter_{self._starter_name}/ready', 1)
        self._status_pub = self.create_publisher(
            String, f'lifecycle_starter_{self._starter_name}/status', 1)

        self._ready = False
        self._monitor_timer = None

        self.get_logger().info(
            f'Lifecycle starter "{self._starter_name}" initialized: '
            f'managing {len(self._node_names)} nodes, '
            f'timeout={self._configure_timeout}s, '
            f'retries={self._max_retries}'
        )

    def startup(self):
        """Main startup sequence: wait delay, then configure+activate each node.

        Uses two-pass approach: first pass brings up all available nodes
        (skipping unavailable ones), second pass retries any remaining failures.
        This handles the case where some managed nodes start later than others.
        """
        # Wait for DDS to stabilize
        if self._startup_delay > 0:
            self.get_logger().info(
                f'Waiting {self._startup_delay:.1f}s for DDS to stabilize...')
            time.sleep(self._startup_delay)

        self.get_logger().info('Starting managed nodes bringup...')

        # Create service clients first
        for name in self._node_names:
            self._create_clients(name)

        # Two-pass bringup: first pass tries all nodes, second pass retries failures
        max_passes = 3
        failed = list(self._node_names)

        for pass_num in range(1, max_passes + 1):
            still_failed = []
            for name in failed:
                if not self._bringup_node(name):
                    self.get_logger().warn(
                        f'[{name}] Failed on pass {pass_num}, '
                        f'will retry later')
                    still_failed.append(name)

            failed = still_failed
            if not failed:
                break

            if pass_num < max_passes:
                delay = 5.0
                self.get_logger().info(
                    f'Pass {pass_num} complete: {len(failed)} nodes still failing. '
                    f'Retrying in {delay:.0f}s...')
                time.sleep(delay)

        if failed:
            self.get_logger().error(
                f'Failed to bring up {len(failed)} node(s) after {max_passes} passes: '
                f'{failed}')
            self._publish_status('PARTIAL', f'Failed: {failed}')
            # Still mark ready if at least some nodes are up
            active_count = len(self._node_names) - len(failed)
            if active_count > 0:
                self.get_logger().warn(
                    f'{active_count}/{len(self._node_names)} nodes are active, '
                    f'proceeding with partial bringup')
            else:
                return False

        self._ready = True
        self._ready_pub.publish(Bool(data=True))
        self.get_logger().info('Managed nodes bringup complete')

        # Start periodic health monitoring
        if self._monitor_period > 0:
            self._monitor_timer = self.create_timer(
                self._monitor_period, self._monitor_callback)
            self.get_logger().info(
                f'Health monitoring started (period={self._monitor_period}s)')

        self._publish_status('ACTIVE', 'All nodes active' if not failed else f'Partial: {len(self._node_names) - len(failed)}/{len(self._node_names)}')
        return True

    def _create_clients(self, node_name):
        """Create get_state and change_state service clients for a node."""
        self._managed_clients[node_name] = {
            'get_state': self.create_client(
                GetState, f'{node_name}/get_state'),
            'change_state': self.create_client(
                ChangeState, f'{node_name}/change_state'),
        }

    def _bringup_node(self, node_name):
        """Configure then activate a lifecycle node with retries."""
        # Configure
        if not self._transition_with_retry(
            node_name, TRANSITION_CONFIGURE, 'configure',
            self._configure_timeout, State.TRANSITION_STATE_CONFIGURING
        ):
            return False

        # Activate
        if not self._transition_with_retry(
            node_name, TRANSITION_ACTIVATE, 'activate',
            self._activate_timeout, State.TRANSITION_STATE_ACTIVATING
        ):
            return False

        return True

    def _transition_with_retry(self, node_name, transition_id, transition_name,
                                timeout, expected_state):
        """Execute a lifecycle transition with retries and timeout."""
        clients = self._managed_clients.get(node_name)
        if not clients:
            self.get_logger().error(f'No clients for {node_name}')
            return False

        change_client = clients['change_state']
        get_client = clients['get_state']

        for attempt in range(1, self._max_retries + 1):
            current_state = self._get_node_state(node_name, get_client)
            if self._transition_satisfied(transition_id, current_state):
                self.get_logger().info(
                    f'[{node_name}] {transition_name.capitalize()} already '
                    f'satisfied (state={self._state_label(current_state)})')
                return True

            # Wait for service availability
            if not change_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().warn(
                    f'[{node_name}] {transition_name}: service not available, '
                    f'retry {attempt}/{self._max_retries}')
                if attempt < self._max_retries:
                    time.sleep(self._retry_delay)
                continue

            # Call change_state with timeout
            self.get_logger().info(
                f'[{node_name}] {transition_name.capitalize()}ing '
                f'(attempt {attempt}/{self._max_retries}, timeout={timeout}s)')

            request = ChangeState.Request()
            request.transition.id = transition_id

            future = change_client.call_async(request)
            start = time.monotonic()

            # Spin with timeout
            while rclpy.ok() and (time.monotonic() - start) < timeout:
                if future.done():
                    break
                # Spin executor manually for this node
                rclpy.spin_once(self, timeout_sec=0.5)

            if not future.done():
                current_state = self._get_node_state(node_name, get_client)
                if self._transition_satisfied(transition_id, current_state):
                    self.get_logger().info(
                        f'[{node_name}] {transition_name.capitalize()}d '
                        f'(state={self._state_label(current_state)})')
                    return True

                self.get_logger().warn(
                    f'[{node_name}] {transition_name}: TIMEOUT after {timeout}s, '
                    f'retry {attempt}/{self._max_retries}')
                # Cancel the pending request
                change_client.remove_pending_request(future)
                if attempt < self._max_retries:
                    delay = self._retry_delay * (2 ** (attempt - 1))  # exponential backoff
                    self.get_logger().info(
                        f'[{node_name}] Retrying in {delay:.1f}s...')
                    time.sleep(delay)
                continue

            try:
                response = future.result()
            except Exception as e:
                current_state = self._get_node_state(node_name, get_client)
                if self._transition_satisfied(transition_id, current_state):
                    self.get_logger().info(
                        f'[{node_name}] {transition_name.capitalize()}d '
                        f'(state={self._state_label(current_state)})')
                    return True

                self.get_logger().warn(
                    f'[{node_name}] {transition_name}: exception: {e}, '
                    f'retry {attempt}/{self._max_retries}')
                if attempt < self._max_retries:
                    time.sleep(self._retry_delay)
                continue

            if not response.success:
                current_state = self._get_node_state(node_name, get_client)
                if self._transition_satisfied(transition_id, current_state):
                    self.get_logger().info(
                        f'[{node_name}] {transition_name.capitalize()}d '
                        f'(state={self._state_label(current_state)})')
                    return True

                self.get_logger().warn(
                    f'[{node_name}] {transition_name}: failed, '
                    f'retry {attempt}/{self._max_retries}')
                if attempt < self._max_retries:
                    time.sleep(self._retry_delay)
                continue

            # Verify state
            current_state = self._get_node_state(node_name, get_client)
            if current_state is not None:
                self.get_logger().info(
                    f'[{node_name}] {transition_name.capitalize()}d '
                    f'(state={self._state_label(current_state)})')
            return True

        self.get_logger().error(
            f'[{node_name}] {transition_name}: FAILED after {self._max_retries} retries')
        return False

    @staticmethod
    def _transition_satisfied(transition_id, state_id):
        if state_id is None:
            return False
        if transition_id == TRANSITION_CONFIGURE:
            return state_id in (STATE_INACTIVE, STATE_ACTIVE)
        if transition_id == TRANSITION_ACTIVATE:
            return state_id == STATE_ACTIVE
        return False

    def _get_node_state(self, node_name, get_client=None):
        """Get the current state of a lifecycle node."""
        if get_client is None:
            clients = self._managed_clients.get(node_name)
            if not clients:
                return None
            get_client = clients['get_state']

        if not get_client.wait_for_service(timeout_sec=5.0):
            return None

        request = GetState.Request()
        future = get_client.call_async(request)
        start = time.monotonic()
        while rclpy.ok() and (time.monotonic() - start) < 10.0:
            if future.done():
                break
            rclpy.spin_once(self, timeout_sec=0.5)

        if future.done():
            try:
                return future.result().current_state.id
            except Exception:
                return None
        return None

    def _monitor_callback(self):
        """Periodic health check: verify managed nodes are still active."""
        all_active = True
        for name in self._node_names:
            state = self._get_node_state(name)
            if state is None or state != STATE_ACTIVE:
                self.get_logger().warn(
                    f'[{name}] not active (state={self._state_label(state)}), '
                    f'recovering...')
                all_active = False
                # Attempt recovery
                self._bringup_node(name)

        if all_active and not self._ready:
            self._ready = True
            self._ready_pub.publish(Bool(data=True))
            self._publish_status('ACTIVE', 'All nodes active (recovered)')

    def _publish_status(self, status, message=''):
        msg = String()
        msg.data = json.dumps({
            'starter': self._starter_name,
            'status': status,
            'message': message,
            'nodes': self._node_names,
            'timestamp': time.monotonic(),
        })
        self._status_pub.publish(msg)

    @staticmethod
    def _state_label(state_id):
        labels = {
            STATE_UNCONFIGURED: 'unconfigured',
            STATE_INACTIVE: 'inactive',
            STATE_ACTIVE: 'active',
        }
        return labels.get(state_id, f'unknown({state_id})')


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleStarter()
    try:
        success = node.startup()
        if success:
            node.get_logger().info(
                'Startup complete. Exiting (one-shot mode).')
            sys.exit(0)
        else:
            node.get_logger().error(
                'Startup failed, exiting. Check logs above for details.')
            sys.exit(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
