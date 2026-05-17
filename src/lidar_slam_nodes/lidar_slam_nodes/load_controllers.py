#!/usr/bin/env python3
"""Load and activate controllers via controller_manager service calls."""

import sys
import time
import rclpy
from controller_manager_msgs.srv import LoadController, ConfigureController, SwitchController


def main():
    rclpy.init()
    node = rclpy.create_node('load_controllers_client')

    controllers = [
        'joint_state_broadcaster',
        'forward_position_controller',
        'forward_velocity_controller',
    ]

    # Discover controller_manager service (try namespaced then root)
    load_srv = None
    config_srv = None
    switch_srv = None

    for base in ['/ackermann_robot/controller_manager', '/controller_manager']:
        load_name = f'{base}/load_controller'
        config_name = f'{base}/configure_controller'
        switch_name = f'{base}/switch_controller'

        # Wait for services
        node.get_logger().info(f'Waiting for {load_name}...')
        load_cli = node.create_client(LoadController, load_name)
        if not load_cli.wait_for_service(timeout_sec=30.0):
            node.get_logger().warn(f'Timeout waiting for {load_name}')
            node.destroy_client(load_cli)
            continue

        config_cli = node.create_client(ConfigureController, config_name)
        switch_cli = node.create_client(SwitchController, switch_name)

        if not config_cli.wait_for_service(timeout_sec=5.0):
            node.get_logger().warn(f'Timeout waiting for {config_name}')
            continue
        if not switch_cli.wait_for_service(timeout_sec=5.0):
            node.get_logger().warn(f'Timeout waiting for {switch_name}')
            continue

        load_srv = load_cli
        config_srv = config_cli
        switch_srv = switch_cli
        break

    if load_srv is None:
        node.get_logger().error('Could not find controller_manager services')
        sys.exit(1)

    # Load and configure controllers
    for ctrl in controllers:
        req = LoadController.Request()
        req.name = ctrl
        future = load_srv.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is None:
            node.get_logger().error(f'Timeout loading {ctrl}')
            continue
        if not future.result().ok:
            node.get_logger().warn(f'Load {ctrl} returned not ok')
        else:
            node.get_logger().info(f'Loaded {ctrl}')

        req2 = ConfigureController.Request()
        req2.name = ctrl
        future2 = config_srv.call_async(req2)
        rclpy.spin_until_future_complete(node, future2, timeout_sec=5.0)
        if future2.result() is None:
            node.get_logger().error(f'Timeout configuring {ctrl}')
            continue
        if not future2.result().ok:
            node.get_logger().warn(f'Configure {ctrl} returned not ok')
        else:
            node.get_logger().info(f'Configured {ctrl}')

    # Activate all controllers
    req3 = SwitchController.Request()
    req3.activate_controllers = controllers
    req3.deactivate_controllers = []
    req3.strictness = SwitchController.Request.BEST_EFFORT
    req3.activate_asap = False
    req3.timeout.sec = 10
    req3.timeout.nanosec = 0

    future3 = switch_srv.call_async(req3)
    rclpy.spin_until_future_complete(node, future3, timeout_sec=10.0)
    if future3.result() is None:
        node.get_logger().error('Timeout switching controllers')
    elif not future3.result().ok:
        node.get_logger().warn(f'Switch controllers returned not ok')
    else:
        node.get_logger().info('Controllers activated')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
