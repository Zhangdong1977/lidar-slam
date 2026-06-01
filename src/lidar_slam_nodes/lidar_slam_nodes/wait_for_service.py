#!/usr/bin/env python3
"""Block until a specified ROS2 service is available, then exit.

Exit codes: 0 = ready, 1 = timeout.
Modeled on the existing wait_for_tf.py readiness guard pattern.
"""

import importlib
import sys

import rclpy
from rclpy.node import Node


def _import_type(type_string: str):
    """Import a ROS2 service type from its fully qualified string.

    Example: 'nav2_msgs/srv/SetRouteGraph' -> nav2_msgs.srv.SetRouteGraph
    """
    parts = type_string.split('/')
    if len(parts) != 3:
        raise ValueError(
            f'Invalid service type: {type_string}. '
            f'Expected format: package/service_type/ServiceName'
        )
    module_path = f'{parts[0]}.{parts[1]}'
    class_name = parts[2]
    module = importlib.import_module(module_path)
    return getattr(module, class_name)


class WaitForService(Node):
    def __init__(self):
        super().__init__('wait_for_service')

        self.declare_parameter('service_name', '')
        self.declare_parameter('service_type', '')
        self.declare_parameter('timeout', 60.0)

        service_name = self.get_parameter('service_name').value
        service_type_str = self.get_parameter('service_type').value
        timeout = self.get_parameter('timeout').value

        if not service_name or not service_type_str:
            self.get_logger().error(
                'service_name and service_type parameters are required'
            )
            sys.exit(1)

        self.get_logger().info(
            f'Waiting for service {service_name} (timeout={timeout}s)'
        )

        try:
            srv_type = _import_type(service_type_str)
        except (ImportError, AttributeError, ValueError) as e:
            self.get_logger().error(f'Cannot import service type {service_type_str}: {e}')
            sys.exit(1)

        cli = self.create_client(srv_type, service_name)

        if cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().info(f'Service {service_name} is ready')
            cli.destroy()
            sys.exit(0)
        else:
            self.get_logger().error(
                f'Timeout waiting for service {service_name} ({timeout}s)'
            )
            cli.destroy()
            sys.exit(1)


def main():
    rclpy.init(args=sys.argv)
    node = WaitForService()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
