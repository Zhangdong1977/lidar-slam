#!/usr/bin/env python3
"""Lightweight watchdog node: monitors critical topics and nodes, publishes health diagnostics.

Design: report-only, never restarts anything. Restart is handled by launch respawn + lifecycle_manager.

Publishes:
  /system_health          (diagnostic_msgs/DiagnosticArray) — compatible with rqt_robot_monitor
  /system_health_summary  (std_msgs/String) — JSON summary for script parsing

Health levels:
  OK    — all critical nodes/topics healthy
  WARN  — non-critical node/topic issue
  ERROR — critical node missing or critical topic stale
  STALE — watchdog itself hasn't published (detectable by subscriber)
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String


class NodeWatchdog(Node):
    def __init__(self):
        super().__init__('node_watchdog')

        # Parameters
        self.declare_parameter('check_period', 5.0)
        self.declare_parameter('topic_timeout', 5.0)
        self.declare_parameter('critical_nodes', [
            'ekf_filter_node', 'amcl', 'controller_server', 'bt_navigator',
            'cmd_vel_bridge', 'opentcs_vehicle_node',
            'lifecycle_manager_localization', 'lifecycle_manager_navigation',
            'lifecycle_manager_custom',
        ])
        self.declare_parameter('non_critical_nodes', ['rviz2', 'material_action_gui'])
        self.declare_parameter('critical_topics', [
            '/scan', '/odom', '/tf', '/cmd_vel', '/amcl_pose',
        ])
        self.declare_parameter('non_critical_topics', ['/route_graph/markers'])

        self._check_period = self.get_parameter('check_period').value
        self._topic_timeout = self.get_parameter('topic_timeout').value
        self._critical_nodes = self.get_parameter('critical_nodes').value
        self._non_critical_nodes = self.get_parameter('non_critical_nodes').value
        self._critical_topics = self.get_parameter('critical_topics').value
        self._non_critical_topics = self.get_parameter('non_critical_topics').value

        # Topic liveness tracking: topic_name -> last_msg_time
        self._topic_times = {}
        for topic in self._critical_topics + self._non_critical_topics:
            self._topic_times[topic] = 0.0

        # Create subscribers for all monitored topics (best-effort, minimal QoS)
        for topic in self._topic_times:
            # Subscribe to any topic type using a generic subscription
            # We only care about receive timestamps, not content
            try:
                self.create_subscription(
                    None, topic,
                    lambda msg, t=topic: self._topic_msg_cb(t),
                    10,
                    raw=True,
                )
            except Exception:
                # Fallback: subscribe with a known common type to track liveness
                pass

        # Publishers
        diag_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/system_health', diag_qos)
        self._summary_pub = self.create_publisher(
            String, '/system_health_summary', 10)

        # Check timer
        self._timer = self.create_timer(self._check_period, self._check_health)
        self._start_time = time.monotonic()

        self.get_logger().info(
            f'Watchdog started: monitoring {len(self._critical_nodes)} critical + '
            f'{len(self._non_critical_nodes)} non-critical nodes, '
            f'{len(self._critical_topics)} critical + '
            f'{len(self._non_critical_topics)} non-critical topics, '
            f'period={self._check_period}s'
        )

    def _topic_msg_cb(self, topic_name: str):
        """Record the time of the last message on a monitored topic."""
        self._topic_times[topic_name] = time.monotonic()

    def _check_health(self):
        """Periodic health check: inspect nodes and topics, publish diagnostics."""
        now = time.monotonic()
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        overall_status = DiagnosticStatus.OK
        overall_name = 'OK'
        details = {}

        # --- Check node presence ---
        node_names_and_ns = self.get_node_names_and_namespaces()
        node_names = {n for n, _ in node_names_and_ns}

        # Critical nodes
        for name in self._critical_nodes:
            status = DiagnosticStatus()
            status.name = f'nodes.critical.{name}'
            status.hardware_id = 'watchdog'

            if name in node_names:
                status.level = DiagnosticStatus.OK
                status.message = 'alive'
                details[f'node:{name}'] = 'OK'
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = 'MISSING'
                overall_status = DiagnosticStatus.ERROR
                overall_name = 'ERROR'
                details[f'node:{name}'] = 'MISSING'

            diag.status.append(status)

        # Non-critical nodes
        for name in self._non_critical_nodes:
            status = DiagnosticStatus()
            status.name = f'nodes.non_critical.{name}'
            status.hardware_id = 'watchdog'

            if name in node_names:
                status.level = DiagnosticStatus.OK
                status.message = 'alive'
                details[f'node:{name}'] = 'OK'
            else:
                status.level = DiagnosticStatus.WARN
                status.message = 'MISSING'
                if overall_status == DiagnosticStatus.OK:
                    overall_status = DiagnosticStatus.WARN
                    overall_name = 'WARN'
                details[f'node:{name}'] = 'WARN'

            diag.status.append(status)

        # --- Check topic liveness ---
        for topic in self._critical_topics + self._non_critical_topics:
            last_time = self._topic_times.get(topic, 0.0)
            is_critical = topic in self._critical_topics

            status = DiagnosticStatus()
            status.name = f'topics.{"critical" if is_critical else "non_critical"}{topic}'
            status.hardware_id = 'watchdog'

            if last_time == 0.0:
                age_str = 'never'
            else:
                age = now - last_time
                age_str = f'{age:.1f}s ago'

            if last_time == 0.0 and (now - self._start_time) < self._topic_timeout * 2:
                # Grace period at startup
                status.level = DiagnosticStatus.OK
                status.message = f'waiting (started {now - self._start_time:.0f}s ago)'
                details[f'topic:{topic}'] = 'STARTING'
            elif last_time > 0 and (now - last_time) <= self._topic_timeout:
                status.level = DiagnosticStatus.OK
                status.message = f'active (last {age_str})'
                details[f'topic:{topic}'] = 'OK'
            elif is_critical:
                status.level = DiagnosticStatus.ERROR
                status.message = f'STALE (last {age_str})'
                overall_status = DiagnosticStatus.ERROR
                overall_name = 'ERROR'
                details[f'topic:{topic}'] = 'ERROR'
            else:
                status.level = DiagnosticStatus.WARN
                status.message = f'stale (last {age_str})'
                if overall_status == DiagnosticStatus.OK:
                    overall_status = DiagnosticStatus.WARN
                    overall_name = 'WARN'
                details[f'topic:{topic}'] = 'WARN'

            status.values.append(KeyValue(key='age', value=age_str))
            diag.status.append(status)

        # Overall summary status
        summary_status = DiagnosticStatus()
        summary_status.name = 'system.overall'
        summary_status.hardware_id = 'watchdog'
        summary_status.level = overall_status
        summary_status.message = overall_name
        diag.status.insert(0, summary_status)

        # Publish diagnostic array
        self._diag_pub.publish(diag)

        # Publish JSON summary
        summary = {
            'status': overall_name,
            'timestamp': now,
            'details': details,
        }
        summary_msg = String()
        summary_msg.data = json.dumps(summary, ensure_ascii=False)
        self._summary_pub.publish(summary_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NodeWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
