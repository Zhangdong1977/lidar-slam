#!/usr/bin/env python3
"""Bridge /PowerVoltage (Float32) to /battery_state (BatteryState).

Subscribes: /PowerVoltage (std_msgs/Float32) — raw voltage from STM32
Publishes:  /battery_state (sensor_msgs/BatteryState) — standard ROS2 battery message

Voltage-to-percentage mapping (configurable):
  24.0V = 100%, 20.0V = 0% (linear interpolation)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState


class BatteryBridge(Node):
    def __init__(self):
        super().__init__('battery_bridge')

        self.declare_parameter('voltage_full', 24.0)
        self.declare_parameter('voltage_empty', 20.0)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('design_capacity', 5.0)  # Ah

        self.voltage_full = self.get_parameter('voltage_full').value
        self.voltage_empty = self.get_parameter('voltage_empty').value
        self.design_capacity = self.get_parameter('design_capacity').value

        self.latest_voltage = None

        self.sub = self.create_subscription(
            Float32, '/PowerVoltage', self.voltage_callback, 10)

        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.publish_battery)

        self.get_logger().info(
            f'Battery bridge started: {self.voltage_empty}V-'
            f'{self.voltage_full}V range')

    def voltage_callback(self, msg):
        self.latest_voltage = msg.data

    def voltage_to_percent(self, voltage):
        if self.voltage_full == self.voltage_empty:
            return 0.0
        pct = (voltage - self.voltage_empty) / (self.voltage_full - self.voltage_empty)
        return max(0.0, min(100.0, pct * 100.0))

    def publish_battery(self):
        if self.latest_voltage is None:
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(self.latest_voltage)
        msg.percentage = self.voltage_to_percent(self.latest_voltage)
        msg.capacity = float('nan')
        msg.design_capacity = self.design_capacity
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.present = True

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
