#!/usr/bin/env python3
"""RS-485 chassis bridge: subscribes to steering/velocity topics, writes 485 frames to serial.

In simulation: writes to virtual serial port (e.g. /tmp/chassis_cmd via socat).
On real hardware: writes to USB-RS485 adapter (e.g. /dev/ttyUSB1).
"""

import serial
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import Float64

from lidar_slam_nodes.rs485_protocol import (
    NEUTRAL,
    encode_frame,
    speed_to_channel,
    steering_to_channel,
)


class RS485ChassisBridge(LifecycleNode):
    def __init__(self):
        super().__init__('rs485_chassis_bridge')

    def on_configure(self, state: LifecycleState):
        self.declare_parameter('serial_port', '/tmp/chassis_cmd')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('refresh_interval_ms', 200)
        self.declare_parameter('max_steering_angle', 0.5236)
        self.declare_parameter('max_velocity', 1.4)
        self.declare_parameter('timeout_ms', 500)

        self.port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.interval_ms = self.get_parameter('refresh_interval_ms').value
        self.timeout_sec = self.get_parameter('timeout_ms').value / 1000.0

        self.ser = None
        self.latest_steering = 0.0
        self.latest_velocity = 0.0
        self.last_steering_time = self.get_clock().now()
        self.last_velocity_time = self.get_clock().now()

        self.get_logger().info(
            f'RS485 bridge configured: interval={self.interval_ms}ms, '
            f'timeout={self.timeout_sec}s'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self.get_logger().info(f'Opened serial port: {self.port} @ {self.baudrate}')

        self.create_subscription(Float64, '/steering_angle', self.steering_cb, 10)
        self.create_subscription(Float64, '/velocity', self.velocity_cb, 10)

        period = self.interval_ms / 1000.0
        self._timer = self.create_timer(period, self.timer_cb)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState):
        self.ser = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        return TransitionCallbackReturn.SUCCESS

    def steering_cb(self, msg):
        self.latest_steering = msg.data
        self.last_steering_time = self.get_clock().now()

    def velocity_cb(self, msg):
        self.latest_velocity = msg.data
        self.last_velocity_time = self.get_clock().now()

    def timer_cb(self):
        if self.ser is None:
            return
        now = self.get_clock().now()
        steer_elapsed = (now - self.last_steering_time).nanoseconds / 1e9
        vel_elapsed = (now - self.last_velocity_time).nanoseconds / 1e9

        if steer_elapsed > self.timeout_sec or vel_elapsed > self.timeout_sec:
            ch1 = NEUTRAL
            ch2 = NEUTRAL
        else:
            ch1 = steering_to_channel(self.latest_steering, self.max_steering_angle)
            ch2 = speed_to_channel(self.latest_velocity, self.max_velocity)

        channels = [ch1, ch2] + [NEUTRAL] * 8
        frame = encode_frame(channels)
        self.ser.write(frame)


def main(args=None):
    rclpy.init(args=args)
    node = RS485ChassisBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
