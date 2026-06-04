#!/usr/bin/env python3
"""RS-485 chassis receiver: reads 485 frames from serial, publishes steering/velocity topics.

Simulation-only node. Reads protocol frames from virtual serial port and publishes
as Float64 on /rs485/ namespace to avoid feedback loops with the bridge node.
"""

import serial
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import Float64

from lidar_slam_nodes.rs485_protocol import (
    channel_to_speed,
    channel_to_steering,
    find_frame,
)


class RS485ChassisReceiver(LifecycleNode):
    def __init__(self):
        super().__init__('rs485_chassis_receiver')

    def on_configure(self, state: LifecycleState):
        self.declare_parameter('serial_port', '/tmp/chassis_recv')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_steering_angle', 0.5236)
        self.declare_parameter('max_velocity', 1.4)
        self.declare_parameter('timeout_ms', 500)

        self.port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.timeout_sec = self.get_parameter('timeout_ms').value / 1000.0

        self.ser = None
        self.buffer = b''
        self.last_frame_time = self.get_clock().now()

        self.steer_pub = self.create_publisher(Float64, 'rs485/steering_angle', 10)
        self.vel_pub = self.create_publisher(Float64, 'rs485/velocity', 10)

        self.get_logger().info(f'RS485 receiver configured: timeout={self.timeout_sec}s')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self.get_logger().info(f'Opened serial port: {self.port} @ {self.baudrate}')
        self._timer = self.create_timer(0.01, self.read_cb)  # 100 Hz read loop
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

    def read_cb(self):
        if self.ser is None:
            return
        # Read available bytes (non-blocking with short timeout)
        available = self.ser.in_waiting
        if available > 0:
            self.buffer += self.ser.read(available)

        # Try to extract a valid frame
        channels, self.buffer = find_frame(self.buffer)
        if channels is not None:
            self.last_frame_time = self.get_clock().now()
            steering = channel_to_steering(channels[0], self.max_steering_angle)
            speed = channel_to_speed(channels[1], self.max_velocity)

            steer_msg = Float64()
            steer_msg.data = steering
            self.steer_pub.publish(steer_msg)

            vel_msg = Float64()
            vel_msg.data = speed
            self.vel_pub.publish(vel_msg)
            return

        # Timeout: no valid frame received
        elapsed = (self.get_clock().now() - self.last_frame_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec:
            steer_msg = Float64()
            steer_msg.data = 0.0
            self.steer_pub.publish(steer_msg)

            vel_msg = Float64()
            vel_msg.data = 0.0
            self.vel_pub.publish(vel_msg)
            self.last_frame_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = RS485ChassisReceiver()
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
