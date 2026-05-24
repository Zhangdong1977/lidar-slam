#!/usr/bin/env python3
"""RS-485 chassis receiver: reads 485 frames from serial, publishes steering/velocity topics.

Simulation-only node. Reads protocol frames from virtual serial port and publishes
as Float64 on /rs485/ namespace to avoid feedback loops with the bridge node.
"""

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from lidar_slam_nodes.rs485_protocol import (
    channel_to_speed,
    channel_to_steering,
    find_frame,
)


class RS485ChassisReceiver(Node):
    def __init__(self):
        super().__init__('rs485_chassis_receiver')

        self.declare_parameter('serial_port', '/tmp/chassis_recv')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_steering_angle', 0.5236)
        self.declare_parameter('max_velocity', 1.4)
        self.declare_parameter('timeout_ms', 500)

        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.timeout_sec = self.get_parameter('timeout_ms').value / 1000.0

        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        self.get_logger().info(f'Opened serial port: {port} @ {baudrate}')

        self.steer_pub = self.create_publisher(Float64, '/rs485/steering_angle', 10)
        self.vel_pub = self.create_publisher(Float64, '/rs485/velocity', 10)

        self.buffer = b''
        self.last_frame_time = self.get_clock().now()

        self.create_timer(0.01, self.read_cb)  # 100 Hz read loop
        self.get_logger().info(f'RS485 receiver started: timeout={self.timeout_sec}s')

    def read_cb(self):
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

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RS485ChassisReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
