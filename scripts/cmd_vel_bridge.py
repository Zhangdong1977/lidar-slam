#!/usr/bin/env python3
"""Bridge Nav2 cmd_vel (Twist) to Ackermann vehicle_controller (Float64).

Subscribes: /cmd_vel (geometry_msgs/Twist)
Publishes:  /steering_angle (std_msgs/Float64), /velocity (std_msgs/Float64)

Conversion: velocity = twist.linear.x
            steering_angle = atan(wheel_base * angular.z / linear.x)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('wheel_base', 0.58)
        self.declare_parameter('max_steering_angle', 0.5236)
        self.declare_parameter('max_velocity', 1.4)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        publish_rate = self.get_parameter('publish_rate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        self.latest_twist = None
        self.last_twist_time = self.get_clock().now()

        self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)
        self.velocity_pub = self.create_publisher(Float64, '/velocity', 10)

        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        period = 1.0 / publish_rate
        self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'cmd_vel_bridge started: wheel_base={self.wheel_base}, '
            f'max_steer={self.max_steering_angle:.4f}, max_vel={self.max_velocity}'
        )

    def clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def twist_callback(self, msg):
        self.latest_twist = msg
        self.last_twist_time = self.get_clock().now()

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.last_twist_time).nanoseconds / 1e9

        if self.latest_twist is None or elapsed > self.cmd_vel_timeout:
            self.publish(0.0, 0.0)
            return

        twist = self.latest_twist
        velocity = self.clamp(twist.linear.x, -self.max_velocity, self.max_velocity)

        # Ackermann needs forward motion to steer; provide minimum creep
        # velocity when Nav2 commands pure rotation or near-zero speed.
        if abs(velocity) < 0.05 and abs(twist.angular.z) > 0.01:
            velocity = 0.05 if twist.linear.x >= 0 else -0.05

        if abs(velocity) > 0.01:
            steering_angle = math.atan(
                self.wheel_base * twist.angular.z / velocity
            )
            steering_angle = self.clamp(
                steering_angle, -self.max_steering_angle, self.max_steering_angle
            )
        else:
            steering_angle = 0.0

        self.publish(steering_angle, velocity)

    def publish(self, steering_angle, velocity):
        steer_msg = Float64()
        steer_msg.data = steering_angle
        self.steering_pub.publish(steer_msg)

        vel_msg = Float64()
        vel_msg.data = velocity
        self.velocity_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
