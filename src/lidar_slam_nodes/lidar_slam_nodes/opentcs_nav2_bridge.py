#!/usr/bin/env python3
"""Bridge openTCS-NeNa topics to Nav2 NavigateToPose action.

Subscribes: /goal_pose (geometry_msgs/PoseStamped) from openTCS
Publishes:  /amcl_pose (geometry_msgs/PoseWithCovarianceStamped) to openTCS
Action:     /navigate_to_pose (nav2_msgs/NavigateToPose) to Nav2

Converts openTCS topic-based navigation interface to Nav2 action calls,
and publishes robot position from TF for openTCS tracking.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros


class OpentcsNav2Bridge(Node):
    def __init__(self):
        super().__init__('opentcs_nav2_bridge')

        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('nav_action_name', '/navigate_to_pose')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'body_link')
        self.declare_parameter('pose_publish_rate', 10.0)

        goal_topic = self.get_parameter('goal_pose_topic').value
        amcl_topic = self.get_parameter('amcl_pose_topic').value
        action_name = self.get_parameter('nav_action_name').value
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        rate = self.get_parameter('pose_publish_rate').value

        self._goal_handle = None
        self._send_goal_future = None

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Publisher: robot position for openTCS
        self._amcl_pub = self.create_publisher(PoseWithCovarianceStamped, amcl_topic, 10)

        # Subscriber: navigation goals from openTCS
        self.create_subscription(PoseStamped, goal_topic, self._goal_cb, 10)

        # Action client: Nav2 NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, action_name)

        # Timer: publish position at configured rate
        self.create_timer(1.0 / rate, self._publish_pose)

        self.get_logger().info(
            f'opentcs_nav2_bridge started: goal={goal_topic}, '
            f'amcl={amcl_topic}, action={action_name}, rate={rate}Hz'
        )

    def _goal_cb(self, msg: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server not available')
            return

        # Cancel previous goal if active
        if self._goal_handle is not None:
            self.get_logger().info('Canceling previous goal')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = msg

        self.get_logger().info(
            f'Sending goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        self._send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        self._send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self._goal_handle = None
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().warn(f'Navigation finished with status: {status}')
        self._goal_handle = None

    def _feedback_cb(self, feedback_msg):
        pass

    def _publish_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation = t.transform.rotation
        # Fixed covariance — this is a TF-based estimate, not probabilistic AMCL
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.01   # x
        msg.pose.covariance[7] = 0.01   # y
        msg.pose.covariance[35] = 0.01  # yaw

        self._amcl_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpentcsNav2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
