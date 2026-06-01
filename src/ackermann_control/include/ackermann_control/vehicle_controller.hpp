#ifndef ACKERMANN_CONTROL__VEHICLE_CONTROLLER_HPP_
#define ACKERMANN_CONTROL__VEHICLE_CONTROLLER_HPP_

#include <chrono>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ackermann_control
{

class VehicleController : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit VehicleController(
    const double timer_period = 0.01,
    const double timeout_duration = 8e8  // 800ms in nanoseconds
  );

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);


  std::pair<double, double> ackermann_steering_angle();
  std::pair<double, double> rear_differential_velocity();

  void timer_callback();
  void steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);

  double timeout_duration_;
  double timer_period_;
  rclcpp::Time last_velocity_time_;
  rclcpp::Time last_steering_time_;

  double body_width_;
  double body_length_;
  double wheel_radius_;
  double wheel_width_;
  double max_steering_angle_;
  double max_velocity_;
  double wheel_base_;
  double track_width_;

  double steering_angle_;
  double velocity_;

  // Direction change transition state
  double prev_velocity_{0.0};
  rclcpp::Time direction_change_time_;
  bool is_transitioning_{false};
  double transition_duration_{0.5};

  std::vector<double> wheel_angular_velocity_;
  std::vector<double> wheel_steering_angle_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ackermann_control

#endif  // ACKERMANN_CONTROL__VEHICLE_CONTROLLER_HPP_
