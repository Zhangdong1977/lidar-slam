#ifndef ACKERMANN_CONTROL__VEHICLE_CONTROLLER_HPP_
#define ACKERMANN_CONTROL__VEHICLE_CONTROLLER_HPP_

#include <chrono>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ackermann_control
{

class VehicleController : public rclcpp::Node
{
public:
  explicit VehicleController(
    const double timer_period = 0.01,
    const double timeout_duration = 8e8  // 800ms in nanoseconds
  );

private:
  std::pair<double, double> ackermann_steering_angle();
  std::pair<double, double> rear_differential_velocity();

  void timer_callback();
  void steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);

  double timeout_duration_;
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

  std::vector<double> wheel_angular_velocity_;
  std::vector<double> wheel_steering_angle_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ackermann_control

#endif  // ACKERMANN_CONTROL__VEHICLE_CONTROLLER_HPP_
