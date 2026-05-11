#include "ackermann_control/vehicle_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace ackermann_control
{

VehicleController::VehicleController(
  const double timer_period,
  const double timeout_duration)
: Node{"vehicle_controller"},
  timeout_duration_{timeout_duration},
  last_velocity_time_{get_clock()->now()},
  last_steering_time_{get_clock()->now()},
  body_width_{0.0},
  body_length_{0.0},
  wheel_radius_{0.0},
  wheel_width_{0.0},
  max_steering_angle_{0.0},
  max_velocity_{0.0},
  wheel_base_{0.0},
  track_width_{0.0},
  steering_angle_{0.0},
  velocity_{0.0},
  wheel_angular_velocity_{0.0, 0.0},
  wheel_steering_angle_{0.0, 0.0}
{
  declare_parameter<double>("body_width", 0.0);
  declare_parameter<double>("body_length", 0.0);
  declare_parameter<double>("wheel_radius", 0.0);
  declare_parameter<double>("wheel_width", 0.0);
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);

  get_parameter("body_width", body_width_);
  get_parameter("body_length", body_length_);
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("wheel_width", wheel_width_);
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);

  track_width_ = body_width_ + (2 * wheel_width_ / 2);
  wheel_base_ = body_length_ - (2 * wheel_radius_);

  steering_angle_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/steering_angle", 10,
    std::bind(&VehicleController::steering_angle_callback, this, std::placeholders::_1));

  velocity_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/velocity", 10,
    std::bind(&VehicleController::velocity_callback, this, std::placeholders::_1));

  position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_position_controller/commands", 10);

  velocity_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_velocity_controller/commands", 10);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&VehicleController::timer_callback, this));
}

std::pair<double, double> VehicleController::ackermann_steering_angle()
{
  double left_wheel_angle{0.0};
  double right_wheel_angle{0.0};

  if (std::abs(steering_angle_) > 1e-3) {
    const double sin_angle = std::sin(std::abs(steering_angle_));
    const double cos_angle = std::cos(std::abs(steering_angle_));

    if (steering_angle_ > 0.0) {
      left_wheel_angle = std::atan(
        (2 * wheel_base_ * sin_angle) /
        (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
      right_wheel_angle = std::atan(
        (2 * wheel_base_ * sin_angle) /
        (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
    } else {
      left_wheel_angle = -std::atan(
        (2 * wheel_base_ * sin_angle) /
        (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
      right_wheel_angle = -std::atan(
        (2 * wheel_base_ * sin_angle) /
        (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
    }
  }

  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

std::pair<double, double> VehicleController::rear_differential_velocity()
{
  double left_wheel_velocity{velocity_};
  double right_wheel_velocity{velocity_};

  if (std::abs(steering_angle_) > 1e-3) {
    const double turning_radius = wheel_base_ / std::tan(std::abs(steering_angle_));
    const double vehicle_angular_velocity = velocity_ / turning_radius;
    const double inner_radius = turning_radius - (track_width_ / 2.0);
    const double outer_radius = turning_radius + (track_width_ / 2.0);

    if (steering_angle_ > 0.0) {
      left_wheel_velocity = vehicle_angular_velocity * inner_radius;
      right_wheel_velocity = vehicle_angular_velocity * outer_radius;
    } else {
      left_wheel_velocity = vehicle_angular_velocity * outer_radius;
      right_wheel_velocity = vehicle_angular_velocity * inner_radius;
    }

    const double max_wheel_velocity = std::max(
      std::abs(left_wheel_velocity), std::abs(right_wheel_velocity));
    if (max_wheel_velocity > max_velocity_) {
      const double scaling_factor = max_velocity_ / max_wheel_velocity;
      left_wheel_velocity *= scaling_factor;
      right_wheel_velocity *= scaling_factor;
    }
  }

  return std::make_pair(left_wheel_velocity, right_wheel_velocity);
}

void VehicleController::timer_callback()
{
  const auto current_time{get_clock()->now()};
  const auto velocity_elapsed{(current_time - last_velocity_time_).nanoseconds()};
  const auto steering_elapsed{(current_time - last_steering_time_).nanoseconds()};

  if (velocity_elapsed > timeout_duration_) {
    wheel_angular_velocity_ = {0.0, 0.0};
  }
  if (steering_elapsed > timeout_duration_) {
    wheel_steering_angle_ = {0.0, 0.0};
  }

  std_msgs::msg::Float64MultiArray position_msg;
  position_msg.data = wheel_steering_angle_;
  position_publisher_->publish(position_msg);

  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data = wheel_angular_velocity_;
  velocity_publisher_->publish(velocity_msg);
}

void VehicleController::steering_angle_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  last_steering_time_ = get_clock()->now();

  if (msg->data > max_steering_angle_) {
    steering_angle_ = max_steering_angle_;
  } else if (msg->data < -max_steering_angle_) {
    steering_angle_ = -max_steering_angle_;
  } else {
    steering_angle_ = msg->data;
  }

  const auto wheel_angles{ackermann_steering_angle()};
  wheel_steering_angle_ = {wheel_angles.first, wheel_angles.second};
}

void VehicleController::velocity_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  last_velocity_time_ = get_clock()->now();

  if (msg->data > max_velocity_) {
    velocity_ = max_velocity_;
  } else if (msg->data < -max_velocity_) {
    velocity_ = -max_velocity_;
  } else {
    velocity_ = msg->data;
  }

  const auto wheel_velocity{rear_differential_velocity()};
  wheel_angular_velocity_ = {
    wheel_velocity.first / wheel_radius_,
    wheel_velocity.second / wheel_radius_};
}

}  // namespace ackermann_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ackermann_control::VehicleController>());
  rclcpp::shutdown();
  return 0;
}
