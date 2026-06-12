#include "kobuki_attention_actuation/TrackTFStatic.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace kobuki_attention_actuation
{

TrackTFStatic::TrackTFStatic()
: rclcpp::Node("track_tf_static")
{
  this->declare_parameter("base_frame_id", "base_link");
  this->declare_parameter("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter("control_period_ms", 100);
  this->declare_parameter("yaw_kp", 4.0);
  this->declare_parameter("yaw_ki", 0.0);
  this->declare_parameter("yaw_kd", 0.1);
  this->declare_parameter("max_angular_speed", 1.0);
  this->declare_parameter("yaw_deadband", 0.001);
  this->declare_parameter("publish_test_traces", false);

  this->get_parameter("base_frame_id", base_frame_id_);
  this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  this->get_parameter("control_period_ms", control_period_ms_);
  this->get_parameter("yaw_kp", yaw_kp_);
  this->get_parameter("yaw_ki", yaw_ki_);
  this->get_parameter("yaw_kd", yaw_kd_);
  this->get_parameter("max_angular_speed", max_angular_speed_);
  this->get_parameter("yaw_deadband", yaw_deadband_);
  this->get_parameter("publish_test_traces", publish_test_traces_);

  yaw_integral_ = 0.0;
  previous_yaw_error_ = 0.0;
  has_previous_error_ = false;
  previous_control_time_ = this->now();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic_,
    10);

  if (publish_test_traces_) {
    test_traces_errors_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/test_traces/track_tf_static/errors",
      10);
    test_traces_control_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/test_traces/track_tf_static/control",
      10);
  }

  tracking_timer_ = nullptr;

  // TODO Usar servicios genéricos

  start_tracking_srv_ =
    this->create_service<attention_actuation_msgs::srv::StartTracking>(
    "/start_tf_tracking",
    std::bind(&TrackTFStatic::start_cb, this, _1, _2));

  stop_tracking_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/stop_tf_tracking",
    std::bind(&TrackTFStatic::stop_cb, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "Created track_tf_static node");
}

void
TrackTFStatic::start_cb(
  const std::shared_ptr<attention_actuation_msgs::srv::StartTracking::Request>
  request,
  std::shared_ptr<attention_actuation_msgs::srv::StartTracking::Response>
  response)
{
  if (request->frame_id.empty()) {
    response->success = false;
    response->message = "frame_id cannot be empty.";
    return;
  }

  frame_id_ = request->frame_id;

  if (tracking_timer_ != nullptr) {
    tracking_timer_->cancel();
    tracking_timer_ = nullptr;
  }

  reset_pid_state();
  previous_control_time_ = this->now();

  tracking_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms_),
    std::bind(&TrackTFStatic::tracking_timer_cb, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[TRACK TF STATIC] Start service initiated (%s)",
    frame_id_.c_str());

  response->success = true;
  response->message = "Kobuki TF tracking start requested.";
}

void
TrackTFStatic::stop_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  if (tracking_timer_ != nullptr) {
    tracking_timer_->cancel();
    tracking_timer_ = nullptr;
  }

  frame_id_.clear();
  reset_pid_state();
  publish_cmd_vel(0.0);

  RCLCPP_INFO(this->get_logger(), "[TRACK TF STATIC] Stop service initiated");

  response->success = true;
  response->message = "Kobuki TF tracking stop requested.";
}

void
TrackTFStatic::tracking_timer_cb()
{
  if (frame_id_.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped = tf_buffer_->lookupTransform(
      base_frame_id_,
      frame_id_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "[TRACK TF STATIC] Could not get transform from %s to %s: %s",
      base_frame_id_.c_str(),
      frame_id_.c_str(),
      ex.what());
    publish_cmd_vel(0.0);
    reset_pid_state();
    return;
  }

  const double x = transform_stamped.transform.translation.x;
  const double y = transform_stamped.transform.translation.y;
  const double yaw_error = std::atan2(y, x);

  const rclcpp::Time current_time = this->now();
  double dt_seconds = (current_time - previous_control_time_).seconds();
  previous_control_time_ = current_time;

  if (dt_seconds <= 0.0) {
    dt_seconds = static_cast<double>(control_period_ms_) / 1000.0;
  }

  const double angular_z = compute_angular_command(yaw_error, dt_seconds);

  if (publish_test_traces_ &&
    test_traces_errors_pub_ != nullptr &&
    test_traces_control_pub_ != nullptr)
  {
    std_msgs::msg::Float64MultiArray errors_msg;
    errors_msg.data = {yaw_error};
    test_traces_errors_pub_->publish(errors_msg);

    std_msgs::msg::Float64MultiArray control_msg;
    control_msg.data = {angular_z};
    test_traces_control_pub_->publish(control_msg);
  }

  publish_cmd_vel(angular_z);

  RCLCPP_INFO(
    this->get_logger(),
    "[TRACK TF STATIC] frame_id=%s yaw_error=%.4f angular_z=%.4f",
    frame_id_.c_str(),
    yaw_error,
    angular_z);
}

double
TrackTFStatic::compute_angular_command(
  double yaw_error,
  double dt_seconds)
{
  if (std::abs(yaw_error) <= yaw_deadband_) {
    reset_pid_state();
    return 0.0;
  }

  yaw_integral_ += yaw_error * dt_seconds;

  double derivative = 0.0;
  if (has_previous_error_) {
    derivative = (yaw_error - previous_yaw_error_) / dt_seconds;
  }

  previous_yaw_error_ = yaw_error;
  has_previous_error_ = true;

  const double angular_z =
    (yaw_kp_ * yaw_error) +
    (yaw_ki_ * yaw_integral_) +
    (yaw_kd_ * derivative);

  return clamp_with_deadband(
    angular_z,
    max_angular_speed_,
    yaw_deadband_);
}

void
TrackTFStatic::reset_pid_state()
{
  yaw_integral_ = 0.0;
  previous_yaw_error_ = 0.0;
  has_previous_error_ = false;
}

void
TrackTFStatic::publish_cmd_vel(
  double angular_z)
{
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = angular_z;
  cmd_vel_pub_->publish(twist_msg);
}

double
TrackTFStatic::clamp_with_deadband(
  double value,
  double max_absolute_value,
  double deadband) const
{
  if (std::abs(value) <= deadband) {
    return 0.0;
  }

  return std::clamp(value, -max_absolute_value, max_absolute_value);
}

}  // namespace kobuki_attention_actuation
