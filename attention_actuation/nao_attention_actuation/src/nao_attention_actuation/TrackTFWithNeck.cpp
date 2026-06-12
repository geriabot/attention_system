#include "nao_attention_actuation/TrackTFWithNeck.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/exceptions.h"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace nao_attention_actuation
{

TrackTFWithNeck::TrackTFWithNeck()
: rclcpp::Node("track_tf_with_neck")
{
  this->declare_parameter("head_frame_id", "Head");
  this->declare_parameter("yaw_kp", 0.1);
  this->declare_parameter("pitch_kp", 0.1);
  this->declare_parameter("max_yaw_delta_per_tick", 0.2);
  this->declare_parameter("max_pitch_delta_per_tick", 0.2);
  this->declare_parameter("yaw_deadband", 0.01);
  this->declare_parameter("pitch_deadband", 0.01);
  this->declare_parameter("min_head_yaw", -1.0);
  this->declare_parameter("max_head_yaw", 1.0);
  this->declare_parameter("min_head_pitch", -0.5);
  this->declare_parameter("max_head_pitch", 0.3);
  this->declare_parameter("publish_test_traces", false);

  this->get_parameter("head_frame_id", head_frame_id_);
  this->get_parameter("yaw_kp", yaw_kp_);
  this->get_parameter("pitch_kp", pitch_kp_);
  this->get_parameter("max_yaw_delta_per_tick", max_yaw_delta_per_tick_);
  this->get_parameter("max_pitch_delta_per_tick", max_pitch_delta_per_tick_);
  this->get_parameter("yaw_deadband", yaw_deadband_);
  this->get_parameter("pitch_deadband", pitch_deadband_);
  this->get_parameter("min_head_yaw", min_head_yaw_);
  this->get_parameter("max_head_yaw", max_head_yaw_);
  this->get_parameter("min_head_pitch", min_head_pitch_);
  this->get_parameter("max_head_pitch", max_head_pitch_);
  this->get_parameter("publish_test_traces", publish_test_traces_);

  current_head_yaw_ = 0.0;
  current_head_pitch_ = 0.0;
  commanded_head_yaw_ = 0.0;
  commanded_head_pitch_ = 0.0;
  has_joint_positions_ = false;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  joint_positions_pub_ = this->create_publisher<nao_lola_command_msgs::msg::JointPositions>(
    "/effectors/joint_positions",
    10);
  if (publish_test_traces_) {
    test_traces_errors_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/test_traces/track_tf_neck/errors",
      10);
    test_traces_target_delta_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/test_traces/track_tf_neck/target_delta",
      10);
  }
  joint_positions_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions",
    10,
    std::bind(&TrackTFWithNeck::joint_positions_cb, this, _1));
  tracking_timer_ = nullptr;

  start_tracking_srv_ =
    this->create_service<attention_actuation_msgs::srv::StartTracking>(
    "/start_tf_tracking",
    std::bind(&TrackTFWithNeck::start_cb, this, _1, _2));

  stop_tracking_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/stop_tf_tracking",
    std::bind(&TrackTFWithNeck::stop_cb, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "Created track_tf_with_neck node");
}

void
TrackTFWithNeck::start_cb(
  const std::shared_ptr<attention_actuation_msgs::srv::StartTracking::Request>
  request,
  std::shared_ptr<attention_actuation_msgs::srv::StartTracking::Response>
  response)
{
  frame_id_ = request->frame_id;

  if (tracking_timer_ != nullptr) {
    tracking_timer_->cancel();
    tracking_timer_ = nullptr;
  }

  tracking_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&TrackTFWithNeck::tracking_timer_cb, this));

  if (has_joint_positions_) {
    commanded_head_yaw_ = current_head_yaw_;
    commanded_head_pitch_ = current_head_pitch_;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[TRACK TF WITH NECK] Start service initiated (%s)",
    frame_id_.c_str());

  response->success = true;
  response->message = "NAO TF tracking start requested.";
}

void
TrackTFWithNeck::stop_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  if (tracking_timer_ != nullptr) {
    tracking_timer_->cancel();
    tracking_timer_ = nullptr;
  }

  frame_id_.clear();

  RCLCPP_INFO(this->get_logger(), "[TRACK TF WITH NECK] Stop service initiated");

  response->success = true;
  response->message = "NAO TF tracking stop requested.";
}

void
TrackTFWithNeck::joint_positions_cb(
  const nao_lola_sensor_msgs::msg::JointPositions::SharedPtr msg)
{
  current_head_yaw_ = msg->positions[nao_lola_command_msgs::msg::JointIndexes::HEADYAW];
  current_head_pitch_ = msg->positions[nao_lola_command_msgs::msg::JointIndexes::HEADPITCH];
  has_joint_positions_ = true;
}

void
TrackTFWithNeck::tracking_timer_cb()
{
  if (frame_id_.empty()) {
    return;
  }

  if (!has_joint_positions_) {
    RCLCPP_WARN(
      this->get_logger(),
      "[TRACK TF WITH NECK] Waiting for joint positions on sensors/joint_positions");
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped = tf_buffer_->lookupTransform(
      head_frame_id_,
      frame_id_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "[TRACK TF WITH NECK] Could not get transform from %s to %s: %s",
      head_frame_id_.c_str(),
      frame_id_.c_str(),
      ex.what());
    return;
  }

  const double x = transform_stamped.transform.translation.x;
  const double y = transform_stamped.transform.translation.y;
  const double z = transform_stamped.transform.translation.z;
  const double yaw_error = std::atan2(y, x);
  const double pitch_error = std::atan2(-z, std::sqrt((x * x) + (y * y)));
  // The TF gives the angular correction still needed from the current head frame.
  const double yaw_delta = compute_incremental_delta(
    yaw_error,
    yaw_kp_,
    max_yaw_delta_per_tick_,
    yaw_deadband_);

  const double pitch_delta = compute_incremental_delta(
    pitch_error,
    pitch_kp_,
    max_pitch_delta_per_tick_,
    pitch_deadband_);

  const double target_head_yaw = std::clamp(
    // current_head_yaw_ (+) --> left
    // target_head_yaw (+) --> right
    current_head_yaw_ + yaw_delta,
    min_head_yaw_,
    max_head_yaw_);
  const double target_head_pitch = std::clamp(
    current_head_pitch_ + pitch_delta,
    min_head_pitch_,
    max_head_pitch_);

  if (publish_test_traces_ &&
    test_traces_errors_pub_ != nullptr &&
    test_traces_target_delta_pub_ != nullptr)
  {
    std_msgs::msg::Float64MultiArray errors_msg;
    errors_msg.data = {yaw_error, pitch_error};
    test_traces_errors_pub_->publish(errors_msg);

    std_msgs::msg::Float64MultiArray target_delta_msg;
    target_delta_msg.data = {target_head_yaw, target_head_pitch};
    test_traces_target_delta_pub_->publish(target_delta_msg);
  }

  if (yaw_delta == 0.0 && pitch_delta == 0.0) {
    return;
  }

  commanded_head_yaw_ = target_head_yaw;
  commanded_head_pitch_ = target_head_pitch;

  nao_lola_command_msgs::msg::JointPositions joint_positions_msg;

  joint_positions_msg.indexes = {
    nao_lola_command_msgs::msg::JointIndexes::HEADYAW,
    nao_lola_command_msgs::msg::JointIndexes::HEADPITCH
  };
  joint_positions_msg.positions = {
    static_cast<float>(target_head_yaw),
    static_cast<float>(target_head_pitch)
  };

  joint_positions_pub_->publish(joint_positions_msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[TRACK TF WITH NECK] frame_id=%s yaw_error=%.4f pitch_error=%.4f yaw_delta=%.4f pitch_delta=%.4f command_yaw=%.4f command_pitch=%.4f",
    frame_id_.c_str(),
    yaw_error,
    pitch_error,
    yaw_delta,
    pitch_delta,
    target_head_yaw,
    target_head_pitch);
}

double
TrackTFWithNeck::compute_incremental_delta(
  double target_angle,
  double proportional_gain,
  double max_delta_per_tick,
  double deadband) const
{
  // Ignore tiny corrections to avoid jitter around the target.
  if (std::abs(target_angle) <= deadband) {
    return 0.0;
  }

  const double proportional_delta = proportional_gain * target_angle;
  // Bound each iteration so the neck motion remains progressive.
  const double limited_delta = std::clamp(
    proportional_delta,
    -max_delta_per_tick,
    max_delta_per_tick);

  // Prevent overshooting when the remaining correction is smaller than the bounded step.
  if (std::abs(limited_delta) > std::abs(target_angle)) {
    return target_angle;
  }

  return limited_delta;
}

}  // namespace nao_attention_actuation
