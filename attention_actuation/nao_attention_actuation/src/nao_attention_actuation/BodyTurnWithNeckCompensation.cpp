#include "nao_attention_actuation/BodyTurnWithNeckCompensation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"

using std::placeholders::_1;
using std::placeholders::_2;

namespace nao_attention_actuation
{

BodyTurnWithNeckCompensation::BodyTurnWithNeckCompensation()
: rclcpp::Node("body_turn_with_neck_compensation")
{
  this->declare_parameter("reference_frame", "odom");
  this->declare_parameter("base_frame_id", "base_link");
  this->declare_parameter("head_frame_id", "Head");
  this->declare_parameter("control_period_ms", 100);
  this->declare_parameter("body_turn_speed", 0.2);
  this->declare_parameter("yaw_kp", 0.1);
  this->declare_parameter("max_yaw_delta_per_tick", 0.2);
  this->declare_parameter("yaw_deadband", 0.01);
  this->declare_parameter("min_head_yaw", -1.0);
  this->declare_parameter("max_head_yaw", 1.0);
  this->declare_parameter("tf_lookup_timeout_ms", 100);
  this->declare_parameter("max_tf_failures_before_abort", 5);

  this->get_parameter("reference_frame", reference_frame_);
  this->get_parameter("base_frame_id", base_frame_id_);
  this->get_parameter("head_frame_id", head_frame_id_);
  this->get_parameter("control_period_ms", control_period_ms_);
  this->get_parameter("body_turn_speed", body_turn_speed_);
  this->get_parameter("yaw_kp", yaw_kp_);
  this->get_parameter("max_yaw_delta_per_tick", max_yaw_delta_per_tick_);
  this->get_parameter("yaw_deadband", yaw_deadband_);
  this->get_parameter("min_head_yaw", min_head_yaw_);
  this->get_parameter("max_head_yaw", max_head_yaw_);
  this->get_parameter("tf_lookup_timeout_ms", tf_lookup_timeout_ms_);
  this->get_parameter("max_tf_failures_before_abort", max_tf_failures_before_abort_);

  current_head_yaw_ = 0.0;
  has_joint_positions_ = false;
  goal_active_ = false;
  last_valid_body_error_ = 0.0;
  initial_head_world_yaw_ = 0.0;
  target_body_yaw_ = 0.0;
  consecutive_tf_failures_ = 0;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  target_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/target", 10);
  joint_positions_pub_ = this->create_publisher<nao_lola_command_msgs::msg::JointPositions>(
    "/effectors/joint_positions",
    10);
  joint_positions_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions",
    10,
    std::bind(&BodyTurnWithNeckCompensation::joint_positions_cb, this, _1));

  action_server_ =
    rclcpp_action::create_server<TurnBodyWithNeckCompensation>(
    this,
    "/nao_attention/turn_body_with_neck_compensation",
    std::bind(&BodyTurnWithNeckCompensation::handle_goal, this, _1, _2),
    std::bind(&BodyTurnWithNeckCompensation::handle_cancel, this, _1),
    std::bind(&BodyTurnWithNeckCompensation::handle_accepted, this, _1));

  feedback_msg_ = std::make_shared<TurnBodyWithNeckCompensation::Feedback>();
  control_timer_ = nullptr;

  RCLCPP_INFO(this->get_logger(), "Created body_turn_with_neck_compensation node");
}

void
BodyTurnWithNeckCompensation::joint_positions_cb(
  const nao_lola_sensor_msgs::msg::JointPositions::SharedPtr msg)
{
  current_head_yaw_ = msg->positions[nao_lola_command_msgs::msg::JointIndexes::HEADYAW];
  has_joint_positions_ = true;
}

rclcpp_action::GoalResponse
BodyTurnWithNeckCompensation::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const TurnBodyWithNeckCompensation::Goal> goal)
{
  (void)uuid;

  if (goal_active_) {
    RCLCPP_WARN(this->get_logger(), "Rejected goal because another goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!has_joint_positions_) {
    RCLCPP_WARN(this->get_logger(), "Rejected goal because no joint positions were received");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->threshold <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Rejected goal because threshold must be positive");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!validate_goal_frames(goal->frame_id)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Rejected goal because required TFs are not available for frame_id=%s",
      goal->frame_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Accepted goal for frame_id=%s threshold=%.4f",
    goal->frame_id.c_str(),
    goal->threshold);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
BodyTurnWithNeckCompensation::handle_cancel(
  const std::shared_ptr<GoalHandleTurnBodyWithNeckCompensation> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
BodyTurnWithNeckCompensation::handle_accepted(
  const std::shared_ptr<GoalHandleTurnBodyWithNeckCompensation> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  if (!get_frame_yaw(reference_frame_, head_frame_id_, initial_head_world_yaw_)) {
    auto result = std::make_shared<TurnBodyWithNeckCompensation::Result>();
    result->success = false;
    result->final_error = static_cast<float>(std::abs(last_valid_body_error_));
    goal_handle->abort(result);
    return;
  }

  active_goal_handle_ = goal_handle;
  active_frame_id_ = goal->frame_id;
  target_body_yaw_ = initial_head_world_yaw_;
  goal_active_ = true;
  last_valid_body_error_ = 0.0;
  consecutive_tf_failures_ = 0;

  RCLCPP_INFO(
    this->get_logger(),
    "Starting body turn with neck compensation for frame_id=%s target_body_yaw=%.4f initial_head_yaw=%.4f",
    goal->frame_id.c_str(),
    target_body_yaw_,
    get_current_head_yaw());

  if (control_timer_ != nullptr) {
    control_timer_->cancel();
  }

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms_),
    std::bind(&BodyTurnWithNeckCompensation::control_timer_cb, this));
}

void
BodyTurnWithNeckCompensation::control_timer_cb()
{
  if (!goal_active_ || active_goal_handle_ == nullptr) {
    return;
  }

  const auto goal = active_goal_handle_->get_goal();

  if (active_goal_handle_->is_canceling()) {
    finish_active_goal(false, true, std::abs(last_valid_body_error_));
    return;
  }

  double current_base_yaw = 0.0;
  double current_head_world_yaw = 0.0;
  const bool has_base_yaw = get_frame_yaw(reference_frame_, base_frame_id_, current_base_yaw);
  const bool has_head_world_yaw = get_frame_yaw(reference_frame_, head_frame_id_, current_head_world_yaw);

  if (!has_base_yaw || !has_head_world_yaw) {
    consecutive_tf_failures_++;
    if (consecutive_tf_failures_ >= max_tf_failures_before_abort_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Aborting goal because TF lookup failed %d consecutive times",
        consecutive_tf_failures_);
      finish_active_goal(false, false, std::abs(last_valid_body_error_));
    }
    return;
  }

  consecutive_tf_failures_ = 0;

  const double body_yaw_error = normalize_angle(target_body_yaw_ - current_base_yaw);
  last_valid_body_error_ = body_yaw_error;

  feedback_msg_->remaining_yaw = static_cast<float>(std::abs(body_yaw_error));
  active_goal_handle_->publish_feedback(feedback_msg_);

  if (std::abs(body_yaw_error) <= static_cast<double>(goal->threshold)) {
    finish_active_goal(true, false, std::abs(body_yaw_error));
    return;
  }

  const double commanded_turn_speed = body_yaw_error > 0.0 ? body_turn_speed_ : -body_turn_speed_;
  publish_body_turn(commanded_turn_speed);

  const double head_world_yaw_error = normalize_angle(
    initial_head_world_yaw_ - current_head_world_yaw);
  const double head_yaw_delta = compute_incremental_delta(
    head_world_yaw_error,
    yaw_kp_,
    max_yaw_delta_per_tick_,
    yaw_deadband_);

  if (head_yaw_delta == 0.0) {
    return;
  }

  const double target_head_yaw = std::clamp(
    get_current_head_yaw() + head_yaw_delta,
    min_head_yaw_,
    max_head_yaw_);

  publish_head_yaw(target_head_yaw);

  if ((target_head_yaw == min_head_yaw_ || target_head_yaw == max_head_yaw_) &&
    std::abs(head_world_yaw_error) > yaw_deadband_)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Aborting goal because head yaw reached its limit while compensating");
    finish_active_goal(false, false, std::abs(body_yaw_error));
  }
}

void
BodyTurnWithNeckCompensation::finish_active_goal(
  bool success,
  bool canceled,
  double final_error)
{
  for (int i = 0; i < 5; i++) {
    publish_body_turn(0.0);
  }

  if (control_timer_ != nullptr) {
    control_timer_->cancel();
    control_timer_ = nullptr;
  }

  auto result = std::make_shared<TurnBodyWithNeckCompensation::Result>();
  result->success = success;
  result->final_error = static_cast<float>(final_error);

  if (active_goal_handle_ != nullptr) {
    if (canceled) {
      active_goal_handle_->canceled(result);
    } else if (success) {
      active_goal_handle_->succeed(result);
    } else {
      active_goal_handle_->abort(result);
    }
  }

  goal_active_ = false;
  active_goal_handle_.reset();
  active_frame_id_.clear();
  consecutive_tf_failures_ = 0;
}

double
BodyTurnWithNeckCompensation::compute_incremental_delta(
  double target_angle,
  double proportional_gain,
  double max_delta_per_tick,
  double deadband) const
{
  if (std::abs(target_angle) <= deadband) {
    return 0.0;
  }

  const double proportional_delta = proportional_gain * target_angle;
  const double limited_delta = std::clamp(
    proportional_delta,
    -max_delta_per_tick,
    max_delta_per_tick);

  if (std::abs(limited_delta) > std::abs(target_angle)) {
    return target_angle;
  }

  return limited_delta;
}

double
BodyTurnWithNeckCompensation::normalize_angle(
  double angle) const
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool
BodyTurnWithNeckCompensation::get_frame_yaw(
  const std::string & source_frame,
  const std::string & target_frame,
  double & yaw)
{
  try {
    const auto transform_stamped = tf_buffer_->lookupTransform(
      source_frame,
      target_frame,
      tf2::TimePointZero,
      std::chrono::milliseconds(tf_lookup_timeout_ms_));

    tf2::Quaternion rotation(
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not get transform from %s to %s: %s",
      source_frame.c_str(),
      target_frame.c_str(),
      ex.what());
    return false;
  }
}

bool
BodyTurnWithNeckCompensation::validate_goal_frames(
  const std::string & frame_id)
{
  double frame_yaw = 0.0;
  if (!get_frame_yaw(reference_frame_, base_frame_id_, frame_yaw)) {
    return false;
  }

  if (!get_frame_yaw(reference_frame_, head_frame_id_, frame_yaw)) {
    return false;
  }

  try {
    (void)tf_buffer_->lookupTransform(
      head_frame_id_,
      frame_id,
      tf2::TimePointZero,
      std::chrono::milliseconds(tf_lookup_timeout_ms_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not validate transform from %s to %s: %s",
      head_frame_id_.c_str(),
      frame_id.c_str(),
      ex.what());
    return false;
  }

  return true;
}

void
BodyTurnWithNeckCompensation::publish_body_turn(
  double angular_z)
{
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = 0.0;
  twist_msg.linear.x = 0.0;
  twist_msg.linear.x = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = angular_z;
  target_pub_->publish(twist_msg);
}

void
BodyTurnWithNeckCompensation::publish_head_yaw(
  double target_head_yaw)
{
  nao_lola_command_msgs::msg::JointPositions joint_positions_msg;
  joint_positions_msg.indexes = {
    nao_lola_command_msgs::msg::JointIndexes::HEADYAW
  };
  joint_positions_msg.positions = {
    static_cast<float>(target_head_yaw)
  };
  joint_positions_pub_->publish(joint_positions_msg);
}

double
BodyTurnWithNeckCompensation::get_current_head_yaw() const
{
  return current_head_yaw_;
}

}  // namespace nao_attention_actuation
