#ifndef NAO_ATTENTION_ACTUATION__BODY_TURN_WITH_NECK_COMPENSATION_HPP_
#define NAO_ATTENTION_ACTUATION__BODY_TURN_WITH_NECK_COMPENSATION_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "attention_actuation_msgs/action/turn_body_with_neck_compensation.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nao_attention_actuation
{

class BodyTurnWithNeckCompensation : public rclcpp::Node
{
public:
  using TurnBodyWithNeckCompensation =
    attention_actuation_msgs::action::TurnBodyWithNeckCompensation;
  using GoalHandleTurnBodyWithNeckCompensation =
    rclcpp_action::ServerGoalHandle<TurnBodyWithNeckCompensation>;

  BodyTurnWithNeckCompensation();
  virtual ~BodyTurnWithNeckCompensation() = default;

private:
  std::string reference_frame_;
  std::string base_frame_id_;
  std::string head_frame_id_;
  int control_period_ms_;
  double body_turn_speed_;
  double yaw_kp_;
  double max_yaw_delta_per_tick_;
  double yaw_deadband_;
  double min_head_yaw_;
  double max_head_yaw_;
  int tf_lookup_timeout_ms_;
  int max_tf_failures_before_abort_;

  double current_head_yaw_;
  bool has_joint_positions_;
  bool goal_active_;
  double last_valid_body_error_;
  double initial_head_world_yaw_;
  double target_body_yaw_;
  int consecutive_tf_failures_;
  std::string active_frame_id_;

  rclcpp_action::Server<TurnBodyWithNeckCompensation>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleTurnBodyWithNeckCompensation> active_goal_handle_;
  std::shared_ptr<TurnBodyWithNeckCompensation::Feedback> feedback_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr target_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr joint_positions_pub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr joint_positions_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void joint_positions_cb(
    const nao_lola_sensor_msgs::msg::JointPositions::SharedPtr msg);

  rclcpp_action::GoalResponse
  handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TurnBodyWithNeckCompensation::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(
    const std::shared_ptr<GoalHandleTurnBodyWithNeckCompensation> goal_handle);

  void
  handle_accepted(
    const std::shared_ptr<GoalHandleTurnBodyWithNeckCompensation> goal_handle);

  void
  control_timer_cb();

  void
  finish_active_goal(
    bool success,
    bool canceled,
    double final_error);

  double
  compute_incremental_delta(
    double target_angle,
    double proportional_gain,
    double max_delta_per_tick,
    double deadband) const;

  double
  normalize_angle(
    double angle) const;

  bool
  get_frame_yaw(
    const std::string & source_frame,
    const std::string & target_frame,
    double & yaw);

  bool
  validate_goal_frames(
    const std::string & frame_id);

  void
  publish_body_turn(
    double angular_z);

  void
  publish_head_yaw(
    double target_head_yaw);

  double
  get_current_head_yaw() const;
};

}  // namespace nao_attention_actuation

#endif  // NAO_ATTENTION_ACTUATION__BODY_TURN_WITH_NECK_COMPENSATION_HPP_
