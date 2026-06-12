#ifndef KOBUKI_ATTENTION_ACTUATION__TRACK_TF_STATIC_HPP_
#define KOBUKI_ATTENTION_ACTUATION__TRACK_TF_STATIC_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "attention_actuation_msgs/srv/start_tracking.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace kobuki_attention_actuation
{

class TrackTFStatic : public rclcpp::Node
{
public:
  TrackTFStatic();
  virtual ~TrackTFStatic() = default;

private:
  std::string frame_id_;
  std::string base_frame_id_;
  std::string cmd_vel_topic_;
  int control_period_ms_;
  double yaw_kp_;
  double yaw_ki_;
  double yaw_kd_;
  double max_angular_speed_;
  double yaw_deadband_;
  bool publish_test_traces_;
  double yaw_integral_;
  double previous_yaw_error_;
  bool has_previous_error_;

  rclcpp::Service<attention_actuation_msgs::srv::StartTracking>::SharedPtr
    start_tracking_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_tracking_srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_traces_errors_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_traces_control_pub_;
  rclcpp::TimerBase::SharedPtr tracking_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Time previous_control_time_;

  void start_cb(
    const std::shared_ptr<attention_actuation_msgs::srv::StartTracking::Request>
    request,
    std::shared_ptr<attention_actuation_msgs::srv::StartTracking::Response>
    response);

  void stop_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void tracking_timer_cb();

  double
  compute_angular_command(
    double yaw_error,
    double dt_seconds);

  void
  reset_pid_state();

  void
  publish_cmd_vel(
    double angular_z);

  double
  clamp_with_deadband(
    double value,
    double max_absolute_value,
    double deadband) const;
};

}  // namespace kobuki_attention_actuation

#endif  // KOBUKI_ATTENTION_ACTUATION__TRACK_TF_STATIC_HPP_
