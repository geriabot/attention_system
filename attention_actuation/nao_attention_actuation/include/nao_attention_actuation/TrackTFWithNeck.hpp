#ifndef NAO_ATTENTION_ACTUATION__TRACK_TF_WITH_NECK_HPP_
#define NAO_ATTENTION_ACTUATION__TRACK_TF_WITH_NECK_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "attention_actuation_msgs/srv/start_tracking.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nao_attention_actuation
{

class TrackTFWithNeck : public rclcpp::Node
{
public:
  TrackTFWithNeck();
  virtual ~TrackTFWithNeck() = default;

private:
  std::string frame_id_;
  std::string head_frame_id_;
  double yaw_kp_;
  double pitch_kp_;
  double max_yaw_delta_per_tick_;
  double max_pitch_delta_per_tick_;
  double yaw_deadband_;
  double pitch_deadband_;
  double min_head_yaw_;
  double max_head_yaw_;
  double min_head_pitch_;
  double max_head_pitch_;
  double current_head_yaw_;
  double current_head_pitch_;
  double commanded_head_yaw_;
  double commanded_head_pitch_;
  bool publish_test_traces_;
  bool has_joint_positions_;

  rclcpp::Service<attention_actuation_msgs::srv::StartTracking>::SharedPtr
    start_tracking_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_tracking_srv_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr
    joint_positions_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_traces_errors_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    test_traces_target_delta_pub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr
    joint_positions_sub_;
  rclcpp::TimerBase::SharedPtr tracking_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void start_cb(
    const std::shared_ptr<attention_actuation_msgs::srv::StartTracking::Request>
    request,
    std::shared_ptr<attention_actuation_msgs::srv::StartTracking::Response>
    response);

  void stop_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void joint_positions_cb(
    const nao_lola_sensor_msgs::msg::JointPositions::SharedPtr msg);

  void tracking_timer_cb();

  double
  compute_incremental_delta(
    double target_angle,
    double proportional_gain,
    double max_delta_per_tick,
    double deadband) const;
};

}  // namespace nao_attention_actuation

#endif  // NAO_ATTENTION_ACTUATION__TRACK_TF_WITH_NECK_HPP_
