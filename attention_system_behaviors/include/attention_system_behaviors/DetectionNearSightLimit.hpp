#ifndef ATTENTION_SYSTEM_BEHAVIORS__DETECTION_NEAR_SIGHT_LIMIT_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__DETECTION_NEAR_SIGHT_LIMIT_HPP

#include <mutex>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace attention_system_behaviors
{

class DetectionNearSightLimit : public BT::ConditionNode
{
public:
  explicit DetectionNearSightLimit(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("side"),
        BT::InputPort<std::string>("class"),
        BT::InputPort<double>("limit_percentage")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
    detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  vision_msgs::msg::Detection2DArray::SharedPtr last_detections_msg_;
  sensor_msgs::msg::CameraInfo::SharedPtr last_camera_info_msg_;
  mutable std::mutex messages_mutex_;

  bool detection_near_sight_limit(
    const std::string & side,
    const std::string & class_name,
    double limit_percentage) const;
  void detections_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg);
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__DETECTION_NEAR_SIGHT_LIMIT_HPP
