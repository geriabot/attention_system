#ifndef ATTENTION_SYSTEM_BEHAVIORS__DETECTION_EXISTS_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__DETECTION_EXISTS_HPP

#include <mutex>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace attention_system_behaviors
{

class DetectionExists : public BT::ConditionNode
{
public:

  explicit DetectionExists(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("class"),
        BT::InputPort<std::string>("id")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;

  std::string topic_;
  vision_msgs::msg::Detection3DArray::SharedPtr last_detections_msg_;

  bool read_detection_id(std::string & detection_id);
  void update_subscription(const std::string & topic);
  bool detection_exists(
    const std::string & class_name,
    const std::string & detection_id);
  void detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__DETECTION_EXISTS_HPP
