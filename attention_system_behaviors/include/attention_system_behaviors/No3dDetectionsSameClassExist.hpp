#ifndef ATTENTION_SYSTEM_BEHAVIORS__NO3D_DETECTIONS_SAME_CLASS_EXIST_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__NO3D_DETECTIONS_SAME_CLASS_EXIST_HPP

#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace attention_system_behaviors
{

class No3dDetectionsSameClassExist : public BT::ConditionNode
{
public:
  explicit No3dDetectionsSameClassExist(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("class")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;

  vision_msgs::msg::Detection3DArray::SharedPtr last_detections_msg_;

  bool detection_has_valid_3d_position(
    const vision_msgs::msg::Detection3D & detection) const;

  bool no_detections_same_class_exist(const std::string & class_name);

  void detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__NO3D_DETECTIONS_SAME_CLASS_EXIST_HPP
