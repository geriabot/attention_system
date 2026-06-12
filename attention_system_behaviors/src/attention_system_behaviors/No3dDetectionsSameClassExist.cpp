#include "attention_system_behaviors/No3dDetectionsSameClassExist.hpp"

#include <cmath>

using std::placeholders::_1;

namespace attention_system_behaviors
{

No3dDetectionsSameClassExist::No3dDetectionsSameClassExist(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("No3dDetectionsSameClassExist: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  last_detections_msg_ = nullptr;

  detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "/detections_3d",
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&No3dDetectionsSameClassExist::detections_callback, this, _1));
}

BT::NodeStatus
No3dDetectionsSameClassExist::tick()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  std::string class_name;

  if (!getInput("class", class_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'class' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (no_detections_same_class_exist(class_name)) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

bool
No3dDetectionsSameClassExist::detection_has_valid_3d_position(
  const vision_msgs::msg::Detection3D & detection) const
{
  const auto & position = detection.bbox.center.position;

  return std::isfinite(position.x) &&
         std::isfinite(position.y) &&
         std::isfinite(position.z);
}

bool
No3dDetectionsSameClassExist::no_detections_same_class_exist(
  const std::string & class_name)
{
  if (!last_detections_msg_) {
    return false;
  }

  for (const auto & detection : last_detections_msg_->detections) {
    if (detection.results.empty()) {
      continue;
    }

    if (!detection_has_valid_3d_position(detection)) {
      continue;
    }

    if (detection.results[0].hypothesis.class_id == class_name) {
      RCLCPP_INFO(
        node_->get_logger(),
        "At least one detection of class '%s' exists.",
        class_name.c_str());
      return false;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "No detections of class '%s' exist.",
    class_name.c_str());

  return true;
}

void
No3dDetectionsSameClassExist::detections_callback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  last_detections_msg_ = msg;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::No3dDetectionsSameClassExist>(
    "No3dDetectionsSameClassExist");
}

#endif
