#include "attention_system_behaviors/N3dDetectionsSameClassExist.hpp"

#include <cmath>

using std::placeholders::_1;

namespace attention_system_behaviors
{

N3dDetectionsSameClassExist::N3dDetectionsSameClassExist(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("N3dDetectionsSameClassExist: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  last_detections_msg_ = nullptr;

  detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "/detections_3d",
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&N3dDetectionsSameClassExist::detections_callback, this, _1));
}

BT::NodeStatus
N3dDetectionsSameClassExist::tick()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  std::string class_name;
  int n_detections;

  if (!getInput("class", class_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'class' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("n_detections", n_detections)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'n_detections' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (n_detections <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'n_detections' must be greater than zero.");
    return BT::NodeStatus::FAILURE;
  }

  if (n_detections_same_class_exist(class_name, n_detections)) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

bool
N3dDetectionsSameClassExist::detection_has_valid_3d_position(
  const vision_msgs::msg::Detection3D & detection) const
{
  const auto & position = detection.bbox.center.position;

  return std::isfinite(position.x) &&
         std::isfinite(position.y) &&
         std::isfinite(position.z);
}

bool
N3dDetectionsSameClassExist::n_detections_same_class_exist(
  const std::string & class_name,
  int n_detections)
{
  if (!last_detections_msg_) {
    return false;
  }

  int matching_detections = 0;

  for (const auto & detection : last_detections_msg_->detections) {
    if (detection.results.empty()) {
      continue;
    }

    if (!detection_has_valid_3d_position(detection)) {
      continue;
    }

    if (detection.results[0].hypothesis.class_id != class_name) {
      continue;
    }

    matching_detections++;

    if (matching_detections >= n_detections) {
      //RCLCPP_INFO(
      //  node_->get_logger(),
      //  "%d detections of class '%s' exist.",
      //  n_detections,
      //  class_name.c_str());
      return true;
    }
  }

  //RCLCPP_INFO(
  //  node_->get_logger(),
  //  "%d detections of class '%s' do not exist.",
  //  n_detections,
  //  class_name.c_str());

  return false;
}

void
N3dDetectionsSameClassExist::detections_callback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  last_detections_msg_ = msg;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::N3dDetectionsSameClassExist>(
    "N3dDetectionsSameClassExist");
}

#endif
