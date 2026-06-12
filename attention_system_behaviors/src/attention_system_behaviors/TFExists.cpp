#include "attention_system_behaviors/TFExists.hpp"

#include "tf2/time.h"

namespace attention_system_behaviors
{

TFExists::TFExists(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("TFExists: 'node' not found in blackboard");
  }

  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus
TFExists::tick()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  std::string target_frame;
  std::string source_frame;
  bool expected_exists;
  double timeout_sec;

  if (!getInput("target_frame", target_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'target_frame' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("source_frame", source_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'source_frame' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("expected_exists", expected_exists)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'expected_exists' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("timeout_sec", timeout_sec)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'timeout_sec' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (timeout_sec < 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'timeout_sec' must be greater than or equal to zero.");
    return BT::NodeStatus::FAILURE;
  }

  std::string error;
  const bool transform_exists = tf_buffer_->canTransform(
    target_frame,
    source_frame,
    tf2::TimePointZero,
    tf2::durationFromSec(timeout_sec),
    &error);

  if (transform_exists == expected_exists) {
    //RCLCPP_INFO(
    //  node_->get_logger(),
    //  "TFExists satisfied: target_frame=%s, source_frame=%s, expected_exists=%d",
    //  target_frame.c_str(),
    //  source_frame.c_str(),
    //  (int) expected_exists);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "TFExists not satisfied: target_frame=%s, source_frame=%s, expected_exists=%d, error=%s",
    target_frame.c_str(),
    source_frame.c_str(),
    (int) expected_exists,
    error.c_str());

  return BT::NodeStatus::FAILURE;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::TFExists>("TFExists");
}

#endif
