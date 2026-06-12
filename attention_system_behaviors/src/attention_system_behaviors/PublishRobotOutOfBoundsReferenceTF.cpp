#include "attention_system_behaviors/PublishRobotOutOfBoundsReferenceTF.hpp"

#include "attention_system_behaviors/IsRobotOutOfBounds.hpp"
#include "tf2/exceptions.h"

namespace attention_system_behaviors
{

PublishRobotOutOfBoundsReferenceTF::PublishRobotOutOfBoundsReferenceTF(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError(
      "PublishRobotOutOfBoundsReferenceTF: 'node' not found in blackboard");
  }

  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

BT::NodeStatus
PublishRobotOutOfBoundsReferenceTF::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  std::string robot_reference_frame;
  std::string fixed_reference_frame;

  if (!getInput("robot_reference_frame", robot_reference_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'robot_reference_frame' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("fixed_reference_frame", fixed_reference_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'fixed_reference_frame' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  try {
    transform_stamped_ = tf_buffer_->lookupTransform(
      fixed_reference_frame,
      robot_reference_frame,
      tf2::TimePointZero);

    transform_stamped_.child_frame_id = IsRobotOutOfBounds::initial_reference_frame_;
    transform_stamped_.header.stamp = node_->now();
    tf_broadcaster_->sendTransform(transform_stamped_);

    RCLCPP_INFO(
      node_->get_logger(),
      "Published robot out-of-bounds reference TF '%s' with parent '%s'.",
      IsRobotOutOfBounds::initial_reference_frame_,
      fixed_reference_frame.c_str());

    return BT::NodeStatus::RUNNING;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Could not publish robot out-of-bounds reference TF from '%s' to '%s': %s",
      fixed_reference_frame.c_str(),
      robot_reference_frame.c_str(),
      exception.what());
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus
PublishRobotOutOfBoundsReferenceTF::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  transform_stamped_.header.stamp = node_->now();
  tf_broadcaster_->sendTransform(transform_stamped_);

  return BT::NodeStatus::RUNNING;
}

void
PublishRobotOutOfBoundsReferenceTF::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "Halted ROOBR TF publisher");
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::PublishRobotOutOfBoundsReferenceTF>(
    "PublishRobotOutOfBoundsReferenceTF");
}

#endif
