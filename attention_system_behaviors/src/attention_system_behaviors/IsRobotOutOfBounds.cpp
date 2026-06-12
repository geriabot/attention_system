#include "attention_system_behaviors/IsRobotOutOfBounds.hpp"

#include <cmath>

#include "tf2/exceptions.h"

namespace attention_system_behaviors
{

IsRobotOutOfBounds::IsRobotOutOfBounds(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("IsRobotOutOfBounds: 'node' not found in blackboard");
  }

  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus
IsRobotOutOfBounds::tick()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  std::string robot_reference_frame;
  double move_back_limit_dist;

  if (!getInput("robot_reference_frame", robot_reference_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'robot_reference_frame' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("move_back_limit_dist", move_back_limit_dist)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'move_back_limit_dist' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (move_back_limit_dist <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'move_back_limit_dist' must be greater than zero.");
    return BT::NodeStatus::FAILURE;
  }

  try {
    const auto transform_stamped = tf_buffer_->lookupTransform(
      initial_reference_frame_,
      robot_reference_frame,
      tf2::TimePointZero);

    const double x = transform_stamped.transform.translation.x;
    const double y = transform_stamped.transform.translation.y;
    const double distance = std::sqrt((x * x) + (y * y));

    if (distance > move_back_limit_dist) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Robot is out of bounds: distance=%f, move_back_limit_dist=%f",
        distance,
        move_back_limit_dist);
      return BT::NodeStatus::SUCCESS;
    }
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Could not get transform from '%s' to '%s': %s",
      initial_reference_frame_,
      robot_reference_frame.c_str(),
      exception.what());
  }

  return BT::NodeStatus::FAILURE;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::IsRobotOutOfBounds>(
    "IsRobotOutOfBounds");
}

#endif
