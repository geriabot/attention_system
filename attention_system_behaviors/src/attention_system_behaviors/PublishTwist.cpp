#include "attention_system_behaviors/PublishTwist.hpp"

namespace attention_system_behaviors
{

PublishTwist::PublishTwist(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("PublishTwist: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
}

BT::NodeStatus
PublishTwist::onStart()
{
  if (!getInput("publish_twist_topic", topic_)) {
    throw BT::RuntimeError("PublishTwist: input port 'publish_twist_topic' missing in XML.");
  }

  if (topic_.empty()) {
    throw BT::RuntimeError("PublishTwist: input port 'publish_twist_topic' cannot be empty");
  }

  twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    topic_,
    10);

  publish_tree_tick(tree_tick_pub_, this->name());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
PublishTwist::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  double lin_x;
  double lin_y;
  double lin_z;
  double ang_x;
  double ang_y;
  double ang_z;

  if (!getInput("lin_x", lin_x)) {
    lin_x = 0.0;
  }

  if (!getInput("lin_y", lin_y)) {
    lin_y = 0.0;
  }

  if (!getInput("lin_z", lin_z)) {
    lin_z = 0.0;
  }

  if (!getInput("ang_x", ang_x)) {
    ang_x = 0.0;
  }

  if (!getInput("ang_y", ang_y)) {
    ang_y = 0.0;
  }

  if (!getInput("ang_z", ang_z)) {
    ang_z = 0.0;
  }

  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = lin_x;
  twist_msg.linear.y = lin_y;
  twist_msg.linear.z = lin_z;
  twist_msg.angular.x = ang_x;
  twist_msg.angular.y = ang_y;
  twist_msg.angular.z = ang_z;

  twist_pub_->publish(twist_msg);

  return BT::NodeStatus::RUNNING;
}

void
PublishTwist::onHalted()
{
  if (twist_pub_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist twist_msg;
  twist_pub_->publish(twist_msg);

  twist_pub_ = nullptr;
}

}  // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::PublishTwist>("PublishTwist");
}

#endif
