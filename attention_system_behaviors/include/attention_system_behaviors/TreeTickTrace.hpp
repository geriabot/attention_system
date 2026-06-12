#ifndef ATTENTION_SYSTEM_BEHAVIORS__TREE_TICK_TRACE_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__TREE_TICK_TRACE_HPP

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace attention_system_behaviors
{

using TreeTickPublisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;

inline TreeTickPublisher
create_tree_tick_publisher(
  const rclcpp::Node::SharedPtr & node)
{
  const std::string topic_parameter = "tree_tick_topic";
  const std::string default_topic = "/test_traces/tree_tick";

  if (!node->has_parameter(topic_parameter)) {
    node->declare_parameter(topic_parameter, default_topic);
  }

  std::string topic;
  node->get_parameter(topic_parameter, topic);

  if (topic.empty()) {
    throw BT::RuntimeError("Parameter '" + topic_parameter + "' cannot be empty");
  }

  return node->create_publisher<std_msgs::msg::String>(
    topic,
    10);
}

inline void
publish_tree_tick(
  const TreeTickPublisher & publisher,
  const std::string & node_name)
{
  if (publisher == nullptr) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = node_name;
  publisher->publish(msg);
}

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__TREE_TICK_TRACE_HPP
