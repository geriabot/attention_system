#ifndef ATTENTION_SYSTEM_BEHAVIORS__PUBLISH_TWIST_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__PUBLISH_TWIST_HPP

#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"

namespace attention_system_behaviors
{

class PublishTwist : public BT::StatefulActionNode
{
public:
  explicit PublishTwist(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<double>("lin_x"),
        BT::InputPort<double>("lin_y"),
        BT::InputPort<double>("lin_z"),
        BT::InputPort<double>("ang_x"),
        BT::InputPort<double>("ang_y"),
        BT::InputPort<double>("ang_z"),
        BT::InputPort<std::string>("publish_twist_topic")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  std::string topic_;
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__PUBLISH_TWIST_HPP
