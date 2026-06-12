#ifndef ATTENTION_SYSTEM_BEHAVIORS__ACTIVATE_TWISTING_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__ACTIVATE_TWISTING_HPP

#include "behaviortree_cpp/behavior_tree.h"

#include "attention_actuation_msgs/srv/start_twisting.hpp"
#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace attention_system_behaviors
{

class ActivateTwisting : public BT::StatefulActionNode
{
public:
  explicit ActivateTwisting(
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
        BT::InputPort<std::string>("topic")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;

  rclcpp::Client<attention_actuation_msgs::srv::StartTwisting>::SharedPtr client_;
  std::shared_future<attention_actuation_msgs::srv::StartTwisting::Response::SharedPtr> response_future_;
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__ACTIVATE_TWISTING_HPP
