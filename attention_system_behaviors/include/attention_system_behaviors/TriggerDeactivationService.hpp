#ifndef ATTENTION_SYSTEM_BEHAVIORS__TRIGGER_DEACTIVATION_HPP_
#define ATTENTION_SYSTEM_BEHAVIORS__TRIGGER_DEACTIVATION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace attention_system_behaviors
{

class TriggerDeactivationService : public BT::StatefulActionNode
{

public:
  explicit TriggerDeactivationService(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<bool>("when_halted"),
        BT::InputPort<std::string>("service_name")
      }
    );
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void sendDeactivationRequest();
  void handleResponse(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  bool request_done_ {false};
  bool request_success_ {false};
  bool when_halted_ {false};

};

} // namespace attention_system_behaviors

#endif  // ATTENTION_SYSTEM_BEHAVIORS__TRIGGER_DEACTIVATION_HPP_
