#ifndef ATTENTION_SYSTEM_BEHAVIORS__CLEAR_VISUAL_DETECTION_CLASS_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__CLEAR_VISUAL_DETECTION_CLASS_HPP

#include <string>

#include "behaviortree_cpp/action_node.h"

#include "omdet_node_msgs/srv/set_detection_prompt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"

namespace attention_system_behaviors
{

class ClearVisualDetectionClass : public BT::StatefulActionNode
{
using SetDetectionPrompt = omdet_node_msgs::srv::SetDetectionPrompt;

public:
  explicit ClearVisualDetectionClass(
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
  void send_clear_request();
  void handle_response(rclcpp::Client<SetDetectionPrompt>::SharedFuture future);

  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Client<SetDetectionPrompt>::SharedPtr client_;
  bool request_done_ {false};
  bool when_halted_ {false};
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__CLEAR_VISUAL_DETECTION_CLASS_HPP
