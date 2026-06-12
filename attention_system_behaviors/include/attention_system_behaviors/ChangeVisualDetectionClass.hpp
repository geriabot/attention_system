#ifndef ATTENTION_SYSTEM_BEHAVIORS__CHANGE_VISUAL_DETECTION_CLASS_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__CHANGE_VISUAL_DETECTION_CLASS_HPP

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"

#include "omdet_node_msgs/srv/set_detection_prompt.hpp"

namespace attention_system_behaviors
{

class ChangeVisualDetectionClass : public BT::StatefulActionNode
{

using SetDetectionPrompt = omdet_node_msgs::srv::SetDetectionPrompt;

public:
  explicit ChangeVisualDetectionClass(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf
  );


  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {};

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("class"),
        BT::InputPort<std::string>("service_name")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;

  rclcpp::Client<SetDetectionPrompt>::SharedPtr prompt_cli_;
  std::shared_future<SetDetectionPrompt::Response::SharedPtr> response_future_;
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__CHANGE_VISUAL_DETECTION_CLASS_HPP
