#ifndef ATTENTION_SYSTEM_BEHAVIORS__REQUEST_BEHAVIOR_INPUT_INTELLIGENCE_HPP_
#define ATTENTION_SYSTEM_BEHAVIORS__REQUEST_BEHAVIOR_INPUT_INTELLIGENCE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

#include "attention_system_interfaces/srv/ask_for_attention_behavior_input.hpp"
#include "attention_system_interfaces/msg/attention_action_details.hpp"

namespace attention_system_behaviors
{

class RequestBehaviorInputIntelligence : public BT::StatefulActionNode
{
public:
  using AskForAttentionBehaviorInput =
    attention_system_interfaces::srv::AskForAttentionBehaviorInput;
  using AttentionActionDetails =
    attention_system_interfaces::msg::AttentionActionDetails;

  explicit RequestBehaviorInputIntelligence(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  static BT::PortsList
  providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("task_details"),
        BT::InputPort<std::string>("context_details", ""),
        BT::InputPort<std::string>("behavior_details"),
        BT::InputPort<std::string>("action_explanation"),
        BT::InputPort<std::string>("inputs"),
        BT::InputPort<bool>("needs_img_for_input_request"),
        BT::InputPort<std::string>("blackboard_inputs"),
        BT::InputPort<std::string>("blackboard_inputs_to_ask"),
        BT::InputPort<std::string>("additional_information", "")
      });
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::vector<std::string>
  split_string(
    const std::string & value,
    char delimiter) const;

  void
  reset_state();

  void
  result_callback(
    rclcpp::Client<AskForAttentionBehaviorInput>::SharedFuture result,
    size_t request_id);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<AskForAttentionBehaviorInput>::SharedPtr client_;

  bool request_in_progress_;
  bool result_received_;
  bool request_succeeded_;
  bool should_fail_;
  size_t request_id_;
  size_t active_request_id_;

  std::vector<std::string> result_inputs_;
  std::vector<std::string> blackboard_input_keys_;
  std::vector<std::string> blackboard_inputs_to_ask_;
  std::vector<size_t> blackboard_input_indices_to_ask_;

  const std::chrono::milliseconds service_wait_time_ {1000};
};

}  // namespace attention_system_behaviors

#endif  // ATTENTION_SYSTEM_BEHAVIORS__REQUEST_BEHAVIOR_INPUT_INTELLIGENCE_HPP_
