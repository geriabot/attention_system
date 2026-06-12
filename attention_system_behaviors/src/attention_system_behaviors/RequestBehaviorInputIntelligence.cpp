#include "attention_system_behaviors/RequestBehaviorInputIntelligence.hpp"

#include <unordered_set>
#include <sstream>

namespace attention_system_behaviors
{

using namespace std::chrono_literals;

RequestBehaviorInputIntelligence::RequestBehaviorInputIntelligence(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError(
            "RequestBehaviorInputIntelligence: 'node' not found in blackboard");
  }

  node_ = node_any;
  client_ = node_->create_client<AskForAttentionBehaviorInput>(
    "attention/ask_intelligence_for_behavior_input");

  request_id_ = 0;
  active_request_id_ = 0;

  reset_state();
}

BT::NodeStatus
RequestBehaviorInputIntelligence::onStart()
{
  reset_state();

  std::string task_details;
  if (!getInput("task_details", task_details)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Input port 'task_details' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  std::string context_details;
  getInput("context_details", context_details);

  std::string behavior_details;
  if (!getInput("behavior_details", behavior_details)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Input port 'behavior_details' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  std::string action_explanation;
  if (!getInput("action_explanation", action_explanation)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Input port 'action_explanation' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  std::string inputs;
  if (!getInput("inputs", inputs)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'inputs' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  bool needs_img_for_inputs_request = false;
  if (!getInput(
        "needs_img_for_input_request",
        needs_img_for_inputs_request))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'needs_img_for_inputs_request' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  std::string blackboard_inputs;
  if (!getInput("blackboard_inputs", blackboard_inputs)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Input port 'blackboard_inputs' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  std::string blackboard_inputs_to_ask;
  if (!getInput("blackboard_inputs_to_ask", blackboard_inputs_to_ask)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Input port 'blackboard_inputs_ask' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  std::string additional_information;
  getInput("additional_information", additional_information);

  std::vector<std::string> action_inputs = split_string(inputs, ':');
  blackboard_input_keys_ = split_string(blackboard_inputs, ':');
  blackboard_inputs_to_ask_ = split_string(blackboard_inputs_to_ask, ':');
  blackboard_input_indices_to_ask_.clear();
  blackboard_input_indices_to_ask_.reserve(blackboard_inputs_to_ask_.size());

  std::unordered_set<std::string> blackboard_inputs_to_ask_set(
    blackboard_inputs_to_ask_.begin(),
    blackboard_inputs_to_ask_.end());

  for (size_t i = 0; i < blackboard_input_keys_.size(); ++i) {
    if (blackboard_inputs_to_ask_set.find(blackboard_input_keys_[i]) !=
      blackboard_inputs_to_ask_set.end()) {
      blackboard_input_indices_to_ask_.push_back(i);
    }
  }

  if (action_inputs.size() != blackboard_input_keys_.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input ports 'inputs' and 'blackboard_inputs' must have the same size.");
    return BT::NodeStatus::FAILURE;
  }

  if (blackboard_input_indices_to_ask_.size() !=
    blackboard_inputs_to_ask_.size())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Some keys from 'blackboard_inputs_to_ask' were not found in 'blackboard_inputs'.");
    return BT::NodeStatus::FAILURE;
  }

  std::vector<std::string> action_inputs_to_ask;
  action_inputs_to_ask.reserve(blackboard_input_indices_to_ask_.size());

  for (size_t index : blackboard_input_indices_to_ask_) {
    action_inputs_to_ask.push_back(action_inputs[index]);
  }

  AttentionActionDetails attention_action;
  attention_action.action_id = 0;
  attention_action.action_explanation = action_explanation;
  attention_action.inputs = action_inputs_to_ask;
  attention_action.needs_img_for_inputs_request =
    needs_img_for_inputs_request;

  auto request = std::make_shared<AskForAttentionBehaviorInput::Request>();
  request->task_details = task_details;
  request->context_details = context_details;
  request->behavior_details = behavior_details;
  request->attention_action = attention_action;
  request->additional_information = additional_information;

  while (!client_->wait_for_service(service_wait_time_)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[RequestBehaviorInputIntelligence] Service not available, waiting again...");
  }

  RCLCPP_INFO(node_->get_logger(), "Sending request");
  active_request_id_ = ++request_id_;
  const size_t request_id = active_request_id_;
  client_->async_send_request(
    request,
    [this, request_id](
      rclcpp::Client<AskForAttentionBehaviorInput>::SharedFuture result)
    {
      result_callback(result, request_id);
    });
  request_in_progress_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "RequestBehaviorInputIntelligence request sent successfully.");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
RequestBehaviorInputIntelligence::onRunning()
{
  if (should_fail_) {
    return BT::NodeStatus::FAILURE;
  }

  if (!result_received_) {
    return BT::NodeStatus::RUNNING;
  }

  if (!request_succeeded_) {
    return BT::NodeStatus::FAILURE;
  }

  if (result_inputs_.size() != blackboard_input_indices_to_ask_.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Received %zu inputs, but %zu blackboard keys were requested.",
      result_inputs_.size(),
      blackboard_input_indices_to_ask_.size());
    return BT::NodeStatus::FAILURE;
  }

  for (size_t i = 0; i < result_inputs_.size(); i++) {
    const size_t blackboard_index = blackboard_input_indices_to_ask_[i];

    config().blackboard->set<std::string>(
      blackboard_input_keys_[blackboard_index],
      result_inputs_[i]);
    RCLCPP_INFO(
      node_->get_logger(),
      "Set in blackboard: %s -> %s",
      blackboard_input_keys_[blackboard_index].c_str(),
      result_inputs_[i].c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

void
RequestBehaviorInputIntelligence::onHalted()
{
  if (request_in_progress_) {
    request_id_++;
    RCLCPP_INFO(
      node_->get_logger(),
      "RequestBehaviorInputIntelligence request ignored due to halt.");
  }

  reset_state();
}

std::vector<std::string>
RequestBehaviorInputIntelligence::split_string(
  const std::string & value,
  char delimiter) const
{
  std::vector<std::string> elements;
  std::stringstream ss(value);
  std::string token;

  while (std::getline(ss, token, delimiter)) {
    elements.push_back(token);
  }

  return elements;
}

void
RequestBehaviorInputIntelligence::reset_state()
{
  request_in_progress_ = false;
  result_received_ = false;
  request_succeeded_ = false;
  should_fail_ = false;
  result_inputs_.clear();
  blackboard_input_keys_.clear();
  blackboard_inputs_to_ask_.clear();
  blackboard_input_indices_to_ask_.clear();
  active_request_id_ = 0;
}

void
RequestBehaviorInputIntelligence::result_callback(
  rclcpp::Client<AskForAttentionBehaviorInput>::SharedFuture result,
  size_t request_id)
{
  if (request_id != active_request_id_ || !request_in_progress_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring stale RequestBehaviorInputIntelligence response.");
    return;
  }

  request_in_progress_ = false;
  result_received_ = true;

  auto response = result.get();
  if (response->success) {
    request_succeeded_ = true;
    result_inputs_ = response->inputs;
    RCLCPP_INFO(
      node_->get_logger(),
      "RequestBehaviorInputIntelligence service succeeded.");
  } else {
    should_fail_ = true;
    RCLCPP_ERROR(
      node_->get_logger(),
      "RequestBehaviorInputIntelligence service failed: %s",
      response->message.c_str());
  }
}

}  // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::RequestBehaviorInputIntelligence>(
    "RequestBehaviorInputIntelligence");
}

#endif
