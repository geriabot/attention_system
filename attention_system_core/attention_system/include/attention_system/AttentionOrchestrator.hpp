#ifndef ATTENTION_SYSTEM__ATTENTION_ORCHESTRATOR_HPP__
#define ATTENTION_SYSTEM__ATTENTION_ORCHESTRATOR_HPP__

#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "attention_system_interfaces/srv/use_attention.hpp"
#include "attention_system_interfaces/srv/ask_for_attention_behavior_input.hpp"
#include "attention_system_interfaces/srv/ask_for_attention_behavior.hpp"
#include "attention_system_interfaces/msg/attention_action_details.hpp"
#include "attention_system_interfaces/msg/attention_system_status.hpp"

#include "attention_system/attention_behaviors_utils.hpp"
#include "attention_system/attention_states.hpp"

#include "behavior_architecture/base_orchestrator.hpp"

#include <unordered_map>
#include <map>
#include <string>
#include <sstream>

namespace attention_system
{

using namespace std::chrono_literals;
using AskForAttentionBehavior = attention_system_interfaces::srv::AskForAttentionBehavior;
using AskForAttentionBehaviorInput = attention_system_interfaces::srv::AskForAttentionBehaviorInput;
using UseAttention = attention_system_interfaces::srv::UseAttention;
using AttentionActionDetails = attention_system_interfaces::msg::AttentionActionDetails;
using AttentionSystemStatus = attention_system_interfaces::msg::AttentionSystemStatus;
using TriggerService = std_srvs::srv::Trigger;

const std::string ATTENTION_ORCHESTRATOR_NAME = "attention_orchestrator";

class AttentionOrchestrator : public behavior_architecture::BaseOrchestrator
{

public:
  AttentionOrchestrator(BT::Blackboard::Ptr blackboard);

protected:
  void control_cycle() override;
  void go_to_state(int state) override;

private:

  // +++++++++ UTILITIES ++++++++++
  void parse_attention_behaviors_parameters();

  // +++++++++ CASCADE LIFECYCLE +++++++++

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  void clear_active_attention_behaviors();
  void reset_operational_state();
  void set_assigned_blackboard_input(
    const std::string & key,
    const std::string & value);
  std::string get_assigned_blackboard_input_as_string(
    const std::string & key,
    const std::string & value);
  void publish_status();
  void behavior_status_callback(std_msgs::msg::String::UniquePtr msg);

  // +++++++++ INTELLIGENCE +++++++++

  const std::chrono::milliseconds ACTION_WAIT_TIME = 1s;
  bool intelligence_result_received_;
  const std::string ATTENTION_FRAME_ID = "attention_point";

  const int MAX_INTELLIGENCE_CONNECTION_ATTEMPTS = 5;
  rclcpp::Client<TriggerService>::SharedPtr start_intelligence_client_;
  rclcpp::Client<TriggerService>::SharedPtr stop_intelligence_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    behavior_status_subscription_;
  bool behavior_returned_failure_;

  // USE ATTENTION

  const std::string USE_ATTENTION_SERVICE_NAME = "use_attention";
  rclcpp::Service<UseAttention>::SharedPtr use_attention_service_;

  void handle_use_attention_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    UseAttention::Request::SharedPtr request);

  void use_attention(
    UseAttention::Request::SharedPtr request);

  void terminate_use_attention(int8_t status, std::string frame_id);
  std::shared_ptr<rmw_request_id_t> use_attention_request_header_;
  UseAttention::Request::SharedPtr use_attention_request_;
  bool use_attention_request_active_;
  bool must_terminate_use_attention_request_;
  int use_attention_result_status_;

  // ASK GEMINI BEHAVIOR ACTION

  rclcpp::Client<AskForAttentionBehavior>::SharedPtr attention_intelligence_at_action_client_;

  int at_action_result_;

  void at_action_intelligence_result_callback(
    rclcpp::Client<AskForAttentionBehavior>::SharedFuture result);
  
  void call_at_action_intelligence_client(
    UseAttention::Request::SharedPtr prompt_information);

  bool waiting_for_at_action_result_;
  bool action_request_sent_;

  // ASK GEMINI BEHAVIOR INPUT

  bool waiting_for_behavior_input_result_;

  // +++++++++++++++++++++++++++++++++++++++++

  rclcpp::TimerBase::SharedPtr timer_;
  const std::chrono::milliseconds CONTROL_PERIOD = 200ms;
 
  AttentionState control_state_;

  std::string frame_id_;

  std::string actual_task_ = "";
  std::string actual_behavior_details_;
  AttentionActuationCapabilities actual_available_actuation_;

  std::map<int, AttentionBehaviorParams> available_attention_behaviors_;
  std::map<int, AttentionActionDetails> attention_behaviors_details_;
  std::vector<std::string> active_attention_behaviors_;

  AttentionActionDetails actual_action_;
  int last_action_id_;
  rclcpp_lifecycle::LifecyclePublisher<AttentionSystemStatus>::SharedPtr
    status_publisher_;
  bool attention_action_activated_;
  bool attention_action_details_published_;
  bool acting_state_initialized_;

  std::vector<std::string> context_details_;
  std::string context_details_str_;
};

} // namespace attention_system

#endif // ATTENTION_SYSTEM__ATTENTION_ORCHESTRATOR_HPP__
