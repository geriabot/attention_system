#ifndef ATTENTION_SYSTEM__ATTENTION_INTELLIGENCE_HPP__
#define ATTENTION_SYSTEM__ATTENTION_INTELLIGENCE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "attention_system_interfaces/srv/ask_for_attention_behavior_input.hpp"
#include "attention_system_interfaces/srv/ask_for_attention_behavior.hpp"

#include "attention_system/attention_behaviors_utils.hpp"
#include "attention_system/intelligence/attention_intelligence_states.hpp"

#include "llm_router_msgs/srv/get_llm_response.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "type_description_interfaces/msg/key_value.hpp"

#include "nlohmann/json.hpp"

#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace attention_system
{

using namespace std::chrono_literals;

using json = nlohmann::json;

using AskForAttentionBehavior = attention_system_interfaces::srv::AskForAttentionBehavior;
using AskForAttentionBehaviorInput = attention_system_interfaces::srv::AskForAttentionBehaviorInput;
using AttentionActionDetails = attention_system_interfaces::msg::AttentionActionDetails;
using GetLLMResponse = llm_router_msgs::srv::GetLLMResponse;
using KeyValue = type_description_interfaces::msg::KeyValue;
using TriggerService = std_srvs::srv::Trigger;

class AttentionIntelligence : public rclcpp::Node
{
public:
  AttentionIntelligence();

  void control_cycle();

private:

  enum class LogRequestType
  {
    ACTION,
    INPUT
  };

  struct LogContext
  {
    bool is_active = false;
    LogRequestType request_type = LogRequestType::ACTION;
    std::string prompt;
    std::string output_format_json;
    std::filesystem::path file_path;
  };

  rclcpp::TimerBase::SharedPtr timer_;
  const std::chrono::milliseconds CONTROL_PERIOD = 500ms;
  IntelligenceState control_state_;

  void on_activate(
    const std::shared_ptr<TriggerService::Request> request,
    std::shared_ptr<TriggerService::Response> response);
  void on_deactivate(
    const std::shared_ptr<TriggerService::Request> request,
    std::shared_ptr<TriggerService::Response> response);

  rclcpp::Service<TriggerService>::SharedPtr start_intelligence_service_;
  rclcpp::Service<TriggerService>::SharedPtr stop_intelligence_service_;

  rclcpp::Client<GetLLMResponse>::SharedPtr llm_router_;
  const std::chrono::milliseconds WAIT_LLM_SERVICE = 500ms;
  void llm_callback_response(
    rclcpp::Client<GetLLMResponse>::SharedFutureWithRequest response);
  void prepare_log_context(
    LogRequestType request_type,
    const std::string & prompt,
    const std::string & output_format_json);
  void write_response_log(const std::string & response);
  std::filesystem::path ensure_log_directory();
  std::filesystem::path build_log_file_path(LogRequestType request_type);
  std::string generate_log_timestamp() const;
  std::string get_log_type_label(LogRequestType request_type) const;
  std::string build_log_content(
    LogRequestType request_type,
    const std::string & prompt,
    const std::string & output_format_json,
    const std::string & response) const;
  std::string build_output_format_json(
    const std::vector<KeyValue> & output_fields) const;
  json build_output_field_json(const std::string & field_type) const;
  std::string serialize_llm_outputs(
    const std::vector<KeyValue> & outputs) const;
  bool find_llm_output_value(
    const std::vector<KeyValue> & outputs,
    const std::string & key,
    std::string & value) const;

  std::vector<KeyValue> llm_response_outputs_;
  LogContext log_context_;




  rclcpp::Service<AskForAttentionBehavior>::SharedPtr
    llm_at_action_service_;

  void handle_at_action_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    AskForAttentionBehavior::Request::SharedPtr request);
  
  void ask_llm_at_action();

  void respond_at_action(
    bool success,
    const std::string & message,
    int8_t attention_action_id);

  int parse_llm_at_action_response(
    const std::vector<KeyValue> & response_outputs);

  std::shared_ptr<rmw_request_id_t> at_action_request_header_;
  AskForAttentionBehavior::Request::SharedPtr at_action_request_;
  bool at_action_request_active_;

  rclcpp::Service<AskForAttentionBehaviorInput>::SharedPtr
    llm_input_service_;
  
  void handle_input_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    AskForAttentionBehaviorInput::Request::SharedPtr request);
  
  bool ask_llm_input();

  void respond_input(
    bool success,
    const std::string & message,
    const std::vector<std::string> & inputs);

  bool parse_llm_input_response(
    const std::vector<std::string> & requested_inputs,
    const std::vector<KeyValue> & response_outputs,
    std::vector<std::string> & parsed_inputs) const;
  bool parse_input_field_definition(
    const std::string & input_definition,
    std::string & field_type,
    std::string & field_name) const;
  bool map_input_field_type(
    const std::string & field_type,
    std::string & output_field_type) const;
  bool build_input_output_fields(
    const std::vector<std::string> & requested_inputs,
    std::vector<KeyValue> & output_fields) const;

  std::shared_ptr<rmw_request_id_t> input_request_header_;
  AskForAttentionBehaviorInput::Request::SharedPtr input_request_;
  bool input_request_active_;
  
  bool include_image_;
  std::unique_ptr<sensor_msgs::msg::Image> prompt_img_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  void image_callback(sensor_msgs::msg::Image::UniquePtr image);

  bool llm_response_received_;
  bool successful_response_;
  bool waiting_for_llm_response_;

  const std::string NEW_LINE = R"(
)";

  const std::string TASK_DETAILS_PREFIX = "You control the attention system of a robot whose task is: ";
  const std::string CONTEXT_DETAILS_PREFIX = "Additional information you should know: ";
  std::string context_details_;

  const std::string BEHAVIOR_DETAILS_PREFIX = "During the task, the robot will perform this behavior:";
  const std::string ACTIONS_PREFIX = "The robot's attention system provides the following actions:";
  const std::string INPUT_REQ_ACTIONS_PREFIX = "The robot's attention system will perform following action:";

  std::string generate_behavior_prompt(
    const std::string & behavior_details);

  std::string generate_action_prompt(
    const AttentionActionDetails & action,
    bool include_inputs);
};

} // namespace attention_system

#endif // ATTENTION_SYSTEM__ATTENTION_INTELLIGENCE_HPP__
