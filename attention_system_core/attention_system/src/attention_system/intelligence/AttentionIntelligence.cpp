#include "attention_system/intelligence/AttentionIntelligence.hpp"

#include <ctime>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <sstream>

namespace attention_system
{

using std::placeholders::_1;
using std::placeholders::_2;

AttentionIntelligence::AttentionIntelligence()
: rclcpp::Node("attention_intelligence")
{
  timer_ = nullptr;
  llm_router_ = nullptr;
  llm_at_action_service_ = nullptr;
  llm_input_service_ = nullptr;
  at_action_request_header_ = nullptr;
  at_action_request_ = nullptr;
  at_action_request_active_ = false;
  input_request_header_ = nullptr;
  input_request_ = nullptr;
  input_request_active_ = false;
  image_sub_ = nullptr;
  include_image_ = false;
  llm_response_received_ = false;
  successful_response_ = false;
  waiting_for_llm_response_ = false;
  control_state_ = IntelligenceState::IDLE;

  start_intelligence_service_ = this->create_service<TriggerService>(
    "/attention/start_intelligence",
    std::bind(&AttentionIntelligence::on_activate, this, _1, _2));

  stop_intelligence_service_ = this->create_service<TriggerService>(
    "/attention/stop_intelligence",
    std::bind(&AttentionIntelligence::on_deactivate, this, _1, _2));
}

void
AttentionIntelligence::on_activate(
  const std::shared_ptr<TriggerService::Request> request,
  std::shared_ptr<TriggerService::Response> response)
{
  (void) request;

  llm_response_received_ = false;
  waiting_for_llm_response_ = false;
  successful_response_ = false;
  control_state_ = IntelligenceState::IDLE;
  log_context_ = LogContext();
  llm_response_outputs_.clear();

  RCLCPP_INFO(get_logger(), "<on_activate> Activating intelligence");

  // INIT LLM CLIENT
  llm_router_ = create_client<GetLLMResponse>("/ask_llm");

  RCLCPP_INFO(get_logger(), "<on_activate> LLM client created");

  // INIT ASK ACTION SERVICE
  llm_at_action_service_ =
    this->create_service<AskForAttentionBehavior>(
      "attention/ask_intelligence_for_behavior",
      std::bind(&AttentionIntelligence::handle_at_action_request, this, _1, _2));

  RCLCPP_INFO(get_logger(), "<on_activate> Attention action service created");

  // INIT ASK INPUT SERVICE
  llm_input_service_ =
    this->create_service<AskForAttentionBehaviorInput>(
      "attention/ask_intelligence_for_behavior_input",
      std::bind(&AttentionIntelligence::handle_input_request, this, _1, _2));

  RCLCPP_INFO(get_logger(), "<on_activate> Input service created");

  // INIT CONTROL CYCLE
  timer_ = this->create_wall_timer(
    CONTROL_PERIOD, std::bind(&AttentionIntelligence::control_cycle, this));

  response->success = true;
  response->message = "Attention intelligence activated";

  RCLCPP_INFO(get_logger(), "ATTENTION INTELLIGENCE SUCCESSFULLY ACTIVATED");
}

void
AttentionIntelligence::on_deactivate(
  const std::shared_ptr<TriggerService::Request> request,
  std::shared_ptr<TriggerService::Response> response)
{
  (void) request;

  if (at_action_request_active_) {
    respond_at_action(false, "Attention intelligence deactivated", 0);
  }
  if (input_request_active_) {
    respond_input(false, "Attention intelligence deactivated", {});
  }

  timer_ = nullptr;
  llm_router_ = nullptr;
  llm_at_action_service_ = nullptr;
  llm_input_service_ = nullptr;
  at_action_request_header_ = nullptr;
  at_action_request_ = nullptr;
  at_action_request_active_ = false;
  input_request_header_ = nullptr;
  input_request_ = nullptr;
  input_request_active_ = false;
  image_sub_ = nullptr;
  prompt_img_ = nullptr;
  llm_response_received_ = false;
  successful_response_ = false;
  waiting_for_llm_response_ = false;
  control_state_ = IntelligenceState::IDLE;
  log_context_ = LogContext();
  llm_response_outputs_.clear();

  response->success = true;
  response->message = "Attention intelligence deactivated";
  
  RCLCPP_INFO(get_logger(), "ATTENTION INTELLIGENCE SUCCESSFULLY DEACTIVATED");
}

void
AttentionIntelligence::control_cycle()
{
  switch (control_state_) {
    case IntelligenceState::IDLE:
      break;
    
    case IntelligenceState::ASK_BEHAVIOR_ACTION:
      if (llm_response_received_) {
        RCLCPP_INFO(get_logger(), "Parsing action");
        
        if (successful_response_) {
          try {
            const int action_id =
              parse_llm_at_action_response(llm_response_outputs_);
            respond_at_action(true, "Attention action selected", action_id);
          } catch (const std::exception & error) {
            RCLCPP_ERROR(get_logger(), "Failed to parse action LLM response: %s", error.what());
            respond_at_action(false, error.what(), 0);
          }
        } else {
          respond_at_action(false, "LLM response was not valid", 0);
        }

        llm_response_received_ = false;
        successful_response_ = false;
        waiting_for_llm_response_ = false;
        llm_response_outputs_.clear();

        control_state_ = IntelligenceState::IDLE;

      } else if (!waiting_for_llm_response_) {
        ask_llm_at_action();
        waiting_for_llm_response_ = true;
      }

      break;

    case IntelligenceState::ASK_BEHAVIOR_INPUTS:
      if (llm_response_received_) {

        // TODO Esto está mal hecho, no tiene sentido pedir el id de una detección si 
        // todavía no se ha pedido cambiar el prompt

        RCLCPP_INFO(get_logger(), "Parsing input");
        
        if (successful_response_) {
          std::vector<std::string> parsed_inputs;
          if (parse_llm_input_response(
              input_request_->attention_action.inputs,
              llm_response_outputs_,
              parsed_inputs))
          {
            respond_input(true, "Attention inputs selected", parsed_inputs);
          } else {
            respond_input(false, "Failed to parse LLM input response", {});
          }
        } else {
          respond_input(false, "LLM response was not valid", {});
        }

        llm_response_received_ = false;
        waiting_for_llm_response_ = false;
        successful_response_ = false;
        llm_response_outputs_.clear();

        control_state_ = IntelligenceState::IDLE;

      } else {
        if (!waiting_for_llm_response_) {
          waiting_for_llm_response_ = ask_llm_input();
        }
      }
      break;
  }

  /*
  RCLCPP_INFO(this->get_logger(), "Control state: %d, response received: %d, waiting: %d",
              (int) control_state_,
              llm_response_received_,
              waiting_for_llm_response_);
  */
}

void
AttentionIntelligence::llm_callback_response(
  rclcpp::Client<GetLLMResponse>::SharedFutureWithRequest response)
{
  RCLCPP_INFO(get_logger(), "LLM RESPONSE RECEIVED");

  llm_response_outputs_ = (response.get().second)->outputs;
  write_response_log(serialize_llm_outputs(llm_response_outputs_));

  if (llm_response_outputs_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Empty LLM response");
    llm_response_received_ = true;
    successful_response_ = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "LLM RESULTS RECEIVED");
  successful_response_ = true;
  llm_response_received_ = true;
}

void
AttentionIntelligence::handle_at_action_request(
  std::shared_ptr<rmw_request_id_t> request_header,
  AskForAttentionBehavior::Request::SharedPtr request)
{
  if (at_action_request_active_ || input_request_active_) {
    AskForAttentionBehavior::Response response;
    response.success = false;
    response.message = "Attention intelligence is already processing a request";
    response.attention_action_id = 0;
    llm_at_action_service_->send_response(*request_header, response);
    RCLCPP_WARN(get_logger(), "Attention action request rejected");
    return;
  }

  RCLCPP_INFO(get_logger(), "Attention action request accepted");

  control_state_ = IntelligenceState::ASK_BEHAVIOR_ACTION;
  at_action_request_header_ = request_header;
  at_action_request_ = request;
  at_action_request_active_ = true;
  llm_response_received_ = false;
  successful_response_ = false;
  waiting_for_llm_response_ = false;
}

void
AttentionIntelligence::ask_llm_at_action()
{
  RCLCPP_INFO(get_logger(), "ATTENTION ACTION");

  if (!at_action_request_) {
    respond_at_action(false, "No active attention action request", 0);
    return;
  }

  context_details_ = at_action_request_->context_details;

  std::string prompt = TASK_DETAILS_PREFIX + at_action_request_->task_details;

  prompt += NEW_LINE + NEW_LINE + BEHAVIOR_DETAILS_PREFIX + NEW_LINE +
    generate_behavior_prompt(at_action_request_->behavior_details);

  prompt += NEW_LINE + ACTIONS_PREFIX + NEW_LINE;

  for (const auto & action : at_action_request_->attention_actions) {
    prompt += generate_action_prompt(action, false);
  }

  prompt += NEW_LINE + std::string("You must select which action the robot's attention system should perform at the described robot behavior.");

  if (context_details_ != "") {
    prompt += NEW_LINE + NEW_LINE + CONTEXT_DETAILS_PREFIX + context_details_;
  }

  RCLCPP_INFO(get_logger(), "PROMPT\n---\n%s\n---", prompt.c_str());

  auto request = std::make_shared<GetLLMResponse::Request>();

  request->prompt = prompt;
  request->uses_image = false;
  request->output_fields.resize(1);
  //request->output_fields[0].key = "chain_of_thoughts";
  //request->output_fields[0].value = GetLLMResponse::Request::STRING;
  request->output_fields[0].key = "attention_action_id";
  request->output_fields[0].value = GetLLMResponse::Request::INT;
  prepare_log_context(
    LogRequestType::ACTION,
    prompt,
    build_output_format_json(request->output_fields));

  while(!llm_router_->wait_for_service(WAIT_LLM_SERVICE)) {
    RCLCPP_INFO(get_logger(), "waiting for /ask_llm service for being available...");
  }

  llm_router_->async_send_request(
    request, std::bind(&AttentionIntelligence::llm_callback_response, this, _1));
}

void
AttentionIntelligence::respond_at_action(
  bool success,
  const std::string & message,
  int8_t attention_action_id)
{
  if (!at_action_request_active_ || !at_action_request_header_) {
    return;
  }

  AskForAttentionBehavior::Response response;
  response.success = success;
  response.message = message;
  response.attention_action_id = attention_action_id;

  llm_at_action_service_->send_response(*at_action_request_header_, response);
  at_action_request_header_ = nullptr;
  at_action_request_ = nullptr;
  at_action_request_active_ = false;
}

int
AttentionIntelligence::parse_llm_at_action_response(
  const std::vector<KeyValue> & response_outputs)
{
  std::string output_value;

  if (!find_llm_output_value(response_outputs, "attention_action_id", output_value)) {
    throw std::runtime_error("Missing 'attention_action_id' in LLM response");
  }

  return std::stoi(output_value);
}

void
AttentionIntelligence::handle_input_request(
  std::shared_ptr<rmw_request_id_t> request_header,
  AskForAttentionBehaviorInput::Request::SharedPtr request)
{
  if (at_action_request_active_ || input_request_active_) {
    AskForAttentionBehaviorInput::Response response;
    response.success = false;
    response.message = "Attention intelligence is already processing a request";
    llm_input_service_->send_response(*request_header, response);
    RCLCPP_WARN(get_logger(), "Attention input request rejected");
    return;
  }

  RCLCPP_INFO(get_logger(), "Attention input request accepted");

  input_request_header_ = request_header;
  input_request_ = request;
  input_request_active_ = true;

  include_image_ = request->attention_action.needs_img_for_inputs_request;

  if (include_image_) {
    prompt_img_ = nullptr;

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                  "/image_rgb", 10, std::bind(
                                    &AttentionIntelligence::image_callback,
                                    this,
                                    std::placeholders::_1));
  }

  llm_response_received_ = false;
  successful_response_ = false;
  waiting_for_llm_response_ = false;
  control_state_ = IntelligenceState::ASK_BEHAVIOR_INPUTS;
}

void
AttentionIntelligence::image_callback(sensor_msgs::msg::Image::UniquePtr image)
{
  image_sub_ = nullptr;
  prompt_img_ = std::move(image);

  RCLCPP_INFO(this->get_logger(), "Image received!!!");
}

bool
AttentionIntelligence::ask_llm_input()
{
  RCLCPP_INFO(get_logger(), "ATTENTION INPUT");

  if (!input_request_) {
    respond_input(false, "No active attention input request", {});
    return false;
  }

  std::string prompt = TASK_DETAILS_PREFIX + input_request_->task_details;

  prompt += NEW_LINE + NEW_LINE + BEHAVIOR_DETAILS_PREFIX + NEW_LINE +
    generate_behavior_prompt(input_request_->behavior_details);

  auto action_prompt = generate_action_prompt(input_request_->attention_action, true);

  if (action_prompt == "") {
    RCLCPP_ERROR(get_logger(), "Inputs are not valid.");
    return false;
  }

  prompt += NEW_LINE +
            INPUT_REQ_ACTIONS_PREFIX +
            NEW_LINE +
            action_prompt;

  prompt += NEW_LINE + std::string("You must establish the values for the inputs needed by the action which the robot is performing. (Only give the value for each field, not also the name of the field)") + 
    std::string("Its obligatory have to indicate the value for all the inputs.");

  if (input_request_->additional_information != "") {
    prompt += NEW_LINE + NEW_LINE +
              std::string("You should take this information in account.") +
              NEW_LINE +
              input_request_->additional_information;
  }

  if (input_request_->context_details != "") {
    prompt += NEW_LINE + NEW_LINE + CONTEXT_DETAILS_PREFIX +
      input_request_->context_details;
  }

  RCLCPP_INFO(get_logger(), "PROMPT\n---\n%s\n---", prompt.c_str());

  auto request = std::make_shared<GetLLMResponse::Request>();

  request->prompt = prompt;
  request->uses_image = include_image_;
  if (!build_input_output_fields(
      input_request_->attention_action.inputs,
      request->output_fields))
  {
    llm_response_received_ = true;
    successful_response_ = false;
    return false;
  }

  prepare_log_context(
    LogRequestType::INPUT,
    prompt,
    build_output_format_json(request->output_fields));

  if (include_image_) {
  if (!prompt_img_) {
    RCLCPP_ERROR(this->get_logger(), "Image not received!");
    return false;
  }

    RCLCPP_INFO(this->get_logger(), "Including image...");
    request->image = *prompt_img_;
  }

  while(!llm_router_->wait_for_service(WAIT_LLM_SERVICE)) {
    RCLCPP_INFO(get_logger(), "waiting for /ask_llm service for being available...");
  }

  llm_router_->async_send_request(
    request, std::bind(&AttentionIntelligence::llm_callback_response, this, _1));

  return true;
}

void
AttentionIntelligence::respond_input(
  bool success,
  const std::string & message,
  const std::vector<std::string> & inputs)
{
  if (!input_request_active_ || !input_request_header_) {
    return;
  }

  AskForAttentionBehaviorInput::Response response;
  response.success = success;
  response.message = message;
  response.inputs = inputs;

  llm_input_service_->send_response(*input_request_header_, response);
  input_request_header_ = nullptr;
  input_request_ = nullptr;
  input_request_active_ = false;
}

bool
AttentionIntelligence::parse_llm_input_response(
  const std::vector<std::string> & requested_inputs,
  const std::vector<KeyValue> & response_outputs,
  std::vector<std::string> & parsed_inputs) const
{
  parsed_inputs.clear();
  parsed_inputs.reserve(requested_inputs.size());

  for (const auto & requested_input : requested_inputs) {
    std::string field_type;
    std::string field_name;
    if (!parse_input_field_definition(requested_input, field_type, field_name)) {
      return false;
    }

    std::string output_value;
    if (!find_llm_output_value(response_outputs, field_name, output_value)) {
      RCLCPP_ERROR(
        get_logger(),
        "Missing '%s' in LLM input response",
        field_name.c_str());
      return false;
    }

    parsed_inputs.push_back(output_value);
  }

  return true;
}

bool
AttentionIntelligence::parse_input_field_definition(
  const std::string & input_definition,
  std::string & field_type,
  std::string & field_name) const
{
  if (input_definition.empty() || input_definition[0] != '<') {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid input definition '%s'. Expected format '<type>name'",
      input_definition.c_str());
    return false;
  }

  const size_t type_end_index = input_definition.find('>');
  if (type_end_index == std::string::npos || type_end_index <= 1U) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid input definition '%s'. Expected format '<type>name'",
      input_definition.c_str());
    return false;
  }

  field_type = input_definition.substr(1, type_end_index - 1);
  field_name = input_definition.substr(type_end_index + 1);

  if (field_name.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid input definition '%s'. Field name cannot be empty",
      input_definition.c_str());
    return false;
  }

  return true;
}

bool
AttentionIntelligence::map_input_field_type(
  const std::string & field_type,
  std::string & output_field_type) const
{
  if (field_type == "int") {
    output_field_type = GetLLMResponse::Request::INT;
    return true;
  }

  if (field_type == "string") {
    output_field_type = GetLLMResponse::Request::STRING;
    return true;
  }

  if (field_type == "float") {
    output_field_type = GetLLMResponse::Request::FLOAT;
    return true;
  }

  if (field_type == "bool") {
    output_field_type = GetLLMResponse::Request::BOOL;
    return true;
  }

  if (field_type == "int[]") {
    output_field_type = GetLLMResponse::Request::INT_ARRAY;
    return true;
  }

  if (field_type == "string[]") {
    output_field_type = GetLLMResponse::Request::STRING_ARRAY;
    return true;
  }

  if (field_type == "float[]") {
    output_field_type = GetLLMResponse::Request::FLOAT_ARRAY;
    return true;
  }

  if (field_type == "bool[]") {
    output_field_type = GetLLMResponse::Request::BOOL_ARRAY;
    return true;
  }

  RCLCPP_ERROR(
    get_logger(),
    "Unsupported input field type '%s'",
    field_type.c_str());
  return false;
}

bool
AttentionIntelligence::build_input_output_fields(
  const std::vector<std::string> & requested_inputs,
  std::vector<KeyValue> & output_fields) const
{
  output_fields.clear();
  output_fields.reserve(requested_inputs.size());

  for (const auto & requested_input : requested_inputs) {
    std::string field_type;
    std::string field_name;
    if (!parse_input_field_definition(requested_input, field_type, field_name)) {
      return false;
    }

    std::string output_field_type;
    if (!map_input_field_type(field_type, output_field_type)) {
      return false;
    }

    KeyValue output_field;
    output_field.key = field_name;
    output_field.value = output_field_type;
    output_fields.push_back(output_field);
  }

  return true;
}

std::string
AttentionIntelligence::serialize_llm_outputs(
  const std::vector<KeyValue> & outputs) const
{
  std::ostringstream output_stream;

  for (size_t index = 0; index < outputs.size(); ++index) {
    output_stream << outputs[index].key << "=" << outputs[index].value;
    if (index + 1 < outputs.size()) {
      output_stream << NEW_LINE;
    }
  }

  return output_stream.str();
}

bool
AttentionIntelligence::find_llm_output_value(
  const std::vector<KeyValue> & outputs,
  const std::string & key,
  std::string & value) const
{
  for (const auto & output : outputs) {
    if (output.key == key) {
      value = output.value;
      return true;
    }
  }

  return false;
}

void
AttentionIntelligence::prepare_log_context(
  LogRequestType request_type,
  const std::string & prompt,
  const std::string & output_format_json)
{
  log_context_.is_active = true;
  log_context_.request_type = request_type;
  log_context_.prompt = prompt;
  log_context_.output_format_json = output_format_json;
  log_context_.file_path = build_log_file_path(request_type);
}

void
AttentionIntelligence::write_response_log(const std::string & response)
{
  if (!log_context_.is_active) {
    RCLCPP_WARN(get_logger(), "Skipping LLM response log because no log context is active");
    return;
  }

  const std::string log_content =
    build_log_content(
      log_context_.request_type,
      log_context_.prompt,
      log_context_.output_format_json,
      response);

  std::ofstream log_file(log_context_.file_path);
  if (!log_file.is_open()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to open log file: %s",
      log_context_.file_path.string().c_str());
    log_context_ = LogContext();
    return;
  }

  log_file << log_content;
  if (!log_file.good()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to write log file: %s",
      log_context_.file_path.string().c_str());
  }

  log_context_ = LogContext();
}

std::filesystem::path
AttentionIntelligence::ensure_log_directory()
{
  const std::filesystem::path log_directory = std::filesystem::current_path() / "attention_logs";

  try {
    std::filesystem::create_directories(log_directory);
  } catch (const std::filesystem::filesystem_error & error) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to create log directory '%s': %s",
      log_directory.string().c_str(),
      error.what());
  }

  return log_directory;
}

std::filesystem::path
AttentionIntelligence::build_log_file_path(LogRequestType request_type)
{
  const std::filesystem::path log_directory = ensure_log_directory();
  const std::string file_name =
    generate_log_timestamp() + "_" + get_log_type_label(request_type) + ".log";

  return log_directory / file_name;
}

std::string
AttentionIntelligence::generate_log_timestamp() const
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t raw_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time {};

  localtime_r(&raw_time, &local_time);

  std::ostringstream timestamp_stream;
  timestamp_stream << std::put_time(&local_time, "%d-%m-%Y-%H-%M-%S");

  return timestamp_stream.str();
}

std::string
AttentionIntelligence::get_log_type_label(LogRequestType request_type) const
{
  switch (request_type) {
    case LogRequestType::ACTION:
      return "ACTION";
    case LogRequestType::INPUT:
      return "INPUT";
  }

  return "UNKNOWN";
}

std::string
AttentionIntelligence::build_log_content(
  LogRequestType request_type,
  const std::string & prompt,
  const std::string & output_format_json,
  const std::string & response) const
{
  std::ostringstream log_stream;

  log_stream << "REQUEST TYPE: " << get_log_type_label(request_type) << NEW_LINE
             << NEW_LINE
             << "PROMPT:" << NEW_LINE
             << prompt << NEW_LINE
             << NEW_LINE
             << "OUTPUT FORMAT JSON:" << NEW_LINE
             << output_format_json << NEW_LINE
             << NEW_LINE
             << "RESPONSE:" << NEW_LINE
             << response << NEW_LINE;

  return log_stream.str();
}

std::string
AttentionIntelligence::build_output_format_json(
  const std::vector<KeyValue> & output_fields) const
{
  json properties = json::object();
  json required = json::array();

  for (const auto & output_field : output_fields) {
    properties[output_field.key] = build_output_field_json(output_field.value);
    required.push_back(output_field.key);
  }

  json schema = {
    {"type", "object"},
    {"properties", properties},
    {"required", required}
  };

  return schema.dump(2);
}

json
AttentionIntelligence::build_output_field_json(const std::string & field_type) const
{
  if (field_type == GetLLMResponse::Request::INT) {
    return {{"type", "integer"}};
  }

  if (field_type == GetLLMResponse::Request::STRING) {
    return {{"type", "string"}};
  }

  if (field_type == GetLLMResponse::Request::FLOAT) {
    return {{"type", "number"}};
  }

  if (field_type == GetLLMResponse::Request::BOOL) {
    return {{"type", "boolean"}};
  }

  if (field_type == GetLLMResponse::Request::INT_ARRAY) {
    return {
      {"type", "array"},
      {"items", {{"type", "integer"}}}
    };
  }

  if (field_type == GetLLMResponse::Request::STRING_ARRAY) {
    return {
      {"type", "array"},
      {"items", {{"type", "string"}}}
    };
  }

  if (field_type == GetLLMResponse::Request::FLOAT_ARRAY) {
    return {
      {"type", "array"},
      {"items", {{"type", "number"}}}
    };
  }

  if (field_type == GetLLMResponse::Request::BOOL_ARRAY) {
    return {
      {"type", "array"},
      {"items", {{"type", "boolean"}}}
    };
  }

  return {{"type", field_type}};
}


std::string
AttentionIntelligence::generate_behavior_prompt(
  const std::string & behavior_details)
{
  return behavior_details + NEW_LINE;
}

std::string
AttentionIntelligence::generate_action_prompt(
  const AttentionActionDetails & action,
  bool include_inputs)
{
  std::string output;

  output += std::string("(attention action id = ") +
              std::to_string(action.action_id) +
              std::string(") ") +
              action.action_explanation +
              NEW_LINE;

  if (include_inputs) {
    std::string input, type;
    output += std::string("  Inputs for the action:") + NEW_LINE;
    for (const auto & typed_input : action.inputs) {
      if (!parse_input_field_definition(typed_input, type, input)) {
        return "";
      }

      output += std::string("    - ") + input + NEW_LINE; 
    }
  }

  output += NEW_LINE;

  return output;
}

} // namespace attention_system
