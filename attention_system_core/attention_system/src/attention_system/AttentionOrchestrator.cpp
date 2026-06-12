#include "attention_system/AttentionOrchestrator.hpp"
#include "behavior_architecture/orchestrator_factory.hpp"

namespace attention_system
{

using std::placeholders::_1;
using std::placeholders::_2;

static behavior_architecture::OrchestratorRegistrar<AttentionOrchestrator>
  attention_orchestrator_registrar(ATTENTION_ORCHESTRATOR_NAME);

AttentionOrchestrator::AttentionOrchestrator(BT::Blackboard::Ptr blackboard)
    : behavior_architecture::BaseOrchestrator(ATTENTION_ORCHESTRATOR_NAME,
                                              blackboard)
{
  status_publisher_ = this->create_publisher<AttentionSystemStatus>(
    "/attention_system/status", 10);
  behavior_status_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/behavior_status", 10,
    std::bind(&AttentionOrchestrator::behavior_status_callback, this, _1));

  start_intelligence_client_ =
    this->create_client<TriggerService>("/attention/start_intelligence");
  stop_intelligence_client_ =
    this->create_client<TriggerService>("/attention/stop_intelligence");

  behavior_returned_failure_ = false;
}

// #####################################
void
AttentionOrchestrator::parse_attention_behaviors_parameters()
{
  RCLCPP_INFO(this->get_logger(), "Parsing attention behaviors parameters...");

  std::vector<std::string> behavior_names;
  this->declare_parameter<std::vector<std::string>>(
      "attention_behaviors.behaviors");
  this->get_parameter("attention_behaviors.behaviors", behavior_names);
  RCLCPP_INFO(this->get_logger(), "%zu available attention behaviors.",
              behavior_names.size());

  std::vector<std::string> actuation_capabilities;
  this->declare_parameter<std::vector<std::string>>("actuation_capabilities");
  this->get_parameter("actuation_capabilities", actuation_capabilities);

  RCLCPP_INFO(this->get_logger(), "BEHAVIORS:");
  for (const auto &behavior_name : behavior_names) {
    std::string prefix = "attention_behaviors." + behavior_name + ".";

    AttentionBehaviorParams behavior;

    behavior.name = behavior_name;

    // RCLCPP_INFO(this->get_logger(), "To parse behavior_id...");
    this->declare_parameter<int>(prefix + "behavior_id");
    this->get_parameter(prefix + "behavior_id", behavior.behavior_id);

    // RCLCPP_INFO(this->get_logger(), "To parse actuation capabilities
    // needed...");
    std::string act_prefix = prefix + "actuation_capabilities_needed.";

    for (const std::string &cap_name : actuation_capabilities) {
      // Search for the capability in our struct using the string from the YAML
      AttentionActuationCapability *cap =
          behavior.behavior_actuation_needed.get_capability_by_name(cap_name);

      if (cap != nullptr) {
        cap->name = cap_name; // Assign the name so that operator<< and
                              // iterators work properly

        // Declare and get the "needed" boolean
        this->declare_parameter<bool>(act_prefix + cap_name + ".needed");
        this->get_parameter(act_prefix + cap_name + ".needed", cap->value);

        // Declare and get the "alternatives" vector
        std::vector<std::string> alt;
        this->declare_parameter<std::vector<std::string>>(
            act_prefix + cap_name + ".alternatives");
        this->get_parameter(act_prefix + cap_name + ".alternatives", alt);

        if (!(alt.size() == 1 && alt[0] == "")) {
          cap->alternative_capabilities = alt;
        }

      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Capability '%s' found in YAML list does not exist.",
                    cap_name.c_str());
      }
    }

    // RCLCPP_INFO(this->get_logger(), "To parse explanation...");
    this->declare_parameter<std::string>(prefix +
                                         "prompt_information.explanation");
    this->get_parameter(prefix + "prompt_information.explanation",
                        behavior.prompt_information.explanation);
    behavior.prompt_information.explanation =
        behavior_name + std::string(":\n") +
        behavior.prompt_information.explanation;

    // RCLCPP_INFO(this->get_logger(), "To parse inputs...");
    this->declare_parameter<std::vector<std::string>>(
        prefix + "prompt_information.inputs");
    this->get_parameter(prefix + "prompt_information.inputs",
                        behavior.prompt_information.inputs);

    // RCLCPP_INFO(this->get_logger(), "To parse not assigned inputs...");

    this->declare_parameter<std::vector<std::string>>(
        prefix + "bt_blackboard_inputs.not_assigned");
    this->get_parameter(prefix + "bt_blackboard_inputs.not_assigned",
                        behavior.bt_blackboard_inputs_not_assigned);

    // RCLCPP_INFO(this->get_logger(), "To parse assigned inputs...");

    std::vector<std::string> assigned_inputs;
    this->declare_parameter<std::vector<std::string>>(
        prefix + "bt_blackboard_inputs.assigned");
    this->get_parameter(prefix + "bt_blackboard_inputs.assigned",
                        assigned_inputs);

    for (const auto &input : assigned_inputs) {
      std::vector<std::string> current_group;
      std::stringstream ss(input);
      std::string token;

      while (std::getline(ss, token, ':')) {
        current_group.push_back(token);
      }

      behavior.bt_blackboard_inputs_assigned[current_group[0]] =
          current_group[1];
    }

    std::string node_name;
    this->declare_parameter<std::string>(prefix + "node_name");
    this->get_parameter(prefix + "node_name", node_name);

    behavior.node_name = node_name;

    RCLCPP_INFO_STREAM(this->get_logger(), "- " << behavior_name << std::endl
                                                << behavior);

    available_attention_behaviors_[behavior.behavior_id] = behavior;
  }
}

void
AttentionOrchestrator::behavior_status_callback(
  std_msgs::msg::String::UniquePtr msg)
{
  if (msg->data == "FAILURE") {
    behavior_returned_failure_ = true;
  }
}

// ##########################################

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AttentionOrchestrator::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  auto parent_result =
    behavior_architecture::BaseOrchestrator::on_activate(previous_state);

  last_action_id_ = -1;

  must_terminate_use_attention_request_ = false;
  behavior_returned_failure_ = false;
  acting_state_initialized_ = false;

  if (parent_result != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed BaseOrchestrator activation.");
    return parent_result;
  }

  // +++++++++ INTELLIGENCE ++++++++++

  bool connected = false;
  for (int i = 0; i < MAX_INTELLIGENCE_CONNECTION_ATTEMPTS; i++) {
    RCLCPP_INFO(this->get_logger(), "Attempt no. %d to request intelligence activation.", i + 1);

    if (start_intelligence_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Correctly connected to intelligence start service.");
      connected = true;
      break;
    }
  }

  if (!connected) {
    RCLCPP_ERROR(this->get_logger(), "Could not connect to intelligence start service.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  auto start_intelligence_request = std::make_shared<TriggerService::Request>();
  start_intelligence_client_->async_send_request(start_intelligence_request);

  if (!this->has_parameter("context_details")) {
    this->declare_parameter<std::vector<std::string>>("context_details", std::vector<std::string>({""}));
  }
  this->get_parameter("context_details", context_details_);

  context_details_str_ = "";

  if (context_details_.size() == 1 && context_details_[0] == "") {
    RCLCPP_INFO(get_logger(), "No context details specified");
  } else {
    for (const auto &detail : context_details_) {
      context_details_str_ += "- " + detail + "\n";
    }

    RCLCPP_INFO_STREAM(get_logger(), "context_details: \n" << context_details_str_);
  }

  intelligence_result_received_ = false;
  waiting_for_at_action_result_ = false;
  waiting_for_behavior_input_result_ = false;
  use_attention_request_active_ = false;

  // USE ATTENTION
  use_attention_service_ = this->create_service<UseAttention>(
    USE_ATTENTION_SERVICE_NAME,
    std::bind(&AttentionOrchestrator::handle_use_attention_request, this, _1, _2));

  // ASK GEMINI BEHAVIOR ACTION
  attention_intelligence_at_action_client_ =
    this->create_client<AskForAttentionBehavior>(
      "attention/ask_intelligence_for_behavior");

  status_publisher_->on_activate();

  // +++++++++++++++++++++++++++++++++

  this->parse_attention_behaviors_parameters();

  control_state_ = AttentionState::IDLE;

  attention_action_activated_ = false;

  RCLCPP_INFO(get_logger(), "ATTENTION ORCHESTRATOR SUCCESSFULLY ACTIVATED");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AttentionOrchestrator::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  auto parent_result =
      behavior_architecture::BaseOrchestrator::on_deactivate(previous_state);

  if (parent_result != rclcpp_lifecycle::node_interfaces::
                           LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed BaseOrchestrator deactivation.");
    return parent_result;
  }

  if (use_attention_request_active_) {
    terminate_use_attention(UseAttention::Response::CANCELED, "");
  }

  use_attention_service_ = nullptr;

  control_state_ = AttentionState::IDLE;

  status_publisher_->on_deactivate();

  use_attention_request_header_ = nullptr;
  use_attention_request_ = nullptr;
  use_attention_request_active_ = false;

  if (!start_intelligence_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->get_logger(), "Could not connect to intelligence start service.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  auto stop_intelligence_request = std::make_shared<TriggerService::Request>();
  stop_intelligence_client_->async_send_request(stop_intelligence_request);

  deactivate_all_runners();

  attention_action_activated_ = false;

  RCLCPP_INFO(get_logger(), "ATTENTION ORCHESTRATOR DEACTIVATED");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

void
AttentionOrchestrator::clear_active_attention_behaviors()
{
  deactivate_all_runners();
  active_attention_behaviors_.clear();
}

void
AttentionOrchestrator::reset_operational_state()
{
  clear_active_attention_behaviors();
  attention_action_activated_ = false;
  attention_action_details_published_ = false;
  acting_state_initialized_ = false;
  last_action_id_ = -1;
}

void
AttentionOrchestrator::set_assigned_blackboard_input(
  const std::string & key,
  const std::string & value)
{
  if (value.rfind("<int>", 0) == 0) {
    RCLCPP_INFO(get_logger(), "Is int");
    blackboard_->set(key, std::stoi(value.substr(5)));
  } else if (value.rfind("<bool>", 0) == 0) {
    RCLCPP_INFO(get_logger(), "Is bool");
    const auto bool_value = value.substr(6);
    blackboard_->set(key, bool_value == "true");
  } else if (value.rfind("<uint>", 0) == 0) {
    RCLCPP_INFO(get_logger(), "Is uint");
    blackboard_->set(
      key,
      static_cast<unsigned int>(std::stoul(value.substr(6))));
  } else if (value.rfind("<double>", 0) == 0) {
    blackboard_->set(key, std::stod(value.substr(8)));
  } else {
    RCLCPP_INFO(get_logger(), "Is string");
    blackboard_->set(key, value);
  }
}

std::string
AttentionOrchestrator::get_assigned_blackboard_input_as_string(
  const std::string & key,
  const std::string & value)
{
  if (value.rfind("<int>", 0) == 0) {
    return std::to_string(blackboard_->get<int>(key));
  } else if (value.rfind("<bool>", 0) == 0) {
    return blackboard_->get<bool>(key) ? "true" : "false";
  } else if (value.rfind("<uint>", 0) == 0) {
    return std::to_string(blackboard_->get<unsigned int>(key));
  } else if (value.rfind("<double>", 0) == 0) {
    return std::to_string(blackboard_->get<double>(key));
  }

  return blackboard_->get<std::string>(key);
}

void
AttentionOrchestrator::publish_status()
{
  AttentionSystemStatus status_msg;

  if (must_terminate_use_attention_request_ &&
      use_attention_result_status_ != UseAttention::Response::CANCELED) {
    status_msg.status = AttentionSystemStatus::FAILED;
  } else if (control_state_ == AttentionState::IDLE) {
    if (behavior_returned_failure_) {
      status_msg.status = AttentionSystemStatus::FAILED;
    } else {
      status_msg.status = AttentionSystemStatus::IDLE;
    }
  } else if (control_state_ == AttentionState::ACTING) {
    status_msg.status = AttentionSystemStatus::RUNNING;
  } else {
    status_msg.status = AttentionSystemStatus::REASONING;
  }

  status_publisher_->publish(status_msg);
}

/**
 * @brief go_to_state base_orchestrator's method overriding.
 *
 * Here we dont change the state of an FSM, we assing blackboard inputs, clear
 * active attention behaviors and activate the correspondant behavior.
 */
void
AttentionOrchestrator::go_to_state(int state)
{
  if (last_action_id_ != state) {
    last_action_id_ = state;

    clear_active_attention_behaviors();

    auto behavior = available_attention_behaviors_[state];

    for (size_t i = 0; i < behavior.bt_blackboard_inputs_not_assigned.size();
         i++) {
      blackboard_->set(behavior.bt_blackboard_inputs_not_assigned[i],
                       actual_action_.inputs[i]);
    }

    for (const auto &[key, value] : behavior.bt_blackboard_inputs_assigned) {
      set_assigned_blackboard_input(key, value);
    }

    activate_runner(behavior.node_name);
    
    active_attention_behaviors_.push_back(behavior.node_name);
  }
}

void
AttentionOrchestrator::control_cycle()
{
  try {
    this->publish_status();

    // If use attention request must terminate
    if (must_terminate_use_attention_request_) {
      // Service requests cannot be canceled. Ignore late responses.
      if (control_state_ == AttentionState::SETTING_BEHAVIOR_ACTION) {
        action_request_sent_ = false;
      } else if (control_state_ == AttentionState::SETTING_BEHAVIOR_INPUT) {
        waiting_for_behavior_input_result_ = false;
      }

      terminate_use_attention(use_attention_result_status_, "");
      control_state_ = AttentionState::IDLE;
      must_terminate_use_attention_request_ = false;
      return;
    }

    attention_system::AttentionBehaviorParams action;
    std::string req_inputs;


    switch (control_state_) {
    case AttentionState::IDLE:
      reset_operational_state();
      break;

    case AttentionState::SETTING_BEHAVIOR_ACTION:
      attention_action_activated_ = false;
      attention_action_details_published_ = false;

      if (intelligence_result_received_) {
        clear_active_attention_behaviors();

        // TODO Gestionar fallos
        if (waiting_for_at_action_result_) {
          RCLCPP_INFO(this->get_logger(), "Parsing action");
          auto action = available_attention_behaviors_[at_action_result_];

          actual_action_.action_explanation =
              action.prompt_information.explanation;
          actual_action_.action_id = action.behavior_id;
          actual_action_.inputs = action.prompt_information.inputs;

          control_state_ = AttentionState::SETTING_BEHAVIOR_INPUT;
          waiting_for_at_action_result_ = false;
          action_request_sent_ = false;
          intelligence_result_received_ = false;
        }
      } else {

        if (!waiting_for_at_action_result_) {
          use_attention(use_attention_request_);
        }
      }

      break;

    case AttentionState::SETTING_BEHAVIOR_INPUT:
      clear_active_attention_behaviors();

      attention_action_activated_ = false;
      attention_action_details_published_ = false;

      action = available_attention_behaviors_[actual_action_.action_id];

      for (const auto &input : action.bt_blackboard_inputs_assigned) {
        set_assigned_blackboard_input(input.first, input.second);
        RCLCPP_INFO(this->get_logger(), "Set in blackboard: %s -> %s",
                    input.first.c_str(), input.second.c_str());

        RCLCPP_INFO(this->get_logger(), "Is set: %s",
                    get_assigned_blackboard_input_as_string(
                      input.first,
                      input.second).c_str());
      }

      blackboard_->set("req_task_details", actual_task_);
      blackboard_->set("req_context_details", context_details_str_);
      blackboard_->set("req_behavior_details", actual_behavior_details_);
      blackboard_->set("req_action_explanation", actual_action_.action_explanation);

      for (size_t i = 0; i < action.prompt_information.inputs.size(); i++) {
        req_inputs += action.prompt_information.inputs[i];
        if (i + 1 < action.prompt_information.inputs.size()) {
          req_inputs += ":";
        }
      }
      blackboard_->set("req_inputs", req_inputs);

      req_inputs.clear();
      for (size_t i = 0; i < action.bt_blackboard_inputs_not_assigned.size(); i++) {
        req_inputs += action.bt_blackboard_inputs_not_assigned[i];
        if (i + 1 < action.bt_blackboard_inputs_not_assigned.size()) {
          req_inputs += ":";
        }
      }
      blackboard_->set("req_blackboard_inputs", req_inputs);

      waiting_for_behavior_input_result_ = false;
      intelligence_result_received_ = false;

      blackboard_->set("can_move_around", actual_available_actuation_.move_around.value);
      blackboard_->set("can_use_joint", actual_available_actuation_.use_joint.value);
      blackboard_->set("can_turn_around", actual_available_actuation_.turn_around.value);

      frame_id_ = ATTENTION_FRAME_ID; // TODO Habrá que parametrizarlo o
                                      // cogerlo de la blackboard

      control_state_ = AttentionState::ACTING;
      acting_state_initialized_ = false;

      break;

    case AttentionState::ACTING:
      if (!acting_state_initialized_) {
        go_to_state(actual_action_.action_id);
        acting_state_initialized_ = true;

        if (!attention_action_activated_) {
          attention_action_activated_ = true;
          RCLCPP_INFO(get_logger(), "Attention action activated (%s)",
                      available_attention_behaviors_[actual_action_.action_id]
                          .name.c_str());
        }

        terminate_use_attention(UseAttention::Response::SUCCESSFUL, frame_id_);
      } else if (check_behavior_finished()) {
        reset_operational_state();
        control_state_ = AttentionState::IDLE;
      }

      break;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Error in control_cylce: " << e.what() << std::endl);
  }
}

void
AttentionOrchestrator::handle_use_attention_request(
  std::shared_ptr<rmw_request_id_t> request_header,
  UseAttention::Request::SharedPtr request)
{
  if (use_attention_request_active_) {
    UseAttention::Response response;
    response.status = UseAttention::Response::ABORTED;
    response.frame_id = "";
    use_attention_service_->send_response(*request_header, response);
    RCLCPP_WARN(
      get_logger(),
      "UseAttention request rejected because another request is active");
    return;
  }

  if (control_state_ == AttentionState::ACTING) {
    reset_operational_state();
  }

  must_terminate_use_attention_request_ = false;
  behavior_returned_failure_ = false;

  RCLCPP_INFO(get_logger(), "handle_use_attention_request");

  control_state_ = AttentionState::SETTING_BEHAVIOR_ACTION;
  use_attention_request_header_ = request_header;
  use_attention_request_ = request;
  use_attention_request_active_ = true;

  intelligence_result_received_ = false;
  waiting_for_at_action_result_ = false;
  waiting_for_behavior_input_result_ = false;
  action_request_sent_ = false;
  acting_state_initialized_ = false;
}

void
AttentionOrchestrator::use_attention(UseAttention::Request::SharedPtr request)
{
  if (!request) {
    use_attention_result_status_ = UseAttention::Response::UNKNOWN;
    must_terminate_use_attention_request_ = true;
    return;
  }

  actual_task_ = request->task_details;
  actual_behavior_details_ = request->behavior_details;

  RCLCPP_INFO(get_logger(), "USE ATTENTION NEW REQUEST!!!");
  RCLCPP_INFO(
    get_logger(),
    "Task details: %s",
    request->task_details.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Robot's behavior details: %s",
    actual_behavior_details_.c_str());

  waiting_for_at_action_result_ = true;
  call_at_action_intelligence_client(request);

  RCLCPP_INFO(get_logger(), "<use_attention> END");
}

void
AttentionOrchestrator::terminate_use_attention(int8_t status, std::string frame_id)
{
  // If there is no active request, return
  if (!use_attention_request_active_ || !use_attention_request_header_) {
    return;
  }

  UseAttention::Response response;
  response.status = status;
  response.frame_id = frame_id;

  use_attention_service_->send_response(*use_attention_request_header_, response);

  if (status == UseAttention::Response::SUCCESSFUL) {
    RCLCPP_INFO(this->get_logger(), "UseAttention request SUCCEEDED");
  } else if (status == UseAttention::Response::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "UseAttention request CANCELED");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "UseAttention request ABORTED with status: %d", status);
  }

  use_attention_request_header_ = nullptr;
  use_attention_request_ = nullptr;
  use_attention_request_active_ = false;
}

// #############################################

void
AttentionOrchestrator::call_at_action_intelligence_client(
  UseAttention::Request::SharedPtr prompt_information)
{
  action_request_sent_ = true;

  RCLCPP_INFO(get_logger(), "<behavior_action> INIT");

  auto request = std::make_shared<AskForAttentionBehavior::Request>();

  RCLCPP_INFO(get_logger(), "<behavior_action> request created");

  request->task_details = prompt_information->task_details;
  request->context_details = context_details_str_;
  request->behavior_details = prompt_information->behavior_details;

  std::vector<AttentionActionDetails> actions;

  // Get available actuation capabilities or this action
  actual_available_actuation_.turn_around.name = "turn_around";
  actual_available_actuation_.turn_around.value =
      prompt_information->can_turn_around;

  actual_available_actuation_.move_around.name = "move_around";
  actual_available_actuation_.move_around.value = prompt_information->can_move_around;

  actual_available_actuation_.use_joint.name = "use_joint";
  actual_available_actuation_.use_joint.value = prompt_information->can_use_joint;

  for (const auto &action : available_attention_behaviors_) {
    if (!actual_available_actuation_.meets_requirements(action.second.behavior_actuation_needed)) {
      continue;
    }

    AttentionActionDetails avail_action;

    avail_action.action_explanation =
        action.second.prompt_information.explanation;
    avail_action.action_id = action.second.behavior_id;
    avail_action.inputs = action.second.prompt_information.inputs;

    actions.push_back(avail_action);

    RCLCPP_INFO(this->get_logger(), "Adding action");
  }

  if (actions.empty()) {
    must_terminate_use_attention_request_ = true;
    use_attention_result_status_ =
        UseAttention::Response::NO_ATTENTION_BEHAVIORS_AVAILABLE;
    RCLCPP_WARN(get_logger(),
                "There are no available actions for making the prompt.");
    return;
  }

  request->attention_actions = actions;

  RCLCPP_INFO(get_logger(), "<behavior_action> request initialized");

  while (!attention_intelligence_at_action_client_->wait_for_service(
      ACTION_WAIT_TIME)) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "<behavior_action> Service not available, waiting again...");
  }

  RCLCPP_INFO(get_logger(), "<behavior_action> service responded");

  attention_intelligence_at_action_client_->async_send_request(
    request,
    std::bind(
      &AttentionOrchestrator::at_action_intelligence_result_callback,
      this,
      _1));

  RCLCPP_INFO(get_logger(), "<behavior_action> request sent");
}

void
AttentionOrchestrator::at_action_intelligence_result_callback(
  rclcpp::Client<AskForAttentionBehavior>::SharedFuture result)
{
  if (!action_request_sent_ || !waiting_for_at_action_result_) {
    RCLCPP_WARN(
      get_logger(),
      "<at_action_result_cb> Ignoring late behavior action response");
    return;
  }

  auto response = result.get();
  intelligence_result_received_ = true;

  if (!response->success) {
    RCLCPP_ERROR(
      get_logger(),
      "<at_action_result_cb> Service failed: %s",
      response->message.c_str());
    must_terminate_use_attention_request_ = true;
    use_attention_result_status_ = UseAttention::Response::ABORTED;
    return;
  }

  at_action_result_ = response->attention_action_id;

  RCLCPP_INFO(get_logger(), "Behavior's action received succesfully");
}


} // namespace attention_system
