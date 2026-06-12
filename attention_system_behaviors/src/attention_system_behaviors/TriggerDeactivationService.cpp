#include "attention_system_behaviors/TriggerDeactivationService.hpp"

#include <memory>

namespace attention_system_behaviors
{

using std::placeholders::_1;
using namespace std::chrono_literals;

TriggerDeactivationService::TriggerDeactivationService(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("TriggerDeactivationService: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
}

BT::NodeStatus
TriggerDeactivationService::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  request_done_ = false;
  request_success_ = false;

  std::string service_name;
  if (!getInput("service_name", service_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'service_name' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  client_ = node_->create_client<std_srvs::srv::Trigger>(service_name);

  if (!getInput("when_halted", when_halted_)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'when_halted' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] When halted? %d", service_name.c_str(), (int)when_halted_);

  if (when_halted_) {
    return BT::NodeStatus::RUNNING;
  }

  if (!client_->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not connect to deactivate_srv client");
    return BT::NodeStatus::FAILURE;
  }

  sendDeactivationRequest();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
TriggerDeactivationService::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  if (when_halted_) {
    return BT::NodeStatus::RUNNING;
  }

  if (request_done_) {
    if (request_success_) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void
TriggerDeactivationService::onHalted()
{
  if (when_halted_) {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_ERROR(node_->get_logger(), "Could not connect to deactivation client.");
    }

    sendDeactivationRequest();
  }
}

void TriggerDeactivationService::sendDeactivationRequest()
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  client_->async_send_request(request, std::bind(&TriggerDeactivationService::handleResponse, this, _1));
}

void
TriggerDeactivationService::handleResponse(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  const auto response = future.get();
  request_success_ = response->success;
  request_done_ = true;
  RCLCPP_INFO(
    node_->get_logger(),
    "TriggerDeactivationService completed with success=%d, message=%s",
    (int) response->success,
    response->message.c_str());
}

}

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::TriggerDeactivationService>("TriggerDeactivationService");
}

#endif
