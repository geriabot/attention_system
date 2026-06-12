#include "attention_system_behaviors/ClearVisualDetectionClass.hpp"

#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace attention_system_behaviors
{

ClearVisualDetectionClass::ClearVisualDetectionClass(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("ClearVisualDetectionClass: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
}

BT::NodeStatus
ClearVisualDetectionClass::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  request_done_ = false;

  std::string service_name;
  if (!getInput("service_name", service_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'service_name' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  client_ = node_->create_client<SetDetectionPrompt>(service_name);

  if (!getInput("when_halted", when_halted_)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'when_halted' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] Clear visual detection class when halted: %d",
    service_name.c_str(),
    (int) when_halted_);

  if (when_halted_) {
    return BT::NodeStatus::RUNNING;
  }

  if (!client_->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not connect to OMDet prompt client.");
    return BT::NodeStatus::FAILURE;
  }

  send_clear_request();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
ClearVisualDetectionClass::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  if (when_halted_) {
    return BT::NodeStatus::RUNNING;
  }

  if (request_done_) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void
ClearVisualDetectionClass::onHalted()
{
  if (when_halted_) {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_ERROR(node_->get_logger(), "Could not connect to OMDet prompt client.");
      return;
    }

    send_clear_request();
  }
}

void
ClearVisualDetectionClass::send_clear_request()
{
  auto request = std::make_shared<SetDetectionPrompt::Request>();
  request->prompt = "";

  client_->async_send_request(
    request,
    std::bind(&ClearVisualDetectionClass::handle_response, this, _1));
}

void
ClearVisualDetectionClass::handle_response(
  rclcpp::Client<SetDetectionPrompt>::SharedFuture future)
{
  future.get();
  request_done_ = true;
  RCLCPP_INFO(node_->get_logger(), "ClearVisualDetectionClass completed.");
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::ClearVisualDetectionClass>(
    "ClearVisualDetectionClass");
}

#endif
