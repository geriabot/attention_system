#include "attention_system_behaviors/ChangeVisualDetectionClass.hpp"

using namespace std::chrono_literals;

namespace attention_system_behaviors
{

ChangeVisualDetectionClass::ChangeVisualDetectionClass(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("ChangeVisualDetectionClass: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);

  RCLCPP_INFO(node_->get_logger(), "CREATED NODE FOR CVDC");
}

BT::NodeStatus
ChangeVisualDetectionClass::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  RCLCPP_INFO(node_->get_logger(), "[CVDC] onStart");

  std::string det_class;
  if (!getInput("class", det_class)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'class' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  std::string service_name;
  if (!getInput("service_name", service_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'service_name' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  prompt_cli_ = node_->create_client<SetDetectionPrompt>(service_name);

  if (!prompt_cli_->wait_for_service(3s)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not connect to %s client", service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<SetDetectionPrompt::Request>();
  request->prompt = det_class;

  response_future_ = prompt_cli_->async_send_request(request).future.share();
  RCLCPP_INFO(node_->get_logger(), "Making asyncronous request: %s", det_class.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
ChangeVisualDetectionClass::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  RCLCPP_INFO(node_->get_logger(), "[CVDC] onRunning");

  auto status = response_future_.wait_for(10ms);

  RCLCPP_INFO(node_->get_logger(), "Response status: %i", (int) status);

  if (status == std::future_status::ready) {
    auto response = response_future_.get();
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::ChangeVisualDetectionClass>(
    "ChangeVisualDetectionClass");
}
#endif
