#include "attention_system_behaviors/ActivateTwisting.hpp"

using namespace std::chrono_literals;

namespace attention_system_behaviors
{

ActivateTwisting::ActivateTwisting(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("ActivateTwisting: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);

  client_ = node_->create_client<attention_actuation_msgs::srv::StartTwisting>(
    "/start_twisting");
}

BT::NodeStatus
ActivateTwisting::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  double lin_x;
  double lin_y;
  double lin_z;
  double ang_x;
  double ang_y;
  double ang_z;
  std::string topic;

  if (!getInput("lin_x", lin_x)) {
    lin_x = 0.0;
  }

  if (!getInput("lin_y", lin_y)) {
    lin_y = 0.0;
  }

  if (!getInput("lin_z", lin_z)) {
    lin_z = 0.0;
  }

  if (!getInput("ang_x", ang_x)) {
    ang_x = 0.0;
  }

  if (!getInput("ang_y", ang_y)) {
    ang_y = 0.0;
  }

  if (!getInput("ang_z", ang_z)) {
    ang_z = 0.0;
  }

  if (!getInput("topic", topic)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'topic' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (topic.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'topic' cannot be empty.");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(500ms)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not connect to /start_twisting service");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<attention_actuation_msgs::srv::StartTwisting::Request>();
  request->twist.linear.x = lin_x;
  request->twist.linear.y = lin_y;
  request->twist.linear.z = lin_z;
  request->twist.angular.x = ang_x;
  request->twist.angular.y = ang_y;
  request->twist.angular.z = ang_z;
  request->topic = topic;

  RCLCPP_INFO(
    node_->get_logger(),
    "Sending start twisting request: topic = %s",
    topic.c_str());

  response_future_ = client_->async_send_request(request).future.share();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
ActivateTwisting::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  auto status = response_future_.wait_for(10ms);

  if (status == std::future_status::ready) {
    auto response = response_future_.get();

    if (response->success) {
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "Start twisting service returned failure: %s",
      response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void
ActivateTwisting::onHalted()
{
}

}  // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::ActivateTwisting>("ActivateTwisting");
}

#endif
