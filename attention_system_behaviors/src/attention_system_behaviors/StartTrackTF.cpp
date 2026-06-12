#include "attention_system_behaviors/StartTrackTF.hpp"

using namespace std::chrono_literals;

namespace attention_system_behaviors
{

StartTrackTF::StartTrackTF(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("StartTrackTF: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
}

BT::NodeStatus
StartTrackTF::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  RCLCPP_INFO(node_->get_logger(), "[STTF] onStart");

  std::string frame_id;
  if (!getInput("frame_id", frame_id)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'frame_id' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  track_tf_cli_ = node_->create_client<StartTracking>("/start_tf_tracking");

  if (!track_tf_cli_->wait_for_service(3s)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not connect to /start_tf_tracking client");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<StartTracking::Request>();
  request->frame_id = frame_id;

  response_future_ = track_tf_cli_->async_send_request(request).future.share();
  RCLCPP_INFO(node_->get_logger(), "Making asynchronous request: frame_id = %s", frame_id.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
StartTrackTF::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  RCLCPP_INFO(node_->get_logger(), "[STTF] onRunning");

  auto status = response_future_.wait_for(10ms);

  RCLCPP_INFO(node_->get_logger(), "Response status: %i", static_cast<int>(status));

  if (status == std::future_status::ready) {
    auto response = response_future_.get();
    RCLCPP_INFO(
      node_->get_logger(),
      "TrackTF response: success=%d, message=%s",
      (int) response->success,
      response->message.c_str());

    if (response->success) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::StartTrackTF>("StartTrackTF");
}

#endif
