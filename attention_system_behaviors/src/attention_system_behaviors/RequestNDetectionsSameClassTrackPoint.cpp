#include "attention_system_behaviors/RequestNDetectionsSameClassTrackPoint.hpp"

#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace attention_system_behaviors
{

RequestNDetectionsSameClassTrackPoint::RequestNDetectionsSameClassTrackPoint(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("RequestNDetectionsSameClassTrackPoint: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);

  n_detections_ = 0;

  client_ = node_->create_client<
    attention_aux_perception_msgs::srv::StartNDetectionsSameClassTFPub>(
    "start_n_detections_same_class_tf_publisher");
}

BT::NodeStatus
RequestNDetectionsSameClassTrackPoint::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  request_done_ = false;
  request_success_ = false;

  if (!getInput("frame_id", frame_id_)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'frame_id' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("class", detection_class_name_)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'class' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("N", n_detections_)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'N' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(500ms)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Could not connect to start_n_detections_same_class_tf_publisher client");
    return BT::NodeStatus::FAILURE;
  }

  send_request();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
RequestNDetectionsSameClassTrackPoint::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  if (request_done_) {
    if (request_success_) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void
RequestNDetectionsSameClassTrackPoint::onHalted()
{
}

void
RequestNDetectionsSameClassTrackPoint::send_request()
{
  auto request = std::make_shared<
      attention_aux_perception_msgs::srv::StartNDetectionsSameClassTFPub::Request>();
  request->frame_id = frame_id_;
  request->n_detections = n_detections_;
  request->class_name = detection_class_name_;

  RCLCPP_INFO(node_->get_logger(), "Sending request: frame_id = %s, n_detections = %d, class_name = %s",
    request->frame_id.c_str(),
    request->n_detections,
    request->class_name.c_str());

  client_->async_send_request(
    request,
    std::bind(&RequestNDetectionsSameClassTrackPoint::handle_response, this, _1));
}

void
RequestNDetectionsSameClassTrackPoint::handle_response(
  rclcpp::Client<
    attention_aux_perception_msgs::srv::StartNDetectionsSameClassTFPub>::SharedFuture future)
{
  const auto response = future.get();
  request_success_ = response->success;
  request_done_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "RequestNDetectionsSameClassTrackPoint completed with success=%d",
    (int) response->success);
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<attention_system_behaviors::RequestNDetectionsSameClassTrackPoint>("RequestNDetectionsSameClassTrackPoint");
}

#endif
