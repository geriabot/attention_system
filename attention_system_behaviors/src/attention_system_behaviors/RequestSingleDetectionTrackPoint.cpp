#include "attention_system_behaviors/RequestSingleDetectionTrackPoint.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace attention_system_behaviors
{

RequestSingleDetectionTrackPoint::RequestSingleDetectionTrackPoint(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("RequestSingleDetectionTrackPoint: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);

  detection_class_id_ = "";
  frame_id_ = "";
  actual_detection_id_ = "";

  client_ = node_->create_client<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub>("start_single_detection_tf_publisher");
}

BT::NodeStatus
RequestSingleDetectionTrackPoint::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  if (!check_input_and_request()) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
RequestSingleDetectionTrackPoint::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  if (!check_input_and_request()) {
    return BT::NodeStatus::FAILURE;
  }

  auto status = response_future_.wait_for(10ms);

  if (status == std::future_status::ready) {
    auto response = response_future_.get();

    if (response->success) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void
RequestSingleDetectionTrackPoint::onHalted()
{
  client_ = nullptr;
}

bool
RequestSingleDetectionTrackPoint::check_input_and_request()
{
  std::string frame_id;
  if (!getInput("frame_id", frame_id)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'frame_id' missing in XML.");
    return false;
  }

  std::string class_name;
  if (!getInput("class", class_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'class' missing in XML.");
    return false;
  }

  std::string detection_id;
  if (!getInput("id", detection_id)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'id' missing in XML.");
    return false;
  }

  if (frame_id != frame_id_ || class_name != detection_class_id_ || detection_id != actual_detection_id_) {
    RCLCPP_INFO(node_->get_logger(), "Last: %s, %s, %s", frame_id_.c_str(), detection_class_id_.c_str(), actual_detection_id_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Now:  %s, %s, %s", frame_id.c_str(), class_name.c_str(), detection_id.c_str());
    
    frame_id_ = frame_id;
    detection_class_id_ = class_name;
    actual_detection_id_ = detection_id;

    if (!client_->wait_for_service(500ms)) {
      RCLCPP_ERROR(node_->get_logger(), "Could not connect to start_single_detection_tf_publisher client");
      return false;
    }

    response_future_ = make_request();
  }

  return true;
}


std::shared_future<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub::Response::SharedPtr>
RequestSingleDetectionTrackPoint::make_request()
{
  auto request = std::make_shared<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub::Request>();
  request->frame_id = frame_id_;
  request->id = actual_detection_id_;
  request->class_name = detection_class_id_;

  RCLCPP_INFO(node_->get_logger(), "Sending request: frame_id = %s, id = %s, class_name = %s",
    request->frame_id.c_str(),
    request->id.c_str(),
    request->class_name.c_str());

  return client_->async_send_request(request).future.share();;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::RequestSingleDetectionTrackPoint>("RequestSingleDetectionTrackPoint");
}

#endif