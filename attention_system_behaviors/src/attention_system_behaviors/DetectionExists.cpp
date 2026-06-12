#include "attention_system_behaviors/DetectionExists.hpp"

#include <sstream>

using std::placeholders::_1;

namespace attention_system_behaviors
{

DetectionExists::DetectionExists(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("DetectionExists: 'node' not found in blackboard");
  }
  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  last_detections_msg_ = nullptr;

  detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "/detections_3d",
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&DetectionExists::detections_callback, this, _1));
}

BT::NodeStatus
DetectionExists::tick()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  std::string class_name;
  std::string detection_id;

  if (!getInput("class", class_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'class' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("id", detection_id)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'id' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Checking detection (class: %s, id: %s)", class_name.c_str(), detection_id.c_str());

  if (detection_exists(class_name, detection_id)) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

bool
DetectionExists::detection_exists(
  const std::string & class_name,
  const std::string & detection_id)
{
  if (!last_detections_msg_) {
    return false;
  }

  for (const auto & detection : last_detections_msg_->detections) {
    if (detection.results.empty()) {
      continue;
    }

    if (detection.id == detection_id &&
      detection.results[0].hypothesis.class_id == class_name) {

      RCLCPP_INFO(node_->get_logger(), "Detection exists!!!");

      return true;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Detection does not exists");


  return false;
}

void
DetectionExists::detections_callback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  std::ostringstream detection_ids;
  bool first_detection = true;

  for (const auto & detection : msg->detections) {
    if (!first_detection) {
      detection_ids << ", ";
    }

    detection_ids << detection.id;
    first_detection = false;
  }

  //RCLCPP_INFO(
  //  node_->get_logger(),
  //  "DetectionExists received detection ids: [%s]",
  //  detection_ids.str().c_str());

  last_detections_msg_ = msg;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::DetectionExists>("DetectionExists");
}

#endif
