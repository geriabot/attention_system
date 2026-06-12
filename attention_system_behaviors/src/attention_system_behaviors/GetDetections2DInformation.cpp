#include "attention_system_behaviors/GetDetections2DInformation.hpp"

#include <sstream>

using std::placeholders::_1;

namespace attention_system_behaviors
{

GetDetections2DInformation::GetDetections2DInformation(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("GetDetections2DInformation: 'node' not found in blackboard");
  }

  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  last_detections_msg_ = nullptr;

  detection_sub_ =
    node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    DETECTION_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast(detection_qos_depth_)),
    std::bind(&GetDetections2DInformation::detections_callback, this, _1));
}

BT::NodeStatus
GetDetections2DInformation::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  if (!setOutput("out_string", build_detections_information())) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Output port 'out_string' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
GetDetections2DInformation::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  return BT::NodeStatus::SUCCESS;
}

void
GetDetections2DInformation::onHalted()
{
}

std::string
GetDetections2DInformation::build_detections_information() const
{
  if (!last_detections_msg_ || last_detections_msg_->detections.empty()) {
    return "";
  }

  std::ostringstream detections_information;
  size_t valid_detection_count = 0;

  detections_information << "Here is information about the actual detections in the image\n";

  for (const auto & detection : last_detections_msg_->detections) {
    RCLCPP_INFO(node_->get_logger(), "Id: %s", detection.id.c_str());

    if (detection.results.empty()) {
      continue;
    }

    valid_detection_count++;

    if (valid_detection_count > 1) {
      detections_information << "\n";
    }

    detections_information
      << "Detection " << valid_detection_count << ": "
      << "id=" << detection.id
      << ", centroid=(x=" << detection.bbox.center.position.x
      << ", y=" << detection.bbox.center.position.y
      << "), bbox=(width=" << detection.bbox.size_x
      << ", height=" << detection.bbox.size_y
      << "), score=" << detection.results.front().hypothesis.score;
  }

  return detections_information.str();
}

void
GetDetections2DInformation::detections_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  last_detections_msg_ = msg;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::GetDetections2DInformation>(
    "GetDetections2DInformation");
}

#endif
