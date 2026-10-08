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

  getInput("class", class_filter_);
  require_id_ = false;
  getInput("require_id", require_id_);
  int timeout_ms = 5000;
  getInput("timeout_ms", timeout_ms);
  timeout_ = std::chrono::milliseconds(timeout_ms);
  start_time_ = std::chrono::steady_clock::now();

  return check_detections();
}

BT::NodeStatus
GetDetections2DInformation::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  return check_detections();
}

void
GetDetections2DInformation::onHalted()
{
}

BT::NodeStatus
GetDetections2DInformation::check_detections()
{
  // Wait until the detector publishes detections of the requested class: right
  // after changing the detection prompt, OMDet needs some frames before reporting
  // them, and the tracker some more (min_hits) before assigning them an id
  vision_msgs::msg::Detection2DArray::SharedPtr detections_msg;
  {
    std::lock_guard<std::mutex> lock(detections_mutex_);
    detections_msg = last_detections_msg_;
  }

  size_t valid_detection_count = 0;
  std::string detections_information;
  if (detections_msg) {
    detections_information =
      build_detections_information(*detections_msg, valid_detection_count);
  }

  if (valid_detection_count > 0) {
    if (!setOutput("out_string", detections_information)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Output port 'out_string' missing in XML.");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (std::chrono::steady_clock::now() - start_time_ >= timeout_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "No detections%s%s%s received on %s in %ld ms",
      require_id_ ? " with tracker id" : "",
      class_filter_.empty() ? "" : " of class ",
      class_filter_.c_str(),
      DETECTION_TOPIC.c_str(),
      static_cast<long>(timeout_.count()));
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

std::string
GetDetections2DInformation::build_detections_information(
  const vision_msgs::msg::Detection2DArray & detections_msg,
  size_t & valid_detection_count) const
{
  std::ostringstream detections_information;
  valid_detection_count = 0;

  detections_information << "Here is information about the actual detections in the image\n";

  for (const auto & detection : detections_msg.detections) {
    if (detection.results.empty()) {
      continue;
    }
    if (!class_filter_.empty() &&
      detection.results.front().hypothesis.class_id != class_filter_)
    {
      continue;
    }
    if (require_id_ && detection.id.empty()) {
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
  std::lock_guard<std::mutex> lock(detections_mutex_);
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
