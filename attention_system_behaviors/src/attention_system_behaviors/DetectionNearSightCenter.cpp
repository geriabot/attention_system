#include "attention_system_behaviors/DetectionNearSightCenter.hpp"

using std::placeholders::_1;

namespace attention_system_behaviors
{

DetectionNearSightCenter::DetectionNearSightCenter(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("DetectionNearSightCenter: 'node' not found in blackboard");
  }

  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  last_detections_msg_ = nullptr;
  last_camera_info_msg_ = nullptr;
  last_tracked_detection_ids_msg_ = nullptr;

  const int detection_qos_depth = 10;
  detection_sub_ =
    node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detections_2d",
    rclcpp::QoS(rclcpp::KeepLast(detection_qos_depth)),
    std::bind(&DetectionNearSightCenter::detections_callback, this, _1));

  const int camera_info_qos_depth = 1;
  auto camera_info_qos = rclcpp::QoS(
    rclcpp::KeepLast(camera_info_qos_depth)).reliable().durability_volatile();

  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/color/camera_info",
    camera_info_qos,
    std::bind(&DetectionNearSightCenter::camera_info_callback, this, _1));

  tracked_detection_ids_sub_ =
    node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/tracked_detection_ids",
    rclcpp::QoS(rclcpp::KeepLast(detection_qos_depth)),
    std::bind(&DetectionNearSightCenter::tracked_detection_ids_callback, this, _1));
}

BT::NodeStatus
DetectionNearSightCenter::tick()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  std::string side;
  std::string class_name;
  double limit_percentage;

  if (!getInput("side", side)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'side' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (side != "right" && side != "left") {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'side' must be 'right' or 'left'.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("class", class_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'class' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("limit_percentage", limit_percentage)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'limit_percentage' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  const double max_limit_percentage = 100.0;
  if (limit_percentage <= 0.0 || limit_percentage > max_limit_percentage) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'limit_percentage' must be greater than zero and less than or equal to %f.",
      max_limit_percentage);
    return BT::NodeStatus::FAILURE;
  }

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[DNSC] Tick: side=%s, class=%s, limit_percentage=%.2f",
  //   side.c_str(),
  //   class_name.c_str(),
  //   limit_percentage);

  if (detection_near_sight_center(side, class_name, limit_percentage)) {
    // RCLCPP_INFO(
    //   node_->get_logger(),
    //   "[DNSC] Result: detection near %s sight limit for class '%s'.",
    //   side.c_str(),
    //   class_name.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[DNSC] Result: no detection near %s sight limit for class '%s'.",
  //   side.c_str(),
  //   class_name.c_str());

  return BT::NodeStatus::FAILURE;
}

bool
DetectionNearSightCenter::detection_near_sight_center(
  const std::string & side,
  const std::string & class_name,
  double limit_percentage) const
{
  vision_msgs::msg::Detection2DArray::SharedPtr detections_msg;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;
  vision_msgs::msg::Detection2DArray::SharedPtr tracked_detection_ids_msg;

  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    detections_msg = last_detections_msg_;
    camera_info_msg = last_camera_info_msg_;
    tracked_detection_ids_msg = last_tracked_detection_ids_msg_;
  }

  if (!camera_info_msg) {
    RCLCPP_WARN(node_->get_logger(), "CameraInfo not received yet.");
    return false;
  }

  if (!detections_msg) {
    RCLCPP_WARN(node_->get_logger(), "Detections not received yet.");
    return false;
  }

  if (!tracked_detection_ids_msg) {
    RCLCPP_WARN(node_->get_logger(), "Tracked detection ids not received yet.");
    return false;
  }

  if (tracked_detection_ids_msg->detections.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Tracked detection ids list is empty.");
    return false;
  }

  if (camera_info_msg->width == 0u) {
    RCLCPP_WARN(node_->get_logger(), "CameraInfo width must be greater than zero.");
    return false;
  }

  std::unordered_set<std::string> tracked_detection_ids;
  tracked_detection_ids.reserve(tracked_detection_ids_msg->detections.size());
  for (const auto & detection : tracked_detection_ids_msg->detections) {
    if (detection.results.empty()) {
      continue;
    }

    if (detection.results.front().hypothesis.class_id != class_name) {
      continue;
    }

    tracked_detection_ids.insert(detection.id);
  }

  if (tracked_detection_ids.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No tracked detection ids match the requested class.");
    return false;
  }

  const double image_width = static_cast<double>(camera_info_msg->width);
  const double image_center = (image_width / 2.0);
  const double limit_px =
    (image_width / 2.0) * (limit_percentage / 100.0);
  const double right_limit = image_center + limit_px;
  const double left_limit = image_center - limit_px;

  for (const auto & detection : detections_msg->detections) {
    if (detection.results.empty()) {
      // RCLCPP_INFO(node_->get_logger(), "[DNSC] Ignoring detection without results.");
      continue;
    }

    if (tracked_detection_ids.find(detection.id) == tracked_detection_ids.end()) {
      continue;
    }

    if (detection.results.front().hypothesis.class_id != class_name) {
      continue;
    }

    const double center_x = detection.bbox.center.position.x;

    if (side == "right" && center_x >= image_center && center_x <= right_limit) {
      return true;
    }

    if (side == "left" && center_x <= image_center && center_x >= left_limit) {
      return true;
    }
  }

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[DNSC] No matching detection reached the configured sight limit.");

  return false;
}

void
DetectionNearSightCenter::detections_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  last_detections_msg_ = msg;
}

void
DetectionNearSightCenter::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    last_camera_info_msg_ = msg;
  }

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[DNSC] Received CameraInfo message: width=%u, height=%u",
  //   msg->width,
  //   msg->height);

  camera_info_sub_ = nullptr;
}

void
DetectionNearSightCenter::tracked_detection_ids_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  last_tracked_detection_ids_msg_ = msg;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::DetectionNearSightCenter>(
    "DetectionNearSightCenter");
}

#endif
