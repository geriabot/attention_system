#include "attention_system_behaviors/DetectionNearSightLimit.hpp"

#include <cmath>

using std::placeholders::_1;

namespace attention_system_behaviors
{

DetectionNearSightLimit::DetectionNearSightLimit(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("DetectionNearSightLimit: 'node' not found in blackboard");
  }

  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  last_detections_msg_ = nullptr;
  last_camera_info_msg_ = nullptr;

  const int detection_qos_depth = 10;
  detection_sub_ =
    node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detections_2d",
    rclcpp::QoS(rclcpp::KeepLast(detection_qos_depth)),
    std::bind(&DetectionNearSightLimit::detections_callback, this, _1));

  const int camera_info_qos_depth = 1;
  auto camera_info_qos = rclcpp::QoS(
    rclcpp::KeepLast(camera_info_qos_depth)).reliable().durability_volatile();

  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/color/camera_info",
    camera_info_qos,
    std::bind(&DetectionNearSightLimit::camera_info_callback, this, _1));
}

BT::NodeStatus
DetectionNearSightLimit::tick()
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
  //   "[DNSL] Tick: side=%s, class=%s, limit_percentage=%.2f",
  //   side.c_str(),
  //   class_name.c_str(),
  //   limit_percentage);

  if (detection_near_sight_limit(side, class_name, limit_percentage)) {
    // RCLCPP_INFO(
    //   node_->get_logger(),
    //   "[DNSL] Result: detection near %s sight limit for class '%s'.",
    //   side.c_str(),
    //   class_name.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[DNSL] Result: no detection near %s sight limit for class '%s'.",
  //   side.c_str(),
  //   class_name.c_str());

  return BT::NodeStatus::FAILURE;
}

bool
DetectionNearSightLimit::detection_near_sight_limit(
  const std::string & side,
  const std::string & class_name,
  double limit_percentage) const
{
  vision_msgs::msg::Detection2DArray::SharedPtr detections_msg;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;

  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    detections_msg = last_detections_msg_;
    camera_info_msg = last_camera_info_msg_;
  }

  if (!camera_info_msg) {
    RCLCPP_WARN(node_->get_logger(), "CameraInfo not received yet.");
    return false;
  }

  if (!detections_msg) {
    RCLCPP_WARN(node_->get_logger(), "Detections not received yet.");
    return false;
  }

  if (camera_info_msg->width == 0u) {
    RCLCPP_WARN(node_->get_logger(), "CameraInfo width must be greater than zero.");
    return false;
  }

  const double image_width = static_cast<double>(camera_info_msg->width);
  const double half_image_width = image_width / 2.0;
  const double limit_px =
    half_image_width * (limit_percentage / 100.0);
  const double right_limit = image_width - limit_px;
  const double left_limit = limit_px;

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[DNSL] Image width=%.2f, left_limit=%.2f, right_limit=%.2f, detections=%zu",
  //   image_width,
  //   left_limit,
  //   right_limit,
  //   detections_msg->detections.size());

  for (const auto & detection : detections_msg->detections) {
    if (detection.results.empty()) {
      // RCLCPP_INFO(node_->get_logger(), "[DNSL] Ignoring detection without results.");
      continue;
    }

    if (detection.results.front().hypothesis.class_id != class_name) {
      // RCLCPP_INFO(
      //   node_->get_logger(),
      //   "[DNSL] Ignoring detection class '%s' while looking for '%s'.",
      //   detection.results.front().hypothesis.class_id.c_str(),
      //   class_name.c_str());
      continue;
    }

    const double center_x = detection.bbox.center.position.x;
    double limit_position = right_limit;
    double signed_distance_to_limit = center_x - right_limit;
    if (side == "left") {
      limit_position = left_limit;
      signed_distance_to_limit = left_limit - center_x;
    }

    const double distance_to_limit_percentage =
      (std::abs(signed_distance_to_limit) / half_image_width) * 100.0;
    bool limit_reached = false;
    if (signed_distance_to_limit >= 0.0) {
      limit_reached = true;
    }
    const char * limit_reached_text = "false";
    if (limit_reached) {
      limit_reached_text = "true";
    }

    // RCLCPP_INFO(
    //   node_->get_logger(),
    //   "[DNSL] Detection id='%s' class='%s' side='%s': center_x=%.2f, limit_x=%.2f, "
    //   "distance_to_limit=%.2f%%, reached=%s",
    //   detection.id.c_str(),
    //   class_name.c_str(),
    //   side.c_str(),
    //   center_x,
    //   limit_position,
    //   distance_to_limit_percentage,
    //   limit_reached_text);

    if (side == "right" && center_x >= right_limit) {
      // RCLCPP_INFO(
      //   node_->get_logger(),
      //   "[DNSL] Detection reached right limit: center_x=%.2f >= %.2f",
      //   center_x,
      //   right_limit);
      return true;
    }

    if (side == "left" && center_x <= left_limit) {
      // RCLCPP_INFO(
      //   node_->get_logger(),
      //   "[DNSL] Detection reached left limit: center_x=%.2f <= %.2f",
      //   center_x,
      //   left_limit);
      return true;
    }
  }

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[DNSL] No matching detection reached the configured sight limit.");

  return false;
}

void
DetectionNearSightLimit::detections_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  last_detections_msg_ = msg;
}

void
DetectionNearSightLimit::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    last_camera_info_msg_ = msg;
  }

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[DNSL] Received CameraInfo message: width=%u, height=%u",
  //   msg->width,
  //   msg->height);

  camera_info_sub_ = nullptr;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::DetectionNearSightLimit>(
    "DetectionNearSightLimit");
}

#endif
