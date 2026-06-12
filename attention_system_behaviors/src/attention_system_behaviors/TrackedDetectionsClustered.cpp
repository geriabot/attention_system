#include "attention_system_behaviors/TrackedDetectionsClustered.hpp"

using std::placeholders::_1;

namespace attention_system_behaviors
{

TrackedDetectionsClustered::TrackedDetectionsClustered(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("TrackedDetectionsClustered: 'node' not found in blackboard");
  }

  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  last_detections_msg_ = nullptr;
  last_tracked_detection_ids_msg_ = nullptr;
  has_camera_info_ = false;
  image_area_ = 0.0;

  const int detection_qos_depth = 10;
  detection_sub_ =
    node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detections_2d",
    rclcpp::QoS(rclcpp::KeepLast(detection_qos_depth)),
    std::bind(&TrackedDetectionsClustered::detections_callback, this, _1));

  const int camera_info_qos_depth = 1;
  auto camera_info_qos = rclcpp::QoS(
    rclcpp::KeepLast(camera_info_qos_depth)).reliable().durability_volatile();

  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/color/camera_info",
    camera_info_qos,
    std::bind(&TrackedDetectionsClustered::camera_info_callback, this, _1));

  tracked_detection_ids_sub_ =
    node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/tracked_detection_ids",
    rclcpp::QoS(rclcpp::KeepLast(detection_qos_depth)),
    std::bind(&TrackedDetectionsClustered::tracked_detection_ids_callback, this, _1));
}

BT::NodeStatus
TrackedDetectionsClustered::tick()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  std::string class_name;
  double max_area_percentage;

  if (!getInput("class", class_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'class' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("max_area_percentage", max_area_percentage)) {
    RCLCPP_ERROR(node_->get_logger(), "Input port 'max_area_percentage' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (max_area_percentage <= 0.0 || max_area_percentage > 100.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'max_area_percentage' must be greater than zero and less than or equal to 100.");
    return BT::NodeStatus::FAILURE;
  }

  if (tracked_detections_clustered(class_name, max_area_percentage)) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

bool
TrackedDetectionsClustered::tracked_detections_clustered(
  const std::string & class_name,
  double max_area_percentage) const
{
  vision_msgs::msg::Detection2DArray::SharedPtr detections_msg;
  vision_msgs::msg::Detection2DArray::SharedPtr tracked_detection_ids_msg;
  bool has_camera_info;
  double image_area;

  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    detections_msg = last_detections_msg_;
    tracked_detection_ids_msg = last_tracked_detection_ids_msg_;
    has_camera_info = has_camera_info_;
    image_area = image_area_;
  }

  if (!has_camera_info) {
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

  if (tracked_detection_ids.size() < MIN_TRACKED_DETECTIONS) {
    //RCLCPP_WARN(
    //  node_->get_logger(),
    //  "At least %zu tracked detections are required.",
    //  MIN_TRACKED_DETECTIONS);
    return true;
  }

  std::vector<DetectionBounds> tracked_detection_bounds;
  tracked_detection_bounds.reserve(tracked_detection_ids.size());

  for (const auto & detection : detections_msg->detections) {
    if (detection.results.empty()) {
      continue;
    }

    if (tracked_detection_ids.find(detection.id) == tracked_detection_ids.end()) {
      continue;
    }

    if (detection.results.front().hypothesis.class_id != class_name) {
      continue;
    }

    const double half_width = detection.bbox.size_x / 2.0;
    const double half_height = detection.bbox.size_y / 2.0;

    DetectionBounds detection_bounds;
    detection_bounds.id = detection.id;
    detection_bounds.min_x = detection.bbox.center.position.x - half_width;
    detection_bounds.max_x = detection.bbox.center.position.x + half_width;
    detection_bounds.min_y = detection.bbox.center.position.y - half_height;
    detection_bounds.max_y = detection.bbox.center.position.y + half_height;
    tracked_detection_bounds.push_back(detection_bounds);
  }

  if (!all_tracked_detections_found(tracked_detection_ids, tracked_detection_bounds)) {
    RCLCPP_WARN(node_->get_logger(), "Not all tracked detections have current 2D detections.");
    return false;
  }

  if (image_area <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Image area must be greater than zero.");
    return false;
  }

  return bounding_box_area_within_limit(
    tracked_detection_bounds,
    image_area,
    max_area_percentage);
}

bool
TrackedDetectionsClustered::all_tracked_detections_found(
  const std::unordered_set<std::string> & tracked_detection_ids,
  const std::vector<DetectionBounds> & tracked_detection_bounds) const
{
  if (tracked_detection_bounds.size() != tracked_detection_ids.size()) {
    return false;
  }

  std::unordered_set<std::string> found_detection_ids;
  found_detection_ids.reserve(tracked_detection_bounds.size());
  for (const auto & detection_bounds : tracked_detection_bounds) {
    found_detection_ids.insert(detection_bounds.id);
  }

  for (const auto & id : tracked_detection_ids) {
    if (found_detection_ids.find(id) == found_detection_ids.end()) {
      return false;
    }
  }

  return true;
}

bool
TrackedDetectionsClustered::bounding_box_area_within_limit(
  const std::vector<DetectionBounds> & tracked_detection_bounds,
  double image_area,
  double max_area_percentage) const
{
  double min_x = tracked_detection_bounds.front().min_x;
  double max_x = tracked_detection_bounds.front().max_x;
  double min_y = tracked_detection_bounds.front().min_y;
  double max_y = tracked_detection_bounds.front().max_y;

  for (const auto & detection_bounds : tracked_detection_bounds) {
    if (detection_bounds.min_x < min_x) {
      min_x = detection_bounds.min_x;
    }

    if (detection_bounds.max_x > max_x) {
      max_x = detection_bounds.max_x;
    }

    if (detection_bounds.min_y < min_y) {
      min_y = detection_bounds.min_y;
    }

    if (detection_bounds.max_y > max_y) {
      max_y = detection_bounds.max_y;
    }
  }

  const double cluster_width = max_x - min_x;
  const double cluster_height = max_y - min_y;
  const double cluster_area = cluster_width * cluster_height;
  const double cluster_area_percentage = (cluster_area / image_area) * 100.0;

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[TrackedDetectionsClustered] Cluster size: width=%.2f px, height=%.2f px, "
  //   "area=%.2f%% of image area, limit=%.2f%%",
  //   cluster_width,
  //   cluster_height,
  //   cluster_area_percentage,
  //   max_area_percentage);

  if (cluster_area_percentage > max_area_percentage) {
    return false;
  }

  return true;
}

void
TrackedDetectionsClustered::detections_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  last_detections_msg_ = msg;
}

void
TrackedDetectionsClustered::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  const double image_width = static_cast<double>(msg->width);
  const double image_height = static_cast<double>(msg->height);
  const double image_area = image_width * image_height;

  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    has_camera_info_ = msg->width > 0u && msg->height > 0u;
    image_area_ = image_area;
  }

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "[DNSL] Received CameraInfo message: width=%u, height=%u",
  //   msg->width,
  //   msg->height);

  camera_info_sub_ = nullptr;
}

void
TrackedDetectionsClustered::tracked_detection_ids_callback(
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
  factory.registerNodeType<attention_system_behaviors::TrackedDetectionsClustered>(
    "TrackedDetectionsClustered");
}

#endif
