#include "attention_aux_perception_nodes/PubNDetectionsSameClassTF.hpp"

#include <unordered_map>
#include <unordered_set>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace attention_aux_perception_nodes
{

PubNDetectionsSameClassTF::PubNDetectionsSameClassTF()
: rclcpp::Node("n_detections_same_class_tf_publisher")
{
  this->declare_parameter<int>("tf_lookup_timeout_ms", 100);
  this->declare_parameter<std::string>(
    "tracked_detection_ids_topic",
    "/tracked_detection_ids");
  this->declare_parameter<std::string>(
    "detection_point_topic",
    "/n_detections_same_class_point");
  this->get_parameter("tf_lookup_timeout_ms", tf_lookup_timeout_ms_);
  this->get_parameter("tracked_detection_ids_topic", tracked_detection_ids_topic_);
  this->get_parameter("detection_point_topic", detection_point_topic_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_updated_ = false;
  has_detection_tf_ = false;
  is_subscribed_ = false;

  detection_sub_ = nullptr;
  timer_ = nullptr;

  tracked_ids_pub_ =
    this->create_publisher<vision_msgs::msg::Detection2DArray>(
    tracked_detection_ids_topic_,
    10);
  detection_point_pub_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>(
    detection_point_topic_,
    10);

  start_pub_srv_ = this->create_service<attention_aux_perception_msgs::srv::StartNDetectionsSameClassTFPub>(
    "/start_n_detections_same_class_tf_publisher",
    std::bind(&PubNDetectionsSameClassTF::start_cb, this, _1, _2)
  );

  stop_pub_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/stop_n_detections_same_class_tf_publisher",
    std::bind(&PubNDetectionsSameClassTF::stop_cb, this, _1, _2)
  );

  RCLCPP_INFO(this->get_logger(), "Created n_detections_same_class_tf_publisher node");
}

void
PubNDetectionsSameClassTF::reset_tracking_state()
{
  tf_updated_ = false;
  has_detection_tf_ = false;
  optical_frame_id_.clear();
  tracked_detection_ids_.clear();
  missing_iterations_by_id_.clear();
  publish_tracked_detection_ids();
}

void
PubNDetectionsSameClassTF::start_cb(
  const std::shared_ptr<attention_aux_perception_msgs::srv::StartNDetectionsSameClassTFPub::Request> request,
  std::shared_ptr<attention_aux_perception_msgs::srv::StartNDetectionsSameClassTFPub::Response> response)
{
  try {
    RCLCPP_INFO(this->get_logger(), "[N DETECTIONS SAME CLASS] Received request to start publishing tf");
    RCLCPP_INFO(this->get_logger(), "[N DETECTIONS SAME CLASS] - frame_id: %s", request->frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "[N DETECTIONS SAME CLASS] - n_detections: %d", request->n_detections);
    RCLCPP_INFO(this->get_logger(), "[N DETECTIONS SAME CLASS] - class_name: %s", request->class_name.c_str());

    frame_id_ = request->frame_id;
    n_detections_ = request->n_detections;
    detection_class_name_ = request->class_name;

    reset_tracking_state();

    if (!is_subscribed_) {
      RCLCPP_INFO(this->get_logger(), "[N DETECTIONS SAME CLASS] Creating subscription");

      detection_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        "detections_3d", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&PubNDetectionsSameClassTF::update_tf, this, _1)
      );

      is_subscribed_ = true;
    }

    timer_ = this->create_wall_timer(
      CONTROL_PERIOD, std::bind(&PubNDetectionsSameClassTF::execute, this));

    response->success = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not start n detections publishing: " << e.what());
    response->success = false;
  }
}

void
PubNDetectionsSameClassTF::stop_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void) request;
  
  response->success = true;
  response->message = "N detections same class TF publisher node stoped.";

  detection_sub_ = nullptr;
  timer_ = nullptr;

  is_subscribed_ = false;
  reset_tracking_state();
}

void
PubNDetectionsSameClassTF::execute()
{
  if (!tf_updated_) {
    return;
  }

  if (optical_frame_id_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Optical frame id not obtained");
    return;
  }

  if (frame_id_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Goal frame id not obtained");
    return;
  }

  publish_detection_tf(frame_id_);
  publish_tracked_detection_ids();
  tf_updated_ = false;
}

void
PubNDetectionsSameClassTF::update_tf(
  const vision_msgs::msg::Detection3DArray::SharedPtr detections_msg)
{
  std::vector<const vision_msgs::msg::Detection3D *> valid_detections;
  std::unordered_map<std::string, const vision_msgs::msg::Detection3D *> detections_by_id;

  //RCLCPP_INFO(this->get_logger(), "Updating TF...");

  for (const auto & detection : detections_msg->detections) {
    if (detection.results.empty()) {
      continue;
    }

    if (detection.results[0].hypothesis.class_id != detection_class_name_) {
      continue;
    }

    valid_detections.push_back(&detection);
    detections_by_id[detection.id] = &detection;
  }

  tf_updated_ = false;

  if (n_detections_ <= 0) {
    tracked_detection_ids_.clear();
    missing_iterations_by_id_.clear();

    if (valid_detections.empty()) {
      has_detection_tf_ = false;
      publish_tracked_detection_ids();
      return;
    }

    tracked_detection_ids_.reserve(valid_detections.size());
    for (const auto * detection : valid_detections) {
      tracked_detection_ids_.push_back(detection->id);
    }

    if (update_tracked_tf(valid_detections)) {
      has_detection_tf_ = true;
      tf_updated_ = true;
      return;
    }

    has_detection_tf_ = false;
    return;
  }

  const std::size_t required_detections = static_cast<std::size_t>(n_detections_);

  if (tracked_detection_ids_.empty()) {
    if (valid_detections.size() < required_detections) {
      has_detection_tf_ = false;
      RCLCPP_INFO(
        this->get_logger(),
        "[N DETECTIONS SAME CLASS] Waiting for %d detections of class %s. Current valid detections: %zu",
        n_detections_,
        detection_class_name_.c_str(),
        valid_detections.size());
      return;
    }

    tracked_detection_ids_.reserve(required_detections);
    for (std::size_t i = 0; i < required_detections; ++i) {
      tracked_detection_ids_.push_back(valid_detections[i]->id);
      missing_iterations_by_id_[valid_detections[i]->id] = 0;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[N DETECTIONS SAME CLASS] Initialized tracking with %zu ids",
      tracked_detection_ids_.size());
  }

  std::vector<const vision_msgs::msg::Detection3D *> tracked_detections(tracked_detection_ids_.size(), nullptr);
  bool all_tracked_ids_found = true;

  for (std::size_t i = 0; i < tracked_detection_ids_.size(); ++i) {
    const auto & tracked_id = tracked_detection_ids_[i];
    const auto detection_it = detections_by_id.find(tracked_id);

    if (detection_it != detections_by_id.end()) {
      tracked_detections[i] = detection_it->second;
      missing_iterations_by_id_[tracked_id] = 0;
    } else {
      all_tracked_ids_found = false;
      missing_iterations_by_id_[tracked_id] += 1;
      RCLCPP_WARN(
        this->get_logger(),
        "[N DETECTIONS SAME CLASS] Detection id=%s missing. Iteration %d/%d",
        tracked_id.c_str(),
        missing_iterations_by_id_[tracked_id],
        max_missing_iterations_);
    }
  }

  if (all_tracked_ids_found && update_tracked_tf(tracked_detections)) {
    has_detection_tf_ = true;
    tf_updated_ = true;
    return;
  }

  std::unordered_set<std::string> tracked_ids_set(
    tracked_detection_ids_.begin(), tracked_detection_ids_.end());

  for (std::size_t i = 0; i < tracked_detection_ids_.size(); ++i) {
    const std::string previous_id = tracked_detection_ids_[i];
    const auto missing_it = missing_iterations_by_id_.find(previous_id);

    if (missing_it == missing_iterations_by_id_.end()) {
      continue;
    }

    if (missing_it->second < max_missing_iterations_) {
      continue;
    }

    const vision_msgs::msg::Detection3D * replacement_detection = nullptr;
    for (const auto * detection : valid_detections) {
      if (tracked_ids_set.find(detection->id) == tracked_ids_set.end()) {
        replacement_detection = detection;
        break;
      }
    }

    if (replacement_detection == nullptr) {
      RCLCPP_WARN(
        this->get_logger(),
        "[N DETECTIONS SAME CLASS] No replacement candidate available for missing id=%s",
        previous_id.c_str());
      continue;
    }

    tracked_ids_set.erase(previous_id);
    tracked_detection_ids_[i] = replacement_detection->id;
    tracked_ids_set.insert(replacement_detection->id);
    missing_iterations_by_id_.erase(previous_id);
    missing_iterations_by_id_[replacement_detection->id] = 0;
    tracked_detections[i] = replacement_detection;

    RCLCPP_INFO(
      this->get_logger(),
      "[N DETECTIONS SAME CLASS] Replaced missing id=%s with id=%s",
      previous_id.c_str(),
      replacement_detection->id.c_str());
  }

  bool all_updated_ids_found = true;
  tracked_detections.assign(tracked_detection_ids_.size(), nullptr);

  for (std::size_t i = 0; i < tracked_detection_ids_.size(); ++i) {
    const auto detection_it = detections_by_id.find(tracked_detection_ids_[i]);
    if (detection_it == detections_by_id.end()) {
      all_updated_ids_found = false;
      break;
    }

    tracked_detections[i] = detection_it->second;
  }

  if (all_updated_ids_found && update_tracked_tf(tracked_detections)) {
    has_detection_tf_ = true;
    tf_updated_ = true;
    return;
  }

  has_detection_tf_ = false;
}

bool
PubNDetectionsSameClassTF::update_tracked_tf(
  const std::vector<const vision_msgs::msg::Detection3D *> & tracked_detections)
{
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  rclcpp::Time stamp;
  bool frame_id_set = false;

  for (const auto * detection : tracked_detections) {
    if (detection == nullptr) {
      return false;
    }

    sum_x += detection->bbox.center.position.x;
    sum_y += detection->bbox.center.position.y;
    sum_z += detection->bbox.center.position.z;

    if (!frame_id_set) {
      optical_frame_id_ = detection->header.frame_id;
      stamp = detection->header.stamp;
      frame_id_set = true;
    }
  }

  if (!frame_id_set || tracked_detections.empty()) {
    return false;
  }

  const double tracked_count = static_cast<double>(tracked_detections.size());

  RCLCPP_INFO(
    this->get_logger(),
    "[N DETECTIONS SAME CLASS] Sum x: %f, y: %f, z: %f, count: %zu",
    sum_x,
    sum_y,
    sum_z,
    tracked_detections.size());

  cam2detection_tf_.header.stamp = stamp;
  cam2detection_tf_.header.frame_id = optical_frame_id_;
  cam2detection_tf_.child_frame_id = "detection_from_optical_frame";

  cam2detection_tf_.transform.translation.x = sum_x / tracked_count;
  cam2detection_tf_.transform.translation.y = sum_y / tracked_count;
  cam2detection_tf_.transform.translation.z = sum_z / tracked_count;
  cam2detection_tf_.transform.rotation.x = 0.0;
  cam2detection_tf_.transform.rotation.y = 0.0;
  cam2detection_tf_.transform.rotation.z = 0.0;
  cam2detection_tf_.transform.rotation.w = 1.0;

  return true;
}

void
PubNDetectionsSameClassTF::publish_detection_tf(std::string child_frame_id)
{
  geometry_msgs::msg::TransformStamped odom2detection_msg;
  const rclcpp::Time detection_stamp(cam2detection_tf_.header.stamp);
  const auto tf_lookup_timeout_nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::milliseconds(tf_lookup_timeout_ms_));
  const rclcpp::Duration tf_lookup_timeout =
    rclcpp::Duration::from_nanoseconds(tf_lookup_timeout_nanoseconds.count());

  tf2::Stamped<tf2::Transform> cam2detection;
  tf2::fromMsg(cam2detection_tf_, cam2detection);

  std::string error;

  if (tf_buffer_->canTransform(
      "odom",
      optical_frame_id_,
      detection_stamp,
      tf_lookup_timeout,
      &error))
  {
    auto odom2cam_msg = tf_buffer_->lookupTransform(
      "odom", optical_frame_id_, tf2::TimePointZero);

    tf2::Stamped<tf2::Transform> odom2cam;
    tf2::fromMsg(odom2cam_msg, odom2cam);

    auto odom2detection = odom2cam * cam2detection;

    odom2detection_msg.header.stamp = this->now();
    odom2detection_msg.header.frame_id = "odom";
    odom2detection_msg.child_frame_id = child_frame_id;

    odom2detection_msg.transform.translation.x = odom2detection.getOrigin().x();
    odom2detection_msg.transform.translation.y = odom2detection.getOrigin().y();
    odom2detection_msg.transform.translation.z = odom2detection.getOrigin().z();
    odom2detection_msg.transform.rotation.x = 0.0;
    odom2detection_msg.transform.rotation.y = 0.0;
    odom2detection_msg.transform.rotation.z = 0.0;
    odom2detection_msg.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(odom2detection_msg);

    geometry_msgs::msg::PointStamped detection_point_msg;
    detection_point_msg.header = odom2detection_msg.header;
    detection_point_msg.point.x = odom2detection_msg.transform.translation.x;
    detection_point_msg.point.y = odom2detection_msg.transform.translation.y;
    detection_point_msg.point.z = odom2detection_msg.transform.translation.z;

    detection_point_pub_->publish(detection_point_msg);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Error in TF odom -> " << optical_frame_id_ << " [<< " << error << "]");
  }
}

void
PubNDetectionsSameClassTF::publish_tracked_detection_ids()
{
  if (tracked_ids_pub_ == nullptr) {
    return;
  }

  vision_msgs::msg::Detection2DArray tracked_ids_msg;
  tracked_ids_msg.header.stamp = this->now();
  tracked_ids_msg.header.frame_id = optical_frame_id_;

  tracked_ids_msg.detections.reserve(tracked_detection_ids_.size());
  for (const auto & id : tracked_detection_ids_) {
    vision_msgs::msg::Detection2D detection;
    detection.header = tracked_ids_msg.header;
    detection.id = id;
    detection.results.resize(1);
    detection.results.front().hypothesis.class_id = detection_class_name_;
    tracked_ids_msg.detections.push_back(detection);
  }

  tracked_ids_pub_->publish(tracked_ids_msg);
}

} // namespace attention_aux_perception_nodes
