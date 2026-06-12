#include "attention_aux_perception_nodes/PubSingleDetectionTF.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace attention_aux_perception_nodes
{

PubSingleDetectionTF::PubSingleDetectionTF()
: rclcpp::Node("single_detection_tf_publisher")
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_updated_ = false;
  has_detection_tf_ = false;
  is_subscribed_ = false;

  detection_sub_ = nullptr;
  timer_ = nullptr;

  start_pub_srv_ = this->create_service<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub>(
    "/start_single_detection_tf_publisher",
    std::bind(&PubSingleDetectionTF::start_cb, this, _1, _2)
  );

  stop_pub_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/stop_single_detection_tf_publisher",
    std::bind(&PubSingleDetectionTF::stop_cb, this, _1, _2)
  );

  RCLCPP_INFO(this->get_logger(), "Created single_detection_tf_publisher node");
}

void
PubSingleDetectionTF::start_cb(
  const std::shared_ptr<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub::Request> request,
  std::shared_ptr<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub::Response> response)
{
  try {
    RCLCPP_INFO(this->get_logger(), "[SINGLE DETECTION] Received request to start publishing tf");
    RCLCPP_INFO(this->get_logger(), "[SINGLE DETECTION] - frame_id: %s", request->frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "[SINGLE DETECTION] - id: %s", request->id.c_str());
    RCLCPP_INFO(this->get_logger(), "[SINGLE DETECTION] - class_name: %s", request->class_name.c_str());

    frame_id_ = request->frame_id;
    actual_detection_id_ = request->id;
    detection_class_name_ = request->class_name;

    tf_updated_ = false;
    has_detection_tf_ = false;
    optical_frame_id_.clear();

    if (!is_subscribed_) {
      RCLCPP_INFO(this->get_logger(), "[SINGLE DETECTION] Creating subscription");

      detection_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        "detections_3d", rclcpp::SensorDataQoS(),
        std::bind(&PubSingleDetectionTF::update_tf, this, _1)
      );

      is_subscribed_ = true;
    }

    timer_ = this->create_wall_timer(
      CONTROL_PERIOD, std::bind(&PubSingleDetectionTF::execute, this));

    response->success = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not start single detection publishing: " << e.what());
    response->success = false;
  }
}

void
PubSingleDetectionTF::stop_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void) request;
  
  response->success = true;
  response->message = "Single detection TF publisher node stoped.";

  detection_sub_ = nullptr;
  timer_ = nullptr;

  is_subscribed_ = false;
  tf_updated_ = false;
  has_detection_tf_ = false;
  optical_frame_id_.clear();
}

void
PubSingleDetectionTF::execute()
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
  tf_updated_ = false;
}

void
PubSingleDetectionTF::update_tf(
  const vision_msgs::msg::Detection3DArray::SharedPtr detections_msg)
{
  RCLCPP_INFO(get_logger(), "Received detections");

  bool detection_found = false;

  for (const auto & detection : detections_msg->detections) {
    if (detection.results.empty()) {
      RCLCPP_WARN(this->get_logger(), "[SINGLE DETECTION] No detection received");
      continue;
    }

    if (detection.id == actual_detection_id_ &&
      detection.results[0].hypothesis.class_id == detection_class_name_)
    {
      optical_frame_id_ = detection.header.frame_id;
      actual_detection_id_ = detection.id;

      cam2detection_tf_.header.stamp = detection.header.stamp;
      cam2detection_tf_.header.frame_id = optical_frame_id_;
      cam2detection_tf_.child_frame_id = "detection_from_optical_frame";

      cam2detection_tf_.transform.translation.x = detection.bbox.center.position.x;
      cam2detection_tf_.transform.translation.y = detection.bbox.center.position.y;
      cam2detection_tf_.transform.translation.z = detection.bbox.center.position.z;
      cam2detection_tf_.transform.rotation.x = 0.0;
      cam2detection_tf_.transform.rotation.y = 0.0;
      cam2detection_tf_.transform.rotation.z = 0.0;
      cam2detection_tf_.transform.rotation.w = 1.0;

      tf_updated_ = true;
      has_detection_tf_ = true;
      detection_found = true;
      break;
    }
  }

  if (!detection_found) {
    tf_updated_ = false;
    RCLCPP_WARN(
      this->get_logger(),
      "[SINGLE DETECTION] Detection id=%s class=%s not found in current message",
      actual_detection_id_.c_str(),
      detection_class_name_.c_str());
  }
}

void
PubSingleDetectionTF::publish_detection_tf(std::string child_frame_id)
{
  geometry_msgs::msg::TransformStamped odom2detection_msg;
  const rclcpp::Time detection_stamp(cam2detection_tf_.header.stamp);

  tf2::Stamped<tf2::Transform> cam2detection;
  tf2::fromMsg(cam2detection_tf_, cam2detection);

  std::string error;

  if (tf_buffer_->canTransform(
      "odom",
      optical_frame_id_,
      detection_stamp,
      rclcpp::Duration::from_nanoseconds(0),
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

  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Error in TF odom -> " << optical_frame_id_ << " [<< " << error << "]");
  }
}

} // namespace attention_aux_perception_nodes
