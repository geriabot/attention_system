#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>

#include "attention_aux_perception_nodes/Detection3dProjectorRgbdDepth.hpp"

#include <sensor_msgs/image_encodings.hpp>

namespace attention_aux_perception_nodes
{

Detection3dProjectorRgbdDepth::Detection3dProjectorRgbdDepth()
: Node("spatial_projector_rgbd_depth_node"),
  camera_info_received_(false),
  camera_info_topic_("/rgbd_camera/camera_info"),
  depth_camera_topic_("/rgbd_camera/depth_image"),
  detections_2d_topic_("/detections_2d"),
  detections_3d_topic_("/detections_3d"),
  centroid_marker_topic_("detection_centroid_markers"),
  centroid_marker_namespace_("detection_centroids"),
  cx_(0.0), cy_(0.0), fx_(0.0), fy_(0.0),
  centroid_marker_scale_(0.05),
  centroid_marker_color_r_(0.0),
  centroid_marker_color_g_(1.0),
  centroid_marker_color_b_(0.0),
  centroid_marker_color_a_(1.0),
  previous_marker_count_(0)
{
  this->declare_parameter<std::string>("camera_info_topic", "/rgbd_camera/camera_info");
  this->declare_parameter<std::string>("depth_camera_topic", "/rgbd_camera/depth_image");
  this->declare_parameter<std::string>("detections_2d_topic", "/detections_2d");
  this->declare_parameter<std::string>("detections_3d_topic", "/detections_3d");
  this->declare_parameter<std::string>("centroid_marker_topic", "detection_centroid_markers");
  this->declare_parameter<std::string>("centroid_marker_namespace", "detection_centroids");
  this->declare_parameter<double>("centroid_marker_scale", 0.05);
  this->declare_parameter<double>("centroid_marker_color_r", 0.0);
  this->declare_parameter<double>("centroid_marker_color_g", 1.0);
  this->declare_parameter<double>("centroid_marker_color_b", 0.0);
  this->declare_parameter<double>("centroid_marker_color_a", 1.0);

  this->get_parameter("camera_info_topic", camera_info_topic_);
  this->get_parameter("depth_camera_topic", depth_camera_topic_);
  this->get_parameter("detections_2d_topic", detections_2d_topic_);
  this->get_parameter("detections_3d_topic", detections_3d_topic_);
  this->get_parameter("centroid_marker_topic", centroid_marker_topic_);
  this->get_parameter("centroid_marker_namespace", centroid_marker_namespace_);
  this->get_parameter("centroid_marker_scale", centroid_marker_scale_);
  this->get_parameter("centroid_marker_color_r", centroid_marker_color_r_);
  this->get_parameter("centroid_marker_color_g", centroid_marker_color_g_);
  this->get_parameter("centroid_marker_color_b", centroid_marker_color_b_);
  this->get_parameter("centroid_marker_color_a", centroid_marker_color_a_);

  det3d_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
    detections_3d_topic_, 10);
  centroid_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    centroid_marker_topic_, 10);

  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_camera_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Detection3dProjectorRgbdDepth::depth_image_callback, this, std::placeholders::_1));

  auto cam_info_qos = rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable();

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, cam_info_qos,
    std::bind(&Detection3dProjectorRgbdDepth::camera_info_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "Spatial Projector RGBD Depth Node initialized. Waiting for CameraInfo...");
  RCLCPP_INFO(this->get_logger(), "Using camera info topic: %s", camera_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Using depth image topic: %s", depth_camera_topic_.c_str());
}

void
Detection3dProjectorRgbdDepth::depth_image_callback(
  const sensor_msgs::msg::Image::SharedPtr msg)
{
  last_depth_image_ = msg;
}

void
Detection3dProjectorRgbdDepth::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!camera_info_received_) {
    fx_ = msg->k[0];
    cx_ = msg->k[2];
    fy_ = msg->k[4];
    cy_ = msg->k[5];

    camera_info_received_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Camera intrinsics obtained: cx=%.2f, cy=%.2f, fx=%.2f, fy=%.2f",
      cx_,
      cy_,
      fx_,
      fy_);

    cam_info_sub_.reset();
    cam_info_sub_ = nullptr;

    det_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_2d_topic_, 10,
      std::bind(&Detection3dProjectorRgbdDepth::detections_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Using detections 2D topic: %s", detections_2d_topic_.c_str());
  }
}

bool
Detection3dProjectorRgbdDepth::get_depth_from_image(
  double u,
  double v,
  double & z_meters) const
{
  if (!last_depth_image_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Detections received, but waiting for depth image...");
    return false;
  }

  const int pixel_u = static_cast<int>(std::round(u));
  const int pixel_v = static_cast<int>(std::round(v));
  const sensor_msgs::msg::Image & depth_image = *last_depth_image_;

  if (pixel_u < 0 || pixel_v < 0) {
    return false;
  }

  if (
    pixel_u >= static_cast<int>(depth_image.width) ||
    pixel_v >= static_cast<int>(depth_image.height))
  {
    return false;
  }

  if (depth_image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    return get_depth_from_32fc1(depth_image, pixel_u, pixel_v, z_meters);
  }

  if (depth_image.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    return get_depth_from_16uc1(depth_image, pixel_u, pixel_v, z_meters);
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Unsupported depth image encoding: %s",
    depth_image.encoding.c_str());
  return false;
}

bool
Detection3dProjectorRgbdDepth::get_depth_from_32fc1(
  const sensor_msgs::msg::Image & depth_image,
  int u,
  int v,
  double & z_meters) const
{
  constexpr size_t bytes_per_pixel = sizeof(float);
  const size_t byte_index = (static_cast<size_t>(v) * depth_image.step) +
    (static_cast<size_t>(u) * bytes_per_pixel);

  if (byte_index + bytes_per_pixel > depth_image.data.size()) {
    return false;
  }

  float depth_value = 0.0F;
  std::memcpy(&depth_value, &depth_image.data[byte_index], bytes_per_pixel);

  z_meters = static_cast<double>(depth_value);
  if (!std::isfinite(z_meters) || z_meters <= 0.0) {
    return false;
  }

  return true;
}

bool
Detection3dProjectorRgbdDepth::get_depth_from_16uc1(
  const sensor_msgs::msg::Image & depth_image,
  int u,
  int v,
  double & z_meters) const
{
  constexpr size_t bytes_per_pixel = sizeof(uint16_t);
  constexpr double millimeters_to_meters = 0.001;
  const size_t byte_index = (static_cast<size_t>(v) * depth_image.step) +
    (static_cast<size_t>(u) * bytes_per_pixel);

  if (byte_index + bytes_per_pixel > depth_image.data.size()) {
    return false;
  }

  uint16_t depth_value = 0U;
  std::memcpy(&depth_value, &depth_image.data[byte_index], bytes_per_pixel);

  z_meters = static_cast<double>(depth_value) * millimeters_to_meters;
  if (!std::isfinite(z_meters) || z_meters <= 0.0) {
    return false;
  }

  return true;
}

visualization_msgs::msg::Marker
Detection3dProjectorRgbdDepth::create_centroid_marker(
  const std_msgs::msg::Header & header,
  int marker_id,
  double x_meters,
  double y_meters,
  double z_meters) const
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = centroid_marker_namespace_;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x_meters;
  marker.pose.position.y = y_meters;
  marker.pose.position.z = z_meters;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = centroid_marker_scale_;
  marker.scale.y = centroid_marker_scale_;
  marker.scale.z = centroid_marker_scale_;

  marker.color.r = centroid_marker_color_r_;
  marker.color.g = centroid_marker_color_g_;
  marker.color.b = centroid_marker_color_b_;
  marker.color.a = centroid_marker_color_a_;

  return marker;
}

void
Detection3dProjectorRgbdDepth::detections_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (!camera_info_received_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Detections received, but waiting for camera intrinsic to project to 3D...");
    return;
  }

  vision_msgs::msg::Detection3DArray out_msg;
  out_msg.header = msg->header;

  visualization_msgs::msg::MarkerArray centroid_markers;
  int marker_id = 0;

  for (const auto & det2d : msg->detections) {
    if (det2d.results.empty()) {
      continue;
    }

    double u = det2d.bbox.center.position.x;
    double v = det2d.bbox.center.position.y;

    const double normalized_x = (u - cx_) / fx_;
    const double normalized_y = (v - cy_) / fy_;
    double z_meters = 0.0;

    if (!get_depth_from_image(u, v, z_meters)) {
      continue;
    }

    const double optical_x_meters = normalized_x * z_meters;
    const double optical_y_meters = normalized_y * z_meters;
    const double optical_z_meters = z_meters;

    vision_msgs::msg::Detection3D det3d;
    det3d.header = msg->header;
    det3d.id = det2d.id;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = det2d.results.front().hypothesis.class_id;
    hypothesis.hypothesis.score = det2d.results.front().hypothesis.score;

    hypothesis.pose.pose.position.x = optical_x_meters;
    hypothesis.pose.pose.position.y = optical_y_meters;
    hypothesis.pose.pose.position.z = optical_z_meters;

    det3d.results.push_back(hypothesis);

    det3d.bbox.center.position.x = optical_x_meters;
    det3d.bbox.center.position.y = optical_y_meters;
    det3d.bbox.center.position.z = optical_z_meters;

    det3d.bbox.size.x = (det2d.bbox.size_x / fx_) * optical_z_meters;
    det3d.bbox.size.y = (det2d.bbox.size_y / fy_) * optical_z_meters;
    det3d.bbox.size.z = 0.01;

    out_msg.detections.push_back(det3d);
    centroid_markers.markers.push_back(create_centroid_marker(
      msg->header,
      marker_id,
      optical_x_meters,
      optical_y_meters,
      optical_z_meters));
    marker_id++;
  }

  for (int old_marker_id = marker_id; old_marker_id < previous_marker_count_; old_marker_id++) {
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header = msg->header;
    delete_marker.ns = centroid_marker_namespace_;
    delete_marker.id = old_marker_id;
    delete_marker.action = visualization_msgs::msg::Marker::DELETE;
    centroid_markers.markers.push_back(delete_marker);
  }

  previous_marker_count_ = marker_id;

  det3d_pub_->publish(out_msg);
  centroid_marker_pub_->publish(centroid_markers);
}

}  // namespace attention_aux_perception_nodes
