#ifndef ATTENTION_AUX_PERCEPTION_NODES__DETECTION_3D_PROJECTOR_ASTRA_DEPHT_HPP
#define ATTENTION_AUX_PERCEPTION_NODES__DETECTION_3D_PROJECTOR_ASTRA_DEPHT_HPP

#include <astra_camera_msgs/msg/extrinsics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <string>

namespace attention_aux_perception_nodes
{

class Detection3dProjectorAstraDepht : public rclcpp::Node
{

public:
  Detection3dProjectorAstraDepht();
  virtual ~Detection3dProjectorAstraDepht() = default;

private:
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<astra_camera_msgs::msg::Extrinsics>::SharedPtr extrinsics_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr det3d_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centroid_marker_pub_;

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void depth_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void extrinsics_callback(const astra_camera_msgs::msg::Extrinsics::SharedPtr msg);
  void detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  bool get_depth_from_image(
    double u,
    double v,
    double & z_meters) const;
  bool get_depth_from_32fc1(
    const sensor_msgs::msg::Image & depth_image,
    int u,
    int v,
    double & z_meters) const;
  bool get_depth_from_16uc1(
    const sensor_msgs::msg::Image & depth_image,
    int u,
    int v,
    double & z_meters) const;
  visualization_msgs::msg::Marker create_centroid_marker(
    const std_msgs::msg::Header & header,
    int marker_id,
    double x_meters,
    double y_meters,
    double z_meters) const;
  bool transform_depth_to_color(
    double & x_meters,
    double & y_meters,
    double & z_meters) const;

  bool camera_info_received_;
  bool depth_camera_info_received_;
  std::string depth_camera_topic_;
  std::string depth_to_color_extrinsics_topic_;
  std::string centroid_marker_topic_;
  std::string centroid_marker_namespace_;
  sensor_msgs::msg::Image::SharedPtr last_depth_image_;
  astra_camera_msgs::msg::Extrinsics::SharedPtr last_depth_to_color_extrinsics_;
  double cx_;
  double cy_;
  double fx_;
  double fy_;
  double depth_cx_;
  double depth_cy_;
  double depth_fx_;
  double depth_fy_;
  double centroid_marker_scale_;
  double centroid_marker_color_r_;
  double centroid_marker_color_g_;
  double centroid_marker_color_b_;
  double centroid_marker_color_a_;
  double extrinsics_translation_scale_;
  int previous_marker_count_;
};

}  // namespace attention_aux_perception_nodes

#endif  // ATTENTION_AUX_PERCEPTION_NODES__DETECTION_3D_PROJECTOR_ASTRA_DEPHT_HPP
