#ifndef ATTENTION_SYSTEM_BEHAVIORS__TRACKED_DETECTIONS_CLUSTERED_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__TRACKED_DETECTIONS_CLUSTERED_HPP

#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace attention_system_behaviors
{

class TrackedDetectionsClustered : public BT::ConditionNode
{
public:
  explicit TrackedDetectionsClustered(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("class"),
        BT::InputPort<double>("max_area_percentage")
      });
  }

private:
  struct DetectionBounds
  {
    std::string id;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
  };

  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
    detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
    tracked_detection_ids_sub_;

  vision_msgs::msg::Detection2DArray::SharedPtr last_detections_msg_;
  vision_msgs::msg::Detection2DArray::SharedPtr last_tracked_detection_ids_msg_;
  bool has_camera_info_;
  double image_area_;
  mutable std::mutex messages_mutex_;

  const std::size_t MIN_TRACKED_DETECTIONS = 2;

  bool tracked_detections_clustered(
    const std::string & class_name,
    double max_area_percentage) const;
  bool all_tracked_detections_found(
    const std::unordered_set<std::string> & tracked_detection_ids,
    const std::vector<DetectionBounds> & tracked_detection_bounds) const;
  bool bounding_box_area_within_limit(
    const std::vector<DetectionBounds> & tracked_detection_bounds,
    double image_area,
    double max_area_percentage) const;
  void detections_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void tracked_detection_ids_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg);
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__TRACKED_DETECTIONS_CLUSTERED_HPP
