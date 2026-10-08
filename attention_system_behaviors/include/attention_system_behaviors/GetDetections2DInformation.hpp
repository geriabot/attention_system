#ifndef ATTENTION_SYSTEM_BEHAVIORS__GET_DETECTIONS_2D_INFORMATION_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__GET_DETECTIONS_2D_INFORMATION_HPP

#include <chrono>
#include <mutex>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace attention_system_behaviors
{

class GetDetections2DInformation : public BT::StatefulActionNode
{
public:
  explicit GetDetections2DInformation(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<std::string>("out_string"),
        BT::InputPort<std::string>(
          "class", "",
          "Only report detections of this class (empty: any class)"),
        BT::InputPort<bool>(
          "require_id", false,
          "Only report detections that already have a tracker id"),
        BT::InputPort<int>(
          "timeout_ms", 5000,
          "Maximum time to wait for detections before failing")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
    detection_sub_;

  // Written by the subscription (executor thread) and read by the tree tick
  std::mutex detections_mutex_;
  vision_msgs::msg::Detection2DArray::SharedPtr last_detections_msg_;

  std::string class_filter_;
  bool require_id_{false};
  std::chrono::milliseconds timeout_{0};
  std::chrono::steady_clock::time_point start_time_;

  BT::NodeStatus check_detections();
  std::string build_detections_information(
    const vision_msgs::msg::Detection2DArray & detections_msg,
    size_t & valid_detection_count) const;
  void detections_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  const int detection_qos_depth_ = 10;
  const std::string DETECTION_TOPIC = "/detections_2d";
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__GET_DETECTIONS_2D_INFORMATION_HPP
