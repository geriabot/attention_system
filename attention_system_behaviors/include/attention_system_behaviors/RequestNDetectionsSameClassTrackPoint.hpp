#ifndef ATTENTION_SYSTEM_BEHAVIORS__REQUEST_N_DETECTIONS_SAME_CLASS_TRACK_POINT_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__REQUEST_N_DETECTIONS_SAME_CLASS_TRACK_POINT_HPP

#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"

#include "attention_aux_perception_msgs/srv/start_n_detections_same_class_tf_pub.hpp"

namespace attention_system_behaviors
{

class RequestNDetectionsSameClassTrackPoint : public BT::StatefulActionNode
{
public:
  explicit RequestNDetectionsSameClassTrackPoint(
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
        BT::InputPort<std::string>("class"),
        BT::InputPort<int>("N"),
        BT::InputPort<std::string>("frame_id")
      });
  }

private:
  void send_request();
  void handle_response(
    rclcpp::Client<
      attention_aux_perception_msgs::srv::StartNDetectionsSameClassTFPub>::SharedFuture future);

  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;

  int n_detections_;
  std::string frame_id_;
  std::string detection_class_name_;

  rclcpp::Client<attention_aux_perception_msgs::srv::StartNDetectionsSameClassTFPub>::SharedPtr client_;
  bool request_done_ {false};
  bool request_success_ {false};
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__REQUEST_N_DETECTIONS_SAME_CLASS_TRACK_POINT_HPP
