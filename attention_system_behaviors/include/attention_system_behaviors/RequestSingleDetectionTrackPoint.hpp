#ifndef ATTENTION_SYSTEM_BEHAVIORS__GENERATE_SINGLE_DETECTION_TRACK_POINT_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__GENERATE_SINGLE_DETECTION_TRACK_POINT_HPP

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "std_msgs/msg/string.hpp"

#include "attention_aux_perception_msgs/srv/start_single_detection_tf_pub.hpp"
#include "attention_aux_perception_msgs/srv/stop_single_detection_tf_pub.hpp"

namespace attention_system_behaviors
{

class RequestSingleDetectionTrackPoint : public BT::StatefulActionNode
{
public:
  explicit RequestSingleDetectionTrackPoint(
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
        BT::InputPort<int>("id"),
        BT::InputPort<std::string>("frame_id")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;

  std::string actual_detection_id_;
  std::string frame_id_;
  std::string detection_class_id_;

  rclcpp::Client<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub>::SharedPtr client_;
  std::shared_future<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub::Response::SharedPtr> response_future_;

  std::shared_future<attention_aux_perception_msgs::srv::StartSingleDetectionTFPub::Response::SharedPtr> make_request();

  bool check_input_and_request();
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__GENERATE_SINGLE_DETECTION_TRACK_POINT_HPP