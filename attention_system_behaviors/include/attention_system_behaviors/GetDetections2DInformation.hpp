#ifndef ATTENTION_SYSTEM_BEHAVIORS__GET_DETECTIONS_2D_INFORMATION_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__GET_DETECTIONS_2D_INFORMATION_HPP

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
        BT::OutputPort<std::string>("out_string")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
    detection_sub_;

  vision_msgs::msg::Detection2DArray::SharedPtr last_detections_msg_;

  std::string build_detections_information() const;
  void detections_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  const int detection_qos_depth_ = 10;
  const std::string DETECTION_TOPIC = "/detections_2d";
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__GET_DETECTIONS_2D_INFORMATION_HPP
