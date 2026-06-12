#ifndef ATTENTION_SYSTEM_BEHAVIORS__PUBLISH_ROBOT_OUT_OF_BOUNDS_REFERENCE_TF_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__PUBLISH_ROBOT_OUT_OF_BOUNDS_REFERENCE_TF_HPP

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace attention_system_behaviors
{

class PublishRobotOutOfBoundsReferenceTF : public BT::StatefulActionNode
{
public:
  explicit PublishRobotOutOfBoundsReferenceTF(
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
        BT::InputPort<std::string>("robot_reference_frame"),
        BT::InputPort<std::string>("fixed_reference_frame")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__PUBLISH_ROBOT_OUT_OF_BOUNDS_REFERENCE_TF_HPP
