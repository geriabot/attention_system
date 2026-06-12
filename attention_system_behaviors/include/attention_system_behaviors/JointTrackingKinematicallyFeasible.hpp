#ifndef ATTENTION_SYSTEM_BEHAVIORS__JOINT_TRACKING_KINEMATICALLY_FEASIBLE_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__JOINT_TRACKING_KINEMATICALLY_FEASIBLE_HPP

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace attention_system_behaviors
{

class JointTrackingKinematicallyFeasible : public BT::ConditionNode
{
public:
  explicit JointTrackingKinematicallyFeasible(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("root_frame"),
        BT::InputPort<std::string>("camera_control_joint_frame"),
        BT::InputPort<std::string>("target_joint_frame")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__JOINT_TRACKING_KINEMATICALLY_FEASIBLE_HPP
