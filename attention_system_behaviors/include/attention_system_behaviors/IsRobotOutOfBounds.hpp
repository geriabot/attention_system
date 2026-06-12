#ifndef ATTENTION_SYSTEM_BEHAVIORS__IS_ROBOT_OUT_OF_BOUNDS_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__IS_ROBOT_OUT_OF_BOUNDS_HPP

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace attention_system_behaviors
{

class IsRobotOutOfBounds : public BT::ConditionNode
{
public:
  static constexpr const char * initial_reference_frame_ = "robot_out_of_bounds_origin";

  explicit IsRobotOutOfBounds(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("robot_reference_frame"),
        BT::InputPort<std::string>("fixed_reference_frame"),
        BT::InputPort<double>("move_back_limit_dist")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__IS_ROBOT_OUT_OF_BOUNDS_HPP
