#ifndef ATTENTION_SYSTEM_BEHAVIORS__TF_EXISTS_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__TF_EXISTS_HPP

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace attention_system_behaviors
{

class TFExists : public BT::ConditionNode
{
public:
  explicit TFExists(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("target_frame"),
        BT::InputPort<std::string>("source_frame"),
        BT::InputPort<bool>("expected_exists", true, "Expected TF existence"),
        BT::InputPort<double>("timeout_sec", 0.0, "TF lookup timeout in seconds")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__TF_EXISTS_HPP
