#ifndef ATTENTION_SYSTEM_BEHAVIORS__GET_TF_COMPACT_TREE_INFORMATION_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__GET_TF_COMPACT_TREE_INFORMATION_HPP

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace attention_system_behaviors
{

class GetTFCompactTreeInformation : public BT::StatefulActionNode
{
public:
  explicit GetTFCompactTreeInformation(
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
        BT::InputPort<std::string>("root_frame"),
        BT::InputPort<int>("max_depth"),
        BT::InputPort<int>("max_frames"),
        BT::InputPort<double>("timeout_sec"),
        BT::OutputPort<std::string>("out_string")
      });
  }

private:
  BT::NodeStatus build_and_set_output();

  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string root_frame_;
  int max_depth_;
  int max_frames_;
  double timeout_sec_;
  rclcpp::Time start_time_;
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__GET_TF_COMPACT_TREE_INFORMATION_HPP
