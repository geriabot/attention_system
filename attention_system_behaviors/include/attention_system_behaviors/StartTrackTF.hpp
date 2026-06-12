#ifndef ATTENTION_SYSTEM_BEHAVIORS__START_TRACK_TF_HPP
#define ATTENTION_SYSTEM_BEHAVIORS__START_TRACK_TF_HPP

#include <memory>

#include "behaviortree_cpp/behavior_tree.h"

#include "attention_actuation_msgs/srv/start_tracking.hpp"
#include "rclcpp/rclcpp.hpp"
#include "attention_system_behaviors/TreeTickTrace.hpp"
#include "std_msgs/msg/string.hpp"

namespace attention_system_behaviors
{

class StartTrackTF : public BT::StatefulActionNode
{
  using StartTracking = attention_actuation_msgs::srv::StartTracking;

public:
  explicit StartTrackTF(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf
  );

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() {};

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("frame_id")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  TreeTickPublisher tree_tick_pub_;
  rclcpp::Client<StartTracking>::SharedPtr track_tf_cli_;
  std::shared_future<StartTracking::Response::SharedPtr> response_future_;
};

} // namespace attention_system_behaviors

#endif // ATTENTION_SYSTEM_BEHAVIORS__START_TRACK_TF_HPP
