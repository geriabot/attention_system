#include <memory>
#include <string>
#include <tuple>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "rclcpp/rclcpp.hpp"

int
main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("track_joint_art_test_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("attention_system_behaviors_bt_plugins"));

  const std::string pkgpath =
    ament_index_cpp::get_package_share_directory("attention_system_behaviors");
  const std::string xml_file = pkgpath + "/behavior_tree_xml/track_joint_art.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<int>("retry_attempts", 5);
  blackboard->set<std::string>("robot_reference_frame", "base_link");
  blackboard->set<std::string>("camera_control_joint_frame", "Head");
  blackboard->set<std::string>("joint_frame", "LShoulderPitch");
  blackboard->set<std::string>("req_task_details", "TrackJointArt task test");
  blackboard->set<std::string>("req_behavior_details", "TrackJointArt test");
  blackboard->set<std::string>("req_action_explanation", "TrackJointArt test");
  blackboard->set<std::string>("req_inputs", "<string>joint_frame");
  blackboard->set<std::string>("req_blackboard_inputs", "joint_frame");

  std::ignore = factory.createTreeFromFile(xml_file, blackboard);

  RCLCPP_INFO(
    node->get_logger(),
    "TrackJointArt XML loaded correctly.");

  rclcpp::shutdown();
  return 0;
}
