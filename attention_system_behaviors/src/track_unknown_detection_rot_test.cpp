#include <string>
#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

int
main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("track_unknown_detection_rot_test_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("request_behavior_input_intelligence_bt_node"));
  factory.registerFromPlugin(loader.getOSName("change_visual_detection_class_bt_node"));
  factory.registerFromPlugin(loader.getOSName("clear_visual_detection_class_bt_node"));
  factory.registerFromPlugin(loader.getOSName("detection_exists_bt_node"));
  factory.registerFromPlugin(loader.getOSName("start_track_tf_bt_node"));
  factory.registerFromPlugin(loader.getOSName("genetate_single_detection_track_point_bt_node"));
  factory.registerFromPlugin(loader.getOSName("trigger_deactivation_service_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("attention_system_behaviors");
  std::string xml_file = pkgpath + "/behavior_tree_xml/track_unknown_detection_rot.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);

  blackboard->set<int>("retry_attempts", 5);
  blackboard->set<std::string>("det_class", "Person");
  blackboard->set<int>("det_id", 1);
  blackboard->set<std::string>("det_prompt_topic", "/omdet_prompt");
  blackboard->set<std::string>("det_frame_id", "attention_point_1");

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.tickExactlyOnce() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
