#include "attention_system_behaviors/JointTrackingKinematicallyFeasible.hpp"

#include <map>
#include <set>
#include <sstream>

namespace attention_system_behaviors
{

namespace
{

struct TFTree
{
  std::map<std::string, std::string> parent_by_child;
};

std::string
trim(
  const std::string & value)
{
  const size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }

  const size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string
normalize_frame_id(
  const std::string & frame_id)
{
  std::string normalized = trim(frame_id);

  while (!normalized.empty() && normalized.front() == '/') {
    normalized.erase(normalized.begin());
  }

  return normalized;
}

TFTree
build_tf_tree_from_string(
  const std::string & frames_string)
{
  TFTree tree;
  std::istringstream stream(frames_string);
  std::string line;

  while (std::getline(stream, line)) {
    const std::string trimmed_line = trim(line);
    const std::string frame_prefix = "Frame ";
    const std::string parent_separator = " exists with parent ";

    if (trimmed_line.rfind(frame_prefix, 0) != 0) {
      continue;
    }

    const size_t parent_separator_pos =
      trimmed_line.find(parent_separator, frame_prefix.size());
    if (parent_separator_pos == std::string::npos) {
      continue;
    }

    const std::string child_frame = normalize_frame_id(
      trimmed_line.substr(
        frame_prefix.size(),
        parent_separator_pos - frame_prefix.size()));

    std::string parent_frame = normalize_frame_id(
      trimmed_line.substr(parent_separator_pos + parent_separator.size()));
    if (!parent_frame.empty() && parent_frame.back() == '.') {
      parent_frame.pop_back();
    }

    if (child_frame.empty() || parent_frame.empty()) {
      continue;
    }

    tree.parent_by_child[child_frame] = parent_frame;
  }

  return tree;
}

bool
is_frame_reachable_from_root(
  const TFTree & tree,
  const std::string & root_frame,
  const std::string & target_frame)
{
  const std::string normalized_root = normalize_frame_id(root_frame);
  std::string current_frame = normalize_frame_id(target_frame);
  std::set<std::string> visited_frames;

  while (!current_frame.empty()) {
    if (current_frame == normalized_root) {
      return true;
    }

    if (visited_frames.find(current_frame) != visited_frames.end()) {
      return false;
    }
    visited_frames.insert(current_frame);

    const auto parent_it = tree.parent_by_child.find(current_frame);
    if (parent_it == tree.parent_by_child.end()) {
      return false;
    }

    current_frame = parent_it->second;
  }

  return false;
}

bool
is_frame_descendant(
  const TFTree & tree,
  const std::string & ancestor_frame,
  const std::string & target_frame)
{
  const std::string normalized_ancestor = normalize_frame_id(ancestor_frame);
  std::string current_frame = normalize_frame_id(target_frame);
  std::set<std::string> visited_frames;

  while (!current_frame.empty()) {
    if (current_frame == normalized_ancestor) {
      return true;
    }

    if (visited_frames.find(current_frame) != visited_frames.end()) {
      return false;
    }
    visited_frames.insert(current_frame);

    const auto parent_it = tree.parent_by_child.find(current_frame);
    if (parent_it == tree.parent_by_child.end()) {
      return false;
    }

    current_frame = parent_it->second;
  }

  return false;
}

} // namespace

JointTrackingKinematicallyFeasible::JointTrackingKinematicallyFeasible(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError(
            "JointTrackingKinematicallyFeasible: 'node' not found in blackboard");
  }

  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus
JointTrackingKinematicallyFeasible::tick()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  std::string root_frame;
  if (!getInput("root_frame", root_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'root_frame' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  std::string camera_control_joint_frame;
  if (!getInput("camera_control_joint_frame", camera_control_joint_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'camera_control_joint_frame' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  std::string target_joint_frame;
  if (!getInput("target_joint_frame", target_joint_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'target_joint_frame' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  const TFTree tree = build_tf_tree_from_string(tf_buffer_->allFramesAsString());

  if (!is_frame_reachable_from_root(tree, root_frame, camera_control_joint_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Camera control joint frame '%s' is not reachable from root frame '%s'.",
      camera_control_joint_frame.c_str(),
      root_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!is_frame_reachable_from_root(tree, root_frame, target_joint_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Target joint frame '%s' is not reachable from root frame '%s'.",
      target_joint_frame.c_str(),
      root_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (is_frame_descendant(tree, camera_control_joint_frame, target_joint_frame)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Joint tracking is not kinematically feasible: target '%s' depends on camera control joint '%s'.",
      target_joint_frame.c_str(),
      camera_control_joint_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Joint tracking is kinematically feasible: camera_control_joint_frame=%s, target_joint_frame=%s",
    camera_control_joint_frame.c_str(),
    target_joint_frame.c_str());

  return BT::NodeStatus::SUCCESS;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::JointTrackingKinematicallyFeasible>(
    "JointTrackingKinematicallyFeasible");
}

#endif
