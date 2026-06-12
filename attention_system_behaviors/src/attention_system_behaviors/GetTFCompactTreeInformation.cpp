#include "attention_system_behaviors/GetTFCompactTreeInformation.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <sstream>
#include <vector>

namespace attention_system_behaviors
{

namespace
{

struct TFTree
{
  std::map<std::string, std::vector<std::string>> children_by_parent;
  std::set<std::string> frames;
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

    tree.children_by_parent[parent_frame].push_back(child_frame);
    tree.frames.insert(parent_frame);
    tree.frames.insert(child_frame);
  }

  return tree;
}

void
append_frame_tree(
  const TFTree & tree,
  const std::string & frame,
  int depth,
  int max_depth,
  int max_frames,
  int & frame_count,
  std::ostringstream & output)
{
  if (frame_count >= max_frames) {
    return;
  }

  for (int i = 0; i < depth; i++) {
    output << "\t";
  }
  output << frame << "\n";
  frame_count++;

  if (depth >= max_depth) {
    const auto children_it = tree.children_by_parent.find(frame);
    if (children_it != tree.children_by_parent.end() &&
      !children_it->second.empty())
    {
      for (int i = 0; i <= depth; i++) {
        output << "\t";
      }
      output << "...\n";
    }
    return;
  }

  const auto children_it = tree.children_by_parent.find(frame);
  if (children_it == tree.children_by_parent.end()) {
    return;
  }

  std::vector<std::string> children = children_it->second;
  std::sort(children.begin(), children.end());

  for (const auto & child : children) {
    append_frame_tree(
      tree,
      child,
      depth + 1,
      max_depth,
      max_frames,
      frame_count,
      output);
  }
}

std::string
format_compact_tf_tree(
  const TFTree & tree,
  const std::string & root_frame,
  int max_depth,
  int max_frames)
{
  const std::string normalized_root = normalize_frame_id(root_frame);

  std::ostringstream output;
  output << "TF tree from " << normalized_root << ":\n";

  if (tree.frames.find(normalized_root) == tree.frames.end() &&
    tree.children_by_parent.find(normalized_root) == tree.children_by_parent.end())
  {
    output << normalized_root << "\n";
    output << "(No TF children available yet.)";
    return output.str();
  }

  int frame_count = 0;
  append_frame_tree(
    tree,
    normalized_root,
    0,
    max_depth,
    max_frames,
    frame_count,
    output);

  if (frame_count >= max_frames) {
    output << "...";
  }

  return output.str();
}

} // namespace

GetTFCompactTreeInformation::GetTFCompactTreeInformation(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError(
            "GetTFCompactTreeInformation: 'node' not found in blackboard");
  }

  node_ = node_any;
  tree_tick_pub_ = create_tree_tick_publisher(node_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  max_depth_ = 0;
  max_frames_ = 0;
  timeout_sec_ = 0.0;
}

BT::NodeStatus
GetTFCompactTreeInformation::onStart()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  if (!getInput("root_frame", root_frame_)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'root_frame' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("max_depth", max_depth_)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'max_depth' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("max_frames", max_frames_)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'max_frames' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("timeout_sec", timeout_sec_)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Input port 'timeout_sec' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  if (max_depth_ < 0 || max_frames_ <= 0 || timeout_sec_ < 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Inputs 'max_depth', 'max_frames' and 'timeout_sec' must be valid.");
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = node_->now();
  return build_and_set_output();
}

BT::NodeStatus
GetTFCompactTreeInformation::onRunning()
{
  publish_tree_tick(tree_tick_pub_, this->name());
  return build_and_set_output();
}

void
GetTFCompactTreeInformation::onHalted()
{
}

BT::NodeStatus
GetTFCompactTreeInformation::build_and_set_output()
{
  const std::string frames_string = tf_buffer_->allFramesAsString();
  const double elapsed_sec = (node_->now() - start_time_).seconds();

  if (frames_string.empty() && elapsed_sec < timeout_sec_) {
    return BT::NodeStatus::RUNNING;
  }

  const TFTree tree = build_tf_tree_from_string(frames_string);
  const std::string compact_tree = format_compact_tf_tree(
    tree,
    root_frame_,
    max_depth_,
    max_frames_);

  if (!setOutput("out_string", compact_tree)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Output port 'out_string' missing in XML.");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Generated compact TF tree information from root_frame=%s",
    root_frame_.c_str());

  return BT::NodeStatus::SUCCESS;
}

} // namespace attention_system_behaviors

#ifdef BUILD_INDIVIDUAL_PLUGIN
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system_behaviors::GetTFCompactTreeInformation>(
    "GetTFCompactTreeInformation");
}

#endif
