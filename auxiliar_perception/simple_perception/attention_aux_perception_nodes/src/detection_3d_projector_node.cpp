#include <rclcpp/rclcpp.hpp>
#include "attention_aux_perception_nodes/Detection3dProjector.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<attention_aux_perception_nodes::Detection3dProjector>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}