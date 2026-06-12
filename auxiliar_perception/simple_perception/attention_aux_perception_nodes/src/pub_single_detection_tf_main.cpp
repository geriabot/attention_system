#include <rclcpp/rclcpp.hpp>
#include "attention_aux_perception_nodes/PubSingleDetectionTF.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<attention_aux_perception_nodes::PubSingleDetectionTF>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}