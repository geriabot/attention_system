#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nao_attention_actuation/TrackTFWithNeck.hpp"

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<nao_attention_actuation::TrackTFWithNeck>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
