#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "kobuki_attention_actuation/TrackTFStatic.hpp"

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<kobuki_attention_actuation::TrackTFStatic>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
