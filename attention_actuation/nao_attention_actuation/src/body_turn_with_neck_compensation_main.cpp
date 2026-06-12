#include <memory>

#include "nao_attention_actuation/BodyTurnWithNeckCompensation.hpp"
#include "rclcpp/rclcpp.hpp"

int
main(
  int argc,
  char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nao_attention_actuation::BodyTurnWithNeckCompensation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
