#include "google_gemini_bridge_cpp/GeminiBridge.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gemini_testing::GeminiBridge>();

  if (!node->init_gemini_client_service()) {
    RCLCPP_ERROR(node->get_logger(), "No Gemini API key");
    return 0;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}