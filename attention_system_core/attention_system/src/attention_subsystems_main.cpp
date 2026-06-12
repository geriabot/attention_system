#include "attention_system/AttentionOrchestrator.hpp"
#include "attention_system/intelligence/AttentionIntelligence.hpp"

#include "behavior_architecture/behavior_runner.hpp"
#include "behavior_architecture/orchestrator_factory.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto attention_intelligence = std::make_shared<attention_system::AttentionIntelligence>();

  exec.add_node(attention_intelligence->get_node_base_interface());

  while (rclcpp::ok()) {
    exec.spin_some();
  }

  rclcpp::shutdown();

  return 0;
}