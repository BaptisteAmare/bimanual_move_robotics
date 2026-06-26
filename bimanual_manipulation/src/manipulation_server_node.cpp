#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "bimanual_manipulation/manipulation_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bimanual_manipulation::ManipulationServer>();
  if (!node->initialize()) {
    RCLCPP_FATAL(node->get_logger(), "Initialization failed, shutting down.");
    rclcpp::shutdown();
    return 1;
  }
  // Multi-threaded so controller action results are processed while the
  // action-execution threads block waiting for them.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
