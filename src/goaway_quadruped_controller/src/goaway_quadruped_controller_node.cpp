#include "goaway_quadruped_controller/quadruped_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<goaway_quadruped_controller::QuadrupedController>();
  // Multi-threaded: services wait for API replies while the cmd_vel mux keeps running.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
