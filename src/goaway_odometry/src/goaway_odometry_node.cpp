#include "goaway_odometry/goaway_odometry.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<goaway_odometry::GoawayOdometry>());
  rclcpp::shutdown();
  return 0;
}
