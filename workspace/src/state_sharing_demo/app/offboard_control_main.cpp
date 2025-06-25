#include "state_sharing_demo/offboard_control.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<state_sharing_demo::OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
