#include "rclcpp/rclcpp.hpp"
#include "controller.h"
#include "PID.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<reef_control::PIDController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
