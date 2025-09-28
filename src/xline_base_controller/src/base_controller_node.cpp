#include <rclcpp/rclcpp.hpp>
#include "xline_base_controller/motion_control_center.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xline::base_controller::MotionControlCenter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
