#include <rclcpp/rclcpp.hpp>
#include "xline_localization/localization_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xline::localization::LocalizationNode>(rclcpp::NodeOptions());

  // 使用多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
