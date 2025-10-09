#pragma once

#include <rclcpp/rclcpp.hpp>

namespace xline {
namespace follow_controller {

// 日志辅助宏 - 直接使用 RCLCPP 的格式化功能（printf 风格）
// 使用方法：LOG_INFO("值: %.2f", value) 而不是 LOG_INFO("值: {:.2f}", value)

}  // namespace follow_controller
}  // namespace xline

// 直接映射到 RCLCPP 日志宏，支持 printf 风格格式化
#define LOG_INFO(...)  RCLCPP_INFO(rclcpp::get_logger("follow_controller"), __VA_ARGS__)
#define LOG_WARN(...)  RCLCPP_WARN(rclcpp::get_logger("follow_controller"), __VA_ARGS__)
#define LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("follow_controller"), __VA_ARGS__)

