#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace xline {
namespace follow_controller {

// Minimal logger helper that ignores formatting args in format-style strings.
// It avoids daosnrs_common/logger.h while keeping call sites unchanged.
struct LoggerHelper {
  template <typename... Args>
  static void info(const char* fmt, Args&&... /*args*/) {
    RCLCPP_INFO(rclcpp::get_logger("follow_controller_tc"), "%s", fmt);
  }

  template <typename... Args>
  static void warn(const char* fmt, Args&&... /*args*/) {
    RCLCPP_WARN(rclcpp::get_logger("follow_controller_tc"), "%s", fmt);
  }

  template <typename... Args>
  static void error(const char* fmt, Args&&... /*args*/) {
    RCLCPP_ERROR(rclcpp::get_logger("follow_controller_tc"), "%s", fmt);
  }
};

}  // namespace follow_controller
}  // namespace xline

#define LOG_INFO(...)  ::xline::follow_controller::LoggerHelper::info(__VA_ARGS__)
#define LOG_WARN(...)  ::xline::follow_controller::LoggerHelper::warn(__VA_ARGS__)
#define LOG_ERROR(...) ::xline::follow_controller::LoggerHelper::error(__VA_ARGS__)

