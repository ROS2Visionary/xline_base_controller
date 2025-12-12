/**
 * @file curve_path_strategy.cpp
 * @brief 曲线路径跟随策略实现
 */

#include "xline_follow_controller/rpp_follow/curve_path_strategy.hpp"
#include "xline_follow_controller/common/yaml_parser.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace xline
{
namespace follow_controller
{

// ============================================================================
// 构造函数
// ============================================================================

CurvePathStrategy::CurvePathStrategy()
  : goal_dist_tol_(0.02)
  , rotate_tol_(0.035)
  , approach_dist_(0.05)
  , approach_min_v_(0.07)
  , goal_x_(0.0)
  , goal_y_(0.0)
  , goal_theta_(0.0)
  , path_length_(0.0)
  , goal_reached_(false)
  , back_follow_(false)
{
  RCLCPP_INFO(getLogger(), "CurvePathStrategy 创建完成");
}

// ============================================================================
// PathStrategy 接口实现
// ============================================================================

bool CurvePathStrategy::setPlan(const nav_msgs::msg::Path& path)
{
  if (path.poses.empty())
  {
    RCLCPP_ERROR(getLogger(), "收到空路径，无法设置计划");
    return false;
  }

  // 重置状态
  reset();

  // 保存路径
  global_plan_ = path;

  // 计算路径信息
  calculatePathInfo();

  RCLCPP_INFO(getLogger(), "曲线路径设置完成 - 点数: %zu, 长度: %.3fm, 目标: (%.3f, %.3f)",
              global_plan_.poses.size(), path_length_, goal_x_, goal_y_);

  return true;
}

bool CurvePathStrategy::isGoalReached(const PathStrategyContext& ctx)
{
  if (goal_reached_)
  {
    return true;
  }

  // 计算到目标的距离
  double distance_to_goal = std::hypot(
      ctx.current_pose.pose.position.x - goal_x_,
      ctx.current_pose.pose.position.y - goal_y_);

  // 判断是否到达
  if (distance_to_goal < goal_dist_tol_)
  {
    goal_reached_ = true;
    RCLCPP_INFO(getLogger(), "曲线路径目标已达到 - 最终距离误差: %.4fm", distance_to_goal);
    return true;
  }

  return false;
}

void CurvePathStrategy::computeAngularVelocity(const PathStrategyContext& ctx,
                                                PathStrategyResult& result)
{
  // 标准 Pure Pursuit 角速度计算
  // w = v * curvature = v * (2 * sin(alpha) / L)
  double desired_angular_velocity = ctx.base_angular_velocity;

  // 应用接近目标约束
  double distance_to_goal = std::hypot(
      ctx.current_pose.pose.position.x - goal_x_,
      ctx.current_pose.pose.position.y - goal_y_);

  double adjusted_linear_velocity = applyApproachConstraint(
      ctx.desired_linear_velocity, distance_to_goal);

  // 更新结果
  result.angular_velocity = desired_angular_velocity;
  result.linear_velocity = adjusted_linear_velocity;
  result.goal_reached = goal_reached_;
  result.filter_reset = false;

  // 生成状态信息
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "Curve: dist=" << distance_to_goal
      << "m, v=" << adjusted_linear_velocity
      << ", w=" << desired_angular_velocity;
  result.status_message = oss.str();
}

void CurvePathStrategy::reset()
{
  global_plan_.poses.clear();
  goal_x_ = 0.0;
  goal_y_ = 0.0;
  goal_theta_ = 0.0;
  path_length_ = 0.0;
  goal_reached_ = false;

  RCLCPP_DEBUG(getLogger(), "CurvePathStrategy 状态已重置");
}

void CurvePathStrategy::updateParameters(const std::string& config_path)
{
  try
  {
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("xline_follow_controller");
    std::string full_path = package_share_directory + config_path;

    xline::YamlParser::YamlParser parser(full_path);

    // 读取参数
    goal_dist_tol_ = parser.getParameter<double>("goal_dist_tol");
    rotate_tol_ = parser.getParameter<double>("rotate_tol");
    approach_dist_ = parser.getParameter<double>("approach_dist");
    approach_min_v_ = parser.getParameter<double>("approach_min_v");

    RCLCPP_INFO(getLogger(),
                "CurvePathStrategy 参数已更新: goal_tol=%.3fm, rotate_tol=%.3frad",
                goal_dist_tol_, rotate_tol_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(getLogger(), "加载曲线路径参数失败: %s，使用默认值", e.what());
  }
}

std::string CurvePathStrategy::getDebugInfo() const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "CurvePathStrategy["
      << " path_len=" << path_length_
      << ", goal=(" << goal_x_ << "," << goal_y_ << "," << goal_theta_ << ")"
      << ", reached=" << (goal_reached_ ? "true" : "false")
      << ", back=" << (back_follow_ ? "true" : "false")
      << "]";
  return oss.str();
}

// ============================================================================
// 曲线路径特有接口
// ============================================================================

void CurvePathStrategy::setBackFollow(bool enable)
{
  back_follow_ = enable;
  RCLCPP_INFO(getLogger(), "后退模式: %s", enable ? "启用" : "禁用");
}

// ============================================================================
// 私有方法
// ============================================================================

void CurvePathStrategy::calculatePathInfo()
{
  if (global_plan_.poses.empty())
  {
    return;
  }

  // 计算路径长度
  path_length_ = 0.0;
  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i)
  {
    double dx = global_plan_.poses[i + 1].pose.position.x -
                global_plan_.poses[i].pose.position.x;
    double dy = global_plan_.poses[i + 1].pose.position.y -
                global_plan_.poses[i].pose.position.y;
    path_length_ += std::hypot(dx, dy);
  }

  // 提取目标信息
  goal_pose_ = global_plan_.poses.back();
  goal_x_ = goal_pose_.pose.position.x;
  goal_y_ = goal_pose_.pose.position.y;
  goal_theta_ = tf2::getYaw(goal_pose_.pose.orientation);
}

double CurvePathStrategy::applyApproachConstraint(double raw_velocity,
                                                   double distance_to_goal)
{
  if (distance_to_goal < approach_dist_)
  {
    // 线性降速
    double ratio = distance_to_goal / approach_dist_;
    return std::max(approach_min_v_, raw_velocity * ratio);
  }
  return raw_velocity;
}

}  // namespace follow_controller
}  // namespace xline
