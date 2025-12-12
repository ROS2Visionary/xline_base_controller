// 曲线路径跟随策略的具体实现。
// 和圆形策略类似，这里只关心局部的路径信息和速度调节，
// 不直接操作上层控制器的状态机。

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

// 构造函数主要是把内部状态拉回一个“已知初始值”，
// 方便复用同一个对象多次计算不同路径。
CurvePathStrategy::CurvePathStrategy()
  : goal_x_(0.0)
  , goal_y_(0.0)
  , goal_theta_(0.0)
  , path_length_(0.0)
  , goal_reached_(false)
  , back_follow_(false)
{
  RCLCPP_INFO(getLogger(), "CurvePathStrategy 创建完成");
}

// ================================
// PathStrategy 接口实现
// ================================
bool CurvePathStrategy::setPlan(const nav_msgs::msg::Path& path)
{
  // 统一入口：从 RPP 控制器传入一条已经规划好的路径
  if (path.poses.empty())
  {
    RCLCPP_ERROR(getLogger(), "收到空路径，无法设置计划");
    return false;
  }

  reset();
  global_plan_ = path;
  calculatePathInfo();

  // 这里记录一下路径长度和终点坐标，方便日志排查问题
  RCLCPP_INFO(getLogger(), "曲线路径设置完成 - 点数: %zu, 长度: %.3fm, 目标: (%.3f, %.3f)",
              global_plan_.poses.size(), path_length_, goal_x_, goal_y_);

  return true;
}

bool CurvePathStrategy::isGoalReached(const PathStrategyContext& ctx)
{
  // 一旦标记为到达，就不再重复计算，避免日志刷屏
  if (goal_reached_)
  {
    return true;
  }

  double distance_to_goal = std::hypot(
      ctx.current_pose.pose.position.x - goal_x_,
      ctx.current_pose.pose.position.y - goal_y_);

  // 这里的判定比较简单：只看位置距离，不看角度。
  // 角度误差由上层控制器在最后阶段单独处理。
  if (distance_to_goal < params_.goal.dist_tol)
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
  // 曲线策略对角速度不做特别处理，直接沿用 RPP 计算出的基础角速度
  double desired_angular_velocity = ctx.base_angular_velocity;

  double distance_to_goal = std::hypot(
      ctx.current_pose.pose.position.x - goal_x_,
      ctx.current_pose.pose.position.y - goal_y_);

  // 线速度在接近终点时做一层“靠近约束”，
  // 避免车子在终点附近速度过高导致 overshoot。
  double adjusted_linear_velocity = applyApproachConstraint(
      ctx.desired_linear_velocity, distance_to_goal);

  result.angular_velocity = desired_angular_velocity;
  result.linear_velocity = adjusted_linear_velocity;
  result.goal_reached = goal_reached_;
  result.filter_reset = false;

  // 生成一条方便排查的状态字符串
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

  // 这里不重置 back_follow_，是否后退由外部显式控制
  RCLCPP_DEBUG(getLogger(), "CurvePathStrategy 状态已重置");
}

void CurvePathStrategy::updateParameters(const std::string& config_path)
{
  try
  {
    // 和其他模块统一，从 ament_index 中拿到安装目录，再拼接配置路径。
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("xline_follow_controller");
    std::string full_path = package_share_directory + config_path;

    xline::YamlParser::YamlParser parser(full_path);

    // 目标到达判定参数
    params_.goal.dist_tol = parser.getParameter<double>("goal.dist_tol");
    params_.goal.rotate_tol = parser.getParameter<double>("goal.rotate_tol");
    
    // 路径跟踪约束参数
    params_.tracking.approach.dist = parser.getParameter<double>("tracking.approach.dist");
    params_.tracking.approach.min_v = parser.getParameter<double>("tracking.approach.min_v");

    RCLCPP_INFO(getLogger(),
                "CurvePathStrategy 参数已更新: goal_tol=%.3fm, rotate_tol=%.3frad",
                params_.goal.dist_tol, params_.goal.rotate_tol);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(getLogger(), "加载曲线路径参数失败: %s，使用默认值", e.what());
  }
}

std::string CurvePathStrategy::getDebugInfo() const
{
  // 这里输出的是当前路径的一些“静态信息”，
  // 和实时误差/曲率无关，适合作为整体状态的一部分。
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

// 曲线路径特有接口
void CurvePathStrategy::setBackFollow(bool enable)
{
  back_follow_ = enable;
  // 这里仅记录模式切换，具体如何根据 back_follow_ 修正速度方向，
  // 由上层控制器或策略调用方决定。
  RCLCPP_INFO(getLogger(), "后退模式: %s", enable ? "启用" : "禁用");
}

// 私有方法
void CurvePathStrategy::calculatePathInfo()
{
  if (global_plan_.poses.empty())
  {
    return;
  }

  // 简单按相邻点累积距离，得到一条路径的总长度，
  // 后面主要用于调试和日志。
  path_length_ = 0.0;
  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i)
  {
    double dx = global_plan_.poses[i + 1].pose.position.x -
                global_plan_.poses[i].pose.position.x;
    double dy = global_plan_.poses[i + 1].pose.position.y -
                global_plan_.poses[i].pose.position.y;
    path_length_ += std::hypot(dx, dy);
  }

  goal_pose_ = global_plan_.poses.back();
  goal_x_ = goal_pose_.pose.position.x;
  goal_y_ = goal_pose_.pose.position.y;
  // 记录终点的朝向，方便做一些调试输出或后续扩展
  goal_theta_ = tf2::getYaw(goal_pose_.pose.orientation);
}

double CurvePathStrategy::applyApproachConstraint(double raw_velocity,
                                                   double distance_to_goal)
{
  // 很典型的一种“靠近目标减速”策略：
  // - 距离大于阈值：直接用原始速度
  // - 距离小于阈值：按比例缩小速度，但不低于一个安全下限
  if (distance_to_goal < params_.tracking.approach.dist)
  {
    double ratio = distance_to_goal / params_.tracking.approach.dist;
    return std::max(params_.tracking.approach.min_v, raw_velocity * ratio);
  }
  return raw_velocity;
}

}  // namespace follow_controller
}  // namespace xline
