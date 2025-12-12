/**
 * @file circle_path_strategy.cpp
 * @brief 圆形路径跟随策略实现
 */

#include "xline_follow_controller/rpp_follow/circle_path_strategy.hpp"
#include "xline_follow_controller/common/yaml_parser.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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

CirclePathStrategy::CirclePathStrategy()
  // 圆形路径参数
  : circle_center_x_(0.0)
  , circle_center_y_(0.0)
  , circle_radius_(0.0)
  , circle_entry_x_(0.0)
  , circle_entry_y_(0.0)
  , circle_start_angle_(0.0)
  , circle_end_angle_(0.0)
  , circle_total_angle_(0.0)
  , baseline_angular_velocity_(0.0)
  // 角度累计相关
  , last_yaw_initialized_(false)
  , last_yaw_(0.0)
  , accumulated_angle_(0.0)
  , angle_debug_counter_(0)
  // 航向预对准
  , need_yaw_prealign_(true)
  , yaw_prealign_done_(false)
  , target_yaw_(0.0)
  // 偏差参数
  , start_deviation_factor_(0.5)
  , end_deviation_factor_(0.5)
  , deviation_rate_(0.1)
  // 其他参数
  , goal_dist_tol_(0.02)
  , rotate_tol_(0.035)
  , radius_offset_(0.0)
  // 状态
  , goal_reached_(false)
  , start_print_(false)
  , stop_print_(false)
{
  RCLCPP_INFO(getLogger(), "CirclePathStrategy 创建完成");
}

// ============================================================================
// PathStrategy 接口实现
// ============================================================================

bool CirclePathStrategy::setPlan(const nav_msgs::msg::Path& path)
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

  RCLCPP_INFO(getLogger(), "圆形路径设置完成 - 点数: %zu, 总角度: %.2f rad (%.1f°)",
              global_plan_.poses.size(), circle_total_angle_,
              circle_total_angle_ * 180.0 / M_PI);

  return true;
}

bool CirclePathStrategy::isGoalReached(const PathStrategyContext& ctx)
{
  if (goal_reached_)
  {
    return true;
  }

  // 使用累计角度判定
  double current_yaw = ctx.current_yaw;
  bool completed = updateAccumulatedAngle(current_yaw);

  if (completed)
  {
    goal_reached_ = true;
    start_print_ = false;
    stop_print_ = true;

    RCLCPP_INFO(getLogger(), "圆形路径完成 - 累计角度: %.4f rad (%.2f°)",
                accumulated_angle_, accumulated_angle_ * 180.0 / M_PI);
  }

  return goal_reached_;
}

void CirclePathStrategy::computeAngularVelocity(const PathStrategyContext& ctx,
                                                 PathStrategyResult& result)
{
  // 基础 Pure Pursuit 角速度
  double pp_angular_velocity = ctx.base_angular_velocity;
  double desired_angular_velocity = pp_angular_velocity;

  // 是否需要重置滤波器
  bool filter_reset = false;

  // 根据累计角度阶段调整角速度
  if (accumulated_angle_ >= ((start_deviation_factor_ - 0.1) * M_PI))
  {
    // 进入正常跟踪阶段，应用基于基准角速度的偏差约束
    desired_angular_velocity = constrainAngularVelocity(pp_angular_velocity);
    filter_reset = false;
  }
  else
  {
    // 起始过渡阶段，不约束角速度
    desired_angular_velocity = pp_angular_velocity;
    filter_reset = true;
  }

  // 更新结果
  result.angular_velocity = desired_angular_velocity;
  result.linear_velocity = ctx.min_v;  // 圆形路径使用固定低速
  result.goal_reached = goal_reached_;
  result.filter_reset = filter_reset;

  // 生成状态信息
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "Circle: acc_angle=" << accumulated_angle_ << "/" << circle_total_angle_
      << ", base_w=" << baseline_angular_velocity_
      << ", w=" << desired_angular_velocity;
  result.status_message = oss.str();
}

void CirclePathStrategy::reset()
{
  // 重置路径
  global_plan_.poses.clear();

  // 重置目标状态
  goal_reached_ = false;

  // 重置圆形路径状态
  resetCirclePathState();

  // 重置预对准
  need_yaw_prealign_ = true;
  yaw_prealign_done_ = false;

  // 重置打印标志
  start_print_ = false;
  stop_print_ = false;

  RCLCPP_DEBUG(getLogger(), "CirclePathStrategy 状态已重置");
}

void CirclePathStrategy::updateParameters(const std::string& config_path)
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
    radius_offset_ = parser.getParameter<double>("radius_offset");
    start_deviation_factor_ = parser.getParameter<double>("start_deviation_factor");
    end_deviation_factor_ = parser.getParameter<double>("end_deviation_factor");
    deviation_rate_ = parser.getParameter<double>("deviation_rate");

    RCLCPP_INFO(getLogger(),
                "CirclePathStrategy 参数已更新: deviation_rate=%.2f, radius_offset=%.3f",
                deviation_rate_, radius_offset_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(getLogger(), "加载圆形路径参数失败: %s，使用默认值", e.what());
  }
}

void CirclePathStrategy::setYawPrealignmentDone()
{
  yaw_prealign_done_ = true;
  need_yaw_prealign_ = false;
  RCLCPP_INFO(getLogger(), "航向预对准完成");
}

std::string CirclePathStrategy::getDebugInfo() const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "CirclePathStrategy["
      << " center=(" << circle_center_x_ << "," << circle_center_y_ << ")"
      << ", radius=" << circle_radius_
      << ", total_angle=" << circle_total_angle_
      << ", acc_angle=" << accumulated_angle_
      << ", baseline_w=" << baseline_angular_velocity_
      << ", reached=" << (goal_reached_ ? "true" : "false")
      << ", prealign=" << (yaw_prealign_done_ ? "done" : "pending")
      << "]";
  return oss.str();
}

// ============================================================================
// 圆形路径特有接口
// ============================================================================

nav_msgs::msg::Path CirclePathStrategy::generateCirclePath(
    double center_x, double center_y, double radius,
    const geometry_msgs::msg::PoseStamped& start_pose)
{
  nav_msgs::msg::Path circle_path;
  circle_path.header.frame_id = "world";
  circle_path.header.stamp = rclcpp::Clock().now();

  // 应用半径偏移
  double actual_radius = radius + radius_offset_;

  // 使用传入位姿作为切入点
  geometry_msgs::msg::PoseStamped entry_pose = start_pose;
  circle_entry_x_ = entry_pose.pose.position.x;
  circle_entry_y_ = entry_pose.pose.position.y;

  // 计算切入点处的切线方向
  double tangent_x = -(entry_pose.pose.position.y - center_y);
  double tangent_y = (entry_pose.pose.position.x - center_x);
  double tangent_length = std::hypot(tangent_x, tangent_y);

  if (tangent_length > 1e-6)
  {
    tangent_x /= tangent_length;
    tangent_y /= tangent_length;
  }

  double entry_yaw = std::atan2(tangent_y, tangent_x);
  target_yaw_ = entry_yaw;

  tf2::Quaternion entry_q;
  entry_q.setRPY(0, 0, entry_yaw);
  entry_pose.pose.orientation = tf2::toMsg(entry_q);

  // 添加切入点
  circle_path.poses.push_back(entry_pose);

  // 生成圆周路径点
  int num_circle_points = 1500;
  double start_angle = std::atan2(
      entry_pose.pose.position.y - center_y,
      entry_pose.pose.position.x - center_x);

  for (int i = 20; i <= num_circle_points; ++i)
  {
    double angle = start_angle + i * (circle_total_angle_ / num_circle_points);

    geometry_msgs::msg::PoseStamped circle_pose;
    circle_pose.header = entry_pose.header;
    circle_pose.pose.position.x = center_x + actual_radius * std::cos(angle);
    circle_pose.pose.position.y = center_y + actual_radius * std::sin(angle);
    circle_pose.pose.position.z = start_pose.pose.position.z;

    // 设置切线方向作为朝向
    double tangent_direction = angle + M_PI / 2;
    tf2::Quaternion circle_q;
    circle_q.setRPY(0, 0, tangent_direction);
    circle_pose.pose.orientation = tf2::toMsg(circle_q);

    circle_path.poses.push_back(circle_pose);
  }

  RCLCPP_INFO(getLogger(), "生成圆形路径: center=(%.3f,%.3f), radius=%.3f, points=%zu",
              center_x, center_y, actual_radius, circle_path.poses.size());

  return circle_path;
}

void CirclePathStrategy::setAngleRange(double start_angle, double end_angle)
{
  circle_start_angle_ = start_angle;
  circle_end_angle_ = end_angle;
  // 加入一些冗余角度作为缓冲
  circle_total_angle_ = std::abs(end_angle - start_angle) + 0.8 * M_PI;

  RCLCPP_INFO(getLogger(), "设置角度范围: [%.2f, %.2f], 总角度: %.2f rad",
              start_angle, end_angle, circle_total_angle_);
}

void CirclePathStrategy::setCircleCenter(double x, double y)
{
  circle_center_x_ = x;
  circle_center_y_ = y;
}

void CirclePathStrategy::setCircleRadius(double radius)
{
  circle_radius_ = radius;
}

void CirclePathStrategy::adjustSpeedForRadius(double radius, double& min_v,
                                               double& max_v, double& lookahead_dist) const
{
  if (radius < 0.5)
  {
    lookahead_dist = 0.15;
    max_v = 0.08;
    min_v = 0.08;
  }
  else if (radius < 0.8)
  {
    lookahead_dist = 0.17;
    max_v = 0.10;
    min_v = 0.10;
  }
  else if (radius < 1.2)
  {
    lookahead_dist = 0.21;
    max_v = 0.11;
    min_v = 0.11;
  }
  else
  {
    lookahead_dist = 0.17;
    max_v = 0.11;
    min_v = 0.11;
  }

  RCLCPP_DEBUG(getLogger(), "根据半径 %.2f 调整速度: v=[%.2f,%.2f], lookahead=%.2f",
               radius, min_v, max_v, lookahead_dist);
}

void CirclePathStrategy::setBaselineLinearVelocity(double min_v)
{
  if (circle_radius_ > 1e-6)
  {
    baseline_angular_velocity_ = min_v / circle_radius_;
    RCLCPP_INFO(getLogger(), "设置基准角速度: %.4f rad/s (v=%.3f, r=%.3f)",
                baseline_angular_velocity_, min_v, circle_radius_);
  }
}

// ============================================================================
// 私有方法
// ============================================================================

void CirclePathStrategy::resetCirclePathState()
{
  last_yaw_initialized_ = false;
  last_yaw_ = 0.0;
  accumulated_angle_ = 0.0;
  angle_debug_counter_ = 0;
}

bool CirclePathStrategy::updateAccumulatedAngle(double current_yaw)
{
  if (!last_yaw_initialized_)
  {
    last_yaw_ = current_yaw;
    last_yaw_initialized_ = true;
    accumulated_angle_ = 0.0;
    RCLCPP_INFO(getLogger(), "圆形路径跟踪开始，初始航向角: %.2f", current_yaw);
    return false;
  }

  // 计算航向角变化量
  double delta_yaw = current_yaw - last_yaw_;

  // 处理角度跨越 ±π 的情况
  if (delta_yaw > M_PI)
  {
    delta_yaw -= 2.0 * M_PI;
  }
  else if (delta_yaw < -M_PI)
  {
    delta_yaw += 2.0 * M_PI;
  }

  // 只累计正向角度变化（假设逆时针为正）
  if (delta_yaw > 0)
  {
    accumulated_angle_ += delta_yaw;
  }

  last_yaw_ = current_yaw;

  // 开始打印标志
  if (accumulated_angle_ > 0.2 * M_PI)
  {
    start_print_ = true;
    stop_print_ = false;
  }

  // 判断是否完成
  if (accumulated_angle_ >= (circle_total_angle_ - 0.4 * M_PI))
  {
    return true;
  }

  return false;
}

double CirclePathStrategy::constrainAngularVelocity(double base_omega)
{
  // 计算与基准角速度的偏差
  double angular_velocity_delta = base_omega - baseline_angular_velocity_;

  // 确定允许的偏差比例
  double error_ratio = 0.05;  // 默认 5%

  // 起始区间：使用更大的偏差容许
  if (accumulated_angle_ < (start_deviation_factor_ * M_PI))
  {
    error_ratio = deviation_rate_;
  }

  // 结束区间：同样使用更大的偏差容许
  if (accumulated_angle_ > (circle_total_angle_ - end_deviation_factor_ * M_PI))
  {
    error_ratio = deviation_rate_;
  }

  // 限制偏差范围
  double max_angular_delta = baseline_angular_velocity_ * error_ratio;
  if (std::abs(angular_velocity_delta) > max_angular_delta)
  {
    angular_velocity_delta = std::copysign(max_angular_delta, angular_velocity_delta);
  }

  // 返回约束后的角速度
  return baseline_angular_velocity_ + angular_velocity_delta;
}

}  // namespace follow_controller
}  // namespace xline
