#include "xline_follow_controller/lqr_follow/lqr_follow_controller.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sstream>
#include <iomanip>

namespace xline
{
namespace follow_controller
{

// ============================================================================
// 构造函数与初始化
// ============================================================================

LQRFollowController::LQRFollowController()
  : BaseFollowController("lqr_follow_controller")
  , K1_(0.0)
  , K2_(0.0)
  , initialized_(false)
  , goal_reached_(false)
  , last_nearest_idx_(0)
  , need_yaw_prealign_(true)
  , yaw_prealign_done_(false)
  , target_yaw_(0.0)
  , integral_e_y_(0.0)
  , last_omega_(0.0)
  , debug_e_y_(0.0)
  , debug_e_theta_(0.0)
  , debug_omega_ff_(0.0)
  , debug_omega_fb_(0.0)
  , debug_ref_curvature_(0.0)
  , debug_ref_index_(0)
{
  // 基于环境变量初始化默认的栅格图路径
  const char* ws_root = std::getenv("XLINE_WS_ROOT");
  if (ws_root && *ws_root)
  {
    params_.grid_map_path = ws_root;
  }

  updateParameters("/config/lqr.yaml");
  initializeFilters();
  initialize();

  RCLCPP_INFO(get_logger(), "LQRFollowController 创建完成");
}

void LQRFollowController::initialize()
{
  // 计算控制周期
  params_.control_period = 1.0 / params_.control_frequency;

  // 计算初始增益（使用默认速度）
  computeLQRGains(params_.v_max);

  // 创建栅格图保存目录
  if (params_.enable_grid_map)
  {
    std::filesystem::create_directories(params_.grid_map_path);
    RCLCPP_INFO(get_logger(), "栅格图将保存到: %s", params_.grid_map_path.c_str());
  }

  initialized_ = true;
  RCLCPP_INFO(get_logger(),
              "LQRFollowController 初始化完成 - K1=%.2f, K2=%.2f, 路径类型=%s, v_max=%.3f m/s",
              K1_, K2_,
              params_.is_circular_path ? "圆形路径(固定速度)" : "一般路径",
              params_.v_max);
}

void LQRFollowController::updateParameters(const std::string& config_path)
{
  try
  {
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("xline_follow_controller");
    std::string full_path = package_share_directory + config_path;

    xline::YamlParser::YamlParser parser(full_path);

    // 加载LQR权重参数
    params_.q1 = parser.getParameter<double>("lqr.q1");
    params_.q2 = parser.getParameter<double>("lqr.q2");
    params_.r = parser.getParameter<double>("lqr.r");

    // 加载直接指定增益
    params_.use_direct_gains = parser.getParameter<bool>("lqr.use_direct_gains");
    params_.K1_direct = parser.getParameter<double>("lqr.K1_direct");
    params_.K2_direct = parser.getParameter<double>("lqr.K2_direct");

    // 加载积分项参数
    params_.enable_integral = parser.getParameter<bool>("lqr.enable_integral");
    params_.Ki = parser.getParameter<double>("lqr.Ki");
    params_.integral_max = parser.getParameter<double>("lqr.integral_max");
    params_.integral_decay = parser.getParameter<double>("lqr.integral_decay");

    // 加载速度限制
    params_.v_max = parser.getParameter<double>("velocity.max");
    params_.v_min = parser.getParameter<double>("velocity.min");
    params_.omega_max = parser.getParameter<double>("velocity.omega_max");
    params_.omega_dot_max = parser.getParameter<double>("velocity.omega_dot_max");

    // 加载机器人参数
    params_.wheel_base = parser.getParameter<double>("robot.wheel_base");

    // 加载前瞻参数
    params_.lookahead_distance = parser.getParameter<double>("lookahead.distance");
    params_.lookahead_time = parser.getParameter<double>("lookahead.time");

    // 加载搜索窗口
    params_.search_window_back = parser.getParameter<int>("search.window_back");
    params_.search_window_forward = parser.getParameter<int>("search.window_forward");

    // 加载目标容差
    params_.goal_dist_tolerance = parser.getParameter<double>("goal.dist_tolerance");
    params_.goal_angle_tolerance = parser.getParameter<double>("goal.angle_tolerance");

    // 加载控制频率
    params_.control_frequency = parser.getParameter<double>("control.frequency");

    // 加载原地旋转控制参数
    params_.rotation_factor = parser.getParameter<double>("rotation.factor");
    params_.rotation_max_w = parser.getParameter<double>("rotation.max_w");
    params_.rotation_min_w = parser.getParameter<double>("rotation.min_w");
    params_.rotation_angle_threshold = parser.getParameter<double>("rotation.angle_threshold");
    params_.rotation_smooth_factor = parser.getParameter<double>("rotation.smooth_factor");

    // 加载反馈限制参数
    params_.feedback_limit_ratio = parser.getParameter<double>("feedback.limit_ratio");
    params_.feedback_min_limit = parser.getParameter<double>("feedback.min_limit");

    // 加载滤波器参数
    // 位置滤波器参数
    params_.hampel_window = parser.getParameter<int>("filter.position.hampel_window");
    params_.hampel_k = parser.getParameter<double>("filter.position.hampel_k");
    params_.savgol_window = parser.getParameter<int>("filter.position.savgol_window");
    params_.savgol_order = parser.getParameter<int>("filter.position.savgol_order");

    // 加载调试参数
    params_.enable_debug = parser.getParameter<bool>("debug.enable");
    params_.verbose = parser.getParameter<bool>("debug.verbose");
    params_.enable_grid_map = parser.getParameter<bool>("debug.enable_grid_map");
    if (parser.hasParameter("debug.grid_map_path"))
    {
      params_.grid_map_path = xline::path_utils::resolve_path(
          parser.getParameter<std::string>("debug.grid_map_path"));
    }

    RCLCPP_INFO(get_logger(), "LQR参数已从配置文件加载: %s", full_path.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "加载LQR参数失败: %s，使用默认值", e.what());
  }
}

// ============================================================================
// BaseFollowController 接口实现
// ============================================================================

bool LQRFollowController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
  if (orig_global_plan.poses.empty())
  {
    RCLCPP_ERROR(get_logger(), "收到空路径，无法设置计划");
    return false;
  }

  // 重置状态
  reset();

  updateParameters("/config/lqr.yaml");

  // 转换路径格式并计算曲率
  path_.clear();
  path_.reserve(orig_global_plan.poses.size());

  double accumulated_arc_length = 0.0;

  for (size_t i = 0; i < orig_global_plan.poses.size(); ++i)
  {
    const auto& pose = orig_global_plan.poses[i];

    PathPointWithCurvature pt;
    pt.x = pose.pose.position.x;
    pt.y = pose.pose.position.y;
    pt.theta = tf2::getYaw(pose.pose.orientation);
    pt.curvature = 0.0;  // 稍后计算
    pt.arc_length = accumulated_arc_length;

    // 累积弧长
    if (i > 0)
    {
      double dx = pt.x - path_[i-1].x;
      double dy = pt.y - path_[i-1].y;
      accumulated_arc_length += std::hypot(dx, dy);
      pt.arc_length = accumulated_arc_length;
    }

    path_.push_back(pt);
  }

  // 计算曲率
  computePathCurvature();

  // 设置目标航向角为路径起点的朝向
  if (!path_.empty())
  {
    target_yaw_ = path_[0].theta;
    need_yaw_prealign_ = true;
    yaw_prealign_done_ = false;
  }

  RCLCPP_INFO(get_logger(), "LQR路径设置完成 - 点数: %zu, 总长度: %.3fm, 起始朝向: %.2f°",
              path_.size(), accumulated_arc_length, target_yaw_ * 180.0 / M_PI);

  // 初始化栅格图
  if (params_.enable_grid_map)
  {
    initializeGridMap(orig_global_plan);
  }

  return true;
}

void LQRFollowController::setAngleRange(double start_angle, double end_angle)
{
  circle_total_angle_ = std::abs(end_angle - start_angle);

  // 兼容：如果输入角度范围为 0，则默认走一整圈
  if (circle_total_angle_ < 1e-6)
  {
    circle_total_angle_ = 2.0 * M_PI;
  }

  RCLCPP_INFO(get_logger(), "设置圆形角度范围: [%.2f, %.2f], 总角度: %.2f rad (%.1f°)",
              start_angle, end_angle, circle_total_angle_,
              circle_total_angle_ * 180.0 / M_PI);
}

bool LQRFollowController::setPlanForCircle(double circle_center_x, double circle_center_y,
                                          double circle_radius,
                                          const geometry_msgs::msg::PoseStamped& robot_pose)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化，请先调用 initialize()");
    return false;
  }

  if (circle_radius <= 0.0)
  {
    RCLCPP_ERROR(get_logger(), "圆半径必须为正值");
    return false;
  }

  params_.is_circular_path = true;

  nav_msgs::msg::Path circle_path =
      generateCirclePath(circle_center_x, circle_center_y, circle_radius, robot_pose);

  RCLCPP_INFO(get_logger(), "圆形路径已生成 - 圆心: (%.3f, %.3f), 半径: %.3f m, 总角度: %.2f rad, 点数: %zu",
              circle_center_x, circle_center_y, circle_radius, circle_total_angle_,
              circle_path.poses.size());

  return setPlan(circle_path);
}

nav_msgs::msg::Path LQRFollowController::generateCirclePath(
    double center_x, double center_y, double radius,
    const geometry_msgs::msg::PoseStamped& start_pose) const
{
  nav_msgs::msg::Path circle_path;
  circle_path.header.frame_id =
      start_pose.header.frame_id.empty() ? std::string("world") : start_pose.header.frame_id;
  circle_path.header.stamp = this->now();

  geometry_msgs::msg::PoseStamped entry_pose = start_pose;

  // 起点相对圆心的切线方向
  double tangent_x = -(entry_pose.pose.position.y - center_y);
  double tangent_y = (entry_pose.pose.position.x - center_x);
  double tangent_length = std::hypot(tangent_x, tangent_y);

  if (tangent_length > 1e-6)
  {
    tangent_x /= tangent_length;
    tangent_y /= tangent_length;
  }

  const double entry_yaw = std::atan2(tangent_y, tangent_x);
  tf2::Quaternion entry_q;
  entry_q.setRPY(0, 0, entry_yaw);
  entry_pose.pose.orientation = tf2::toMsg(entry_q);

  circle_path.poses.push_back(entry_pose);

  // 以“弧长间距”决定采样密度：点间距固定 0.003m（3mm）
  constexpr double kPointSpacingMeters = 0.003;
  const double safe_radius = std::max(radius, 1e-6);
  const double max_step_angle = kPointSpacingMeters / safe_radius;

  const double start_angle = std::atan2(
      entry_pose.pose.position.y - center_y,
      entry_pose.pose.position.x - center_x);

  const double total_angle = std::max(circle_total_angle_, 1e-6);

  size_t num_segments = static_cast<size_t>(std::ceil(total_angle / max_step_angle));
  num_segments = std::max<size_t>(num_segments, 1);

  constexpr size_t kMaxSegments = 20000;
  if (num_segments > kMaxSegments)
  {
    RCLCPP_WARN(get_logger(),
                "圆弧采样点过多(%zu)，已限制到 %zu 段；可通过增大点间距降低负载",
                num_segments, kMaxSegments);
    num_segments = kMaxSegments;
  }

  const double step_angle = total_angle / static_cast<double>(num_segments);

  // 跳过入口处一小段，减少“入口过密”对控制的干扰
  const size_t start_idx = (num_segments > 5) ? 5 : 1;
  for (size_t idx = start_idx; idx <= num_segments; ++idx)
  {
    const double angle = start_angle + static_cast<double>(idx) * step_angle;

    geometry_msgs::msg::PoseStamped circle_pose;
    circle_pose.header = entry_pose.header;
    circle_pose.pose.position.x = center_x + radius * std::cos(angle);
    circle_pose.pose.position.y = center_y + radius * std::sin(angle);
    circle_pose.pose.position.z = start_pose.pose.position.z;

    const double tangent_direction = angle + M_PI / 2.0;
    tf2::Quaternion circle_q;
    circle_q.setRPY(0, 0, tangent_direction);
    circle_pose.pose.orientation = tf2::toMsg(circle_q);

    circle_path.poses.push_back(circle_pose);
  }

  return circle_path;
}

bool LQRFollowController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    geometry_msgs::msg::TwistStamped& cmd_vel)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化");
    return false;
  }

  if (path_.empty())
  {
    RCLCPP_ERROR(get_logger(), "路径为空");
    return false;
  }

  if (goal_reached_)
  {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return true;
  }

  // 位置滤波处理（与RPP一致：先过滤机器人坐标再进入计算）
  geometry_msgs::msg::PoseStamped current_pose = filterRobotPose(pose);

  // 提取当前状态（使用滤波后的位置）
  double current_x = current_pose.pose.position.x;
  double current_y = current_pose.pose.position.y;
  double current_theta = tf2::getYaw(current_pose.pose.orientation);

  // 根据路径类型设置期望速度
  // 无论圆形路径还是一般路径，都使用期望速度计算前馈
  // 原因：底层速度控制良好，使用期望速度可避免测量噪声影响前馈稳定性
  double current_v;
  if (params_.is_circular_path)
  {
    // 圆形路径：固定使用最大速度以保持稳定的曲率跟踪
    // 前馈控制 ω_ff = v × κ 要求速度恒定才能准确跟踪圆弧
    current_v = params_.v_max;
  }
  else
  {
    // 一般路径：同样使用期望速度
    // 保持前馈的稳定性和可预测性
    current_v = params_.v_max;
  }

  // 0. 检查是否需要航向预对准
  if (need_yaw_prealign_ && !yaw_prealign_done_)
  {
    if (performYawPrealignment(pose, target_yaw_, cmd_vel))
    {
      yaw_prealign_done_ = true;
      need_yaw_prealign_ = false;
      RCLCPP_INFO(get_logger(), "LQR航向预对准完成，开始路径跟随");
    }
    return true;  // 预对准期间返回 true，继续执行预对准
  }

  // 1. 找到最近的路径点
  size_t nearest_idx = findNearestPoint(current_x, current_y);

  // 2. 计算前瞻距离
  double lookahead_dist = getLookaheadDistance(current_v);

  // 3. 获取前瞻参考点（带插值）
  PathPointWithCurvature ref = findLookaheadPoint(nearest_idx, lookahead_dist);

  // 4. 计算误差
  double e_y, e_theta;
  computeErrors(current_x, current_y, current_theta, ref, e_y, e_theta);

  // 5. 计算LQR增益（基于当前速度）
  computeLQRGains(current_v);

  // 6. 前馈控制 ω_ff = v × κ
  double omega_ff = current_v * ref.curvature;

  // 7. LQR反馈控制 ω_fb = -K₁·e_y - K₂·e_θ
  double omega_fb = -K1_ * e_y - K2_ * e_theta;

  // 8. 积分项（可选）
  double omega_i = 0.0;
  if (params_.enable_integral)
  {
    integral_e_y_ = params_.integral_decay * integral_e_y_ + e_y * params_.control_period;
    // 积分限幅
    integral_e_y_ = std::clamp(integral_e_y_, -params_.integral_max / params_.Ki,
                                params_.integral_max / params_.Ki);
    omega_i = -params_.Ki * integral_e_y_;
  }

  // 9. 限制反馈量不超过前馈的指定比例
  // 总反馈量 = LQR反馈 + 积分项
  double omega_correction = omega_fb + omega_i;

  // 计算反馈限制：前馈 × 限制比例（默认5%）
  double feedback_limit = std::abs(omega_ff) * params_.feedback_limit_ratio;

  // 仅当配置了最小限制（>0）时才使用
  if (params_.feedback_min_limit > 0.0 && feedback_limit < params_.feedback_min_limit)
  {
    feedback_limit = params_.feedback_min_limit;
  }

  // 限幅反馈控制量
  omega_correction = std::clamp(omega_correction, -feedback_limit, feedback_limit);

  // 10. 总控制量 = 前馈 + 限幅后的反馈
  double omega = omega_ff + omega_correction;

  // 11. 应用角速度和角加速度限幅
  omega = applyLimits(omega);

  // 13. 设置输出
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.twist.linear.x = current_v;
  cmd_vel.twist.angular.z = omega;

  // 保存调试信息
  debug_e_y_ = e_y;
  debug_e_theta_ = e_theta;
  debug_omega_ff_ = omega_ff;
  debug_omega_fb_ = omega_correction;  // 保存限幅后的反馈量
  debug_ref_curvature_ = ref.curvature;
  debug_ref_index_ = nearest_idx;  // 使用最近点索引作为参考

  // 调试输出
  if (params_.enable_debug && params_.verbose)
  {
    RCLCPP_INFO(get_logger(),
                "LQR控制: 横向误差=%.1fmm, 航向角误差=%.2f°, 前馈角速度=%.3f, 反馈角速度=%.3f(限幅±%.3f), 总角速度=%.3f, 曲率=%.3f",
                e_y * 1000, e_theta * 180 / M_PI,
                omega_ff, omega_correction, feedback_limit, omega, ref.curvature);
  }

  // 更新栅格图可视化
  updateGridMapIfNeeded(pose, ref);

  return true;
}

bool LQRFollowController::isGoalReached()
{
  if (goal_reached_)
  {
    return true;
  }

  // 简化实现：检查是否接近最后一个路径点
  // 实际应用中应在computeVelocityCommands中检查
  return false;
}

bool LQRFollowController::cancel()
{
  reset();
  RCLCPP_INFO(get_logger(), "LQR控制器已取消");
  return true;
}

// ============================================================================
// LQR 特有接口
// ============================================================================

void LQRFollowController::getGains(double& K1, double& K2) const
{
  K1 = K1_;
  K2 = K2_;
}

std::string LQRFollowController::getDebugInfo() const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "LQR["
      << " e_y=" << debug_e_y_ * 1000 << "mm"
      << ", e_θ=" << debug_e_theta_ * 180 / M_PI << "°"
      << ", ω_ff=" << debug_omega_ff_
      << ", ω_fb=" << debug_omega_fb_
      << ", κ=" << debug_ref_curvature_
      << ", K1=" << K1_
      << ", K2=" << K2_
      << "]";
  return oss.str();
}

// ============================================================================
// 核心计算函数
// ============================================================================

void LQRFollowController::computeLQRGains(double v)
{
  if (params_.use_direct_gains)
  {
    // 使用直接指定的增益
    K1_ = params_.K1_direct;
    K2_ = params_.K2_direct;
  }
  else
  {
    // 从LQR权重参数计算增益
    // 防止除零
    v = std::max(v, 0.05);

    // K1 = √(q1/r) / v
    K1_ = std::sqrt(params_.q1 / params_.r) / v;

    // K2 = √(2√(q1·q2)/r + q2/r)
    K2_ = std::sqrt(2.0 * std::sqrt(params_.q1 * params_.q2) / params_.r +
                    params_.q2 / params_.r);
  }
}

void LQRFollowController::computePathCurvature()
{
  if (path_.size() < 3)
  {
    // 路径点太少，无法计算曲率
    for (auto& pt : path_)
    {
      pt.curvature = 0.0;
    }
    return;
  }

  // 使用三点法计算曲率
  for (size_t i = 1; i < path_.size() - 1; ++i)
  {
    const auto& p0 = path_[i - 1];
    const auto& p1 = path_[i];
    const auto& p2 = path_[i + 1];

    // 计算一阶导数（中心差分）
    double dx = (p2.x - p0.x) / 2.0;
    double dy = (p2.y - p0.y) / 2.0;

    // 计算二阶导数
    double ddx = p2.x - 2.0 * p1.x + p0.x;
    double ddy = p2.y - 2.0 * p1.y + p0.y;

    // 曲率公式: κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
    double cross = dx * ddy - dy * ddx;
    double norm_cubed = std::pow(dx * dx + dy * dy, 1.5);

    if (norm_cubed < 1e-10)
    {
      path_[i].curvature = 0.0;
    }
    else
    {
      path_[i].curvature = cross / norm_cubed;
    }
  }

  // 端点使用相邻点的曲率
  path_[0].curvature = path_[1].curvature;
  path_.back().curvature = path_[path_.size() - 2].curvature;
}

size_t LQRFollowController::findNearestPoint(double x, double y)
{
  if (path_.empty())
  {
    return 0;
  }

  // 使用搜索窗口优化（避免曲线处震荡）
  size_t start_idx = (last_nearest_idx_ > params_.search_window_back)
                         ? (last_nearest_idx_ - params_.search_window_back)
                         : 0;

  size_t end_idx = std::min(last_nearest_idx_ + params_.search_window_forward,
                            path_.size());

  size_t nearest_idx = start_idx;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = start_idx; i < end_idx; ++i)
  {
    double dx = path_[i].x - x;
    double dy = path_[i].y - y;
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq < min_dist_sq)
    {
      min_dist_sq = dist_sq;
      nearest_idx = i;
    }
  }

  last_nearest_idx_ = nearest_idx;
  return nearest_idx;
}

PathPointWithCurvature LQRFollowController::findLookaheadPoint(size_t nearest_idx,
                                                                double lookahead_dist)
{
  if (path_.empty())
  {
    return PathPointWithCurvature();
  }

  // 如果最近点已经是最后一个点，直接返回
  if (nearest_idx >= path_.size() - 1)
  {
    return path_.back();
  }

  double accumulated_dist = 0.0;

  // 从最近点开始，沿路径累积距离
  for (size_t i = nearest_idx; i < path_.size() - 1; ++i)
  {
    double segment_length = std::hypot(
        path_[i + 1].x - path_[i].x,
        path_[i + 1].y - path_[i].y);

    // 如果累积距离加上当前段长度超过前瞻距离，进行插值
    if (accumulated_dist + segment_length >= lookahead_dist)
    {
      double remaining = lookahead_dist - accumulated_dist;
      double ratio = remaining / segment_length;

      // 插值计算前瞻点
      PathPointWithCurvature lookahead_pt;
      lookahead_pt.x = path_[i].x + ratio * (path_[i + 1].x - path_[i].x);
      lookahead_pt.y = path_[i].y + ratio * (path_[i + 1].y - path_[i].y);

      // 插值曲率
      lookahead_pt.curvature = path_[i].curvature +
                                ratio * (path_[i + 1].curvature - path_[i].curvature);

      // 计算朝向（根据路径段的方向）
      lookahead_pt.theta = std::atan2(
          path_[i + 1].y - path_[i].y,
          path_[i + 1].x - path_[i].x);

      // 插值弧长
      lookahead_pt.arc_length = path_[i].arc_length + remaining;

      return lookahead_pt;
    }

    accumulated_dist += segment_length;
  }

  // 如果前瞻距离超过路径末端，返回最后一个点
  return path_.back();
}

double LQRFollowController::getLookaheadDistance(double speed)
{
  // 基于时间-速度模型计算前瞻距离
  double lookahead_dist = params_.lookahead_distance +
                          params_.lookahead_time * std::abs(speed);

  // 简单限幅（可根据需要添加 min/max 参数）
  return std::max(0.02, lookahead_dist);  // 最小20mm前瞻
}

void LQRFollowController::computeErrors(double current_x, double current_y,
                                         double current_theta,
                                         const PathPointWithCurvature& ref,
                                         double& e_y, double& e_theta)
{
  // 计算位置偏差
  double dx = current_x - ref.x;
  double dy = current_y - ref.y;

  // 横向误差（垂直于路径方向，Frenet坐标系）
  e_y = -dx * std::sin(ref.theta) + dy * std::cos(ref.theta);

  // 航向误差
  e_theta = normalizeAngle(current_theta - ref.theta);
}

double LQRFollowController::applyLimits(double omega)
{
  // 角速度限幅
  omega = std::clamp(omega, -params_.omega_max, params_.omega_max);

  // 角加速度限幅（平滑控制）
  double omega_change = omega - last_omega_;
  double max_change = params_.omega_dot_max * params_.control_period;

  if (std::abs(omega_change) > max_change)
  {
    omega = last_omega_ + std::copysign(max_change, omega_change);
  }

  last_omega_ = omega;
  return omega;
}

double LQRFollowController::normalizeAngle(double angle)
{
  // 使用 fmod 实现 O(1) 复杂度，避免 while 循环
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0)
  {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

bool LQRFollowController::performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose,
                                                  double target_yaw,
                                                  geometry_msgs::msg::TwistStamped& cmd_vel)
{
  // 获取当前航向角
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  // 计算航向角差
  double angle_diff = normalizeAngle(target_yaw - current_yaw);

  // 检查是否已对准（默认阈值：约5度）
  const double yaw_tolerance = params_.goal_angle_tolerance;
  if (std::abs(angle_diff) < yaw_tolerance)
  {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return true;  // 对准完成
  }

  // 计算旋转速度
  double rotation_velocity = calculateRotationVelocity(angle_diff);

  // 设置输出命令（仅旋转，不前进）
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = current_pose.header.frame_id;
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = rotation_velocity;

  return false;  // 对准未完成
}

double LQRFollowController::calculateRotationVelocity(double angle_diff)
{
  // 使用sigmoid函数计算角速度因子（与line_follow_controller一致）
  double factor = 1.0 / (1.0 + std::exp(-params_.rotation_factor * std::abs(angle_diff)));

  // 对于小角度，使用余弦函数进一步平滑
  if (std::abs(angle_diff) < params_.rotation_angle_threshold)
  {
    double cosine_factor = params_.rotation_smooth_factor *
        (1.0 - std::cos(M_PI * std::abs(angle_diff) / params_.rotation_angle_threshold));
    factor *= cosine_factor;
  }

  // 计算角速度
  double rot_vel = params_.rotation_max_w * factor;
  rot_vel = std::max(rot_vel, params_.rotation_min_w);

  // 根据角度差的符号确定旋转方向
  return (angle_diff > 0.0) ? rot_vel : -rot_vel;
}

void LQRFollowController::reset()
{
  goal_reached_ = false;
  last_nearest_idx_ = 0;
  integral_e_y_ = 0.0;
  last_omega_ = 0.0;
  debug_e_y_ = 0.0;
  debug_e_theta_ = 0.0;
  debug_omega_ff_ = 0.0;
  debug_omega_fb_ = 0.0;
  debug_ref_curvature_ = 0.0;
  debug_ref_index_ = 0;


  // 重置位置滤波器（使用配置参数）
  sg_x_filter_.reset(params_.savgol_window, params_.savgol_order);
  sg_y_filter_.reset(params_.savgol_window, params_.savgol_order);
  h_x_filter.reset(params_.hampel_window, params_.hampel_k);
  h_y_filter.reset(params_.hampel_window, params_.hampel_k);


  RCLCPP_DEBUG(get_logger(), "LQR控制器状态已重置");
}

void LQRFollowController::initializeFilters()
{
  // 初始化位置滤波器（从参数配置）
  sg_x_filter_ = SavitzkyGolayFilter(params_.savgol_window, params_.savgol_order);
  sg_y_filter_ = SavitzkyGolayFilter(params_.savgol_window, params_.savgol_order);
  h_x_filter = HampelFilter(params_.hampel_window, params_.hampel_k);
  h_y_filter = HampelFilter(params_.hampel_window, params_.hampel_k);
}

geometry_msgs::msg::PoseStamped LQRFollowController::filterRobotPose(
    const geometry_msgs::msg::PoseStamped& robot_pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header = robot_pose.header;
  current_pose.pose.orientation = robot_pose.pose.orientation;

  // 应用Hampel滤波器
  double filtered_x = h_x_filter.filter(robot_pose.pose.position.x);
  double filtered_y = h_y_filter.filter(robot_pose.pose.position.y);

  // 应用Savitzky-Golay滤波器
  filtered_x = sg_x_filter_.filter(filtered_x);
  filtered_y = sg_y_filter_.filter(filtered_y);

  current_pose.pose.position.x = filtered_x;
  current_pose.pose.position.y = filtered_y;

  return current_pose;
}

// ============================================================================
// 栅格图可视化
// ============================================================================

void LQRFollowController::initializeGridMap(const nav_msgs::msg::Path& path)
{
  if (path.poses.empty())
  {
    RCLCPP_WARN(get_logger(), "无法初始化栅格图：路径为空");
    return;
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& pose : path.poses)
  {
    min_x = std::min(min_x, pose.pose.position.x);
    min_y = std::min(min_y, pose.pose.position.y);
    max_x = std::max(max_x, pose.pose.position.x);
    max_y = std::max(max_y, pose.pose.position.y);
  }

  double margin = 0.05;  // 50mm margin
  grid_width_ = (max_x - min_x) + 2 * margin;
  grid_height_ = (max_y - min_y) + 2 * margin;
  grid_origin_x_ = min_x - margin;
  grid_origin_y_ = min_y - margin;

  int width_pixels = static_cast<int>(grid_width_ / grid_resolution_);
  int height_pixels = static_cast<int>(grid_height_ / grid_resolution_);
  grid_map_ = cv::Mat(height_pixels, width_pixels, CV_8UC3, cv::Scalar(255, 255, 255));

  drawGridLines();
  drawPathOnGrid(path, cv::Scalar(0, 0, 255), 1);  // Red path

  last_grid_update_time_ = this->now();
  saveGridMap();

  RCLCPP_INFO(get_logger(), "初始化栅格图 大小: %.2f x %.2f m, 尺寸: %d x %d 像素",
              grid_width_, grid_height_, width_pixels, height_pixels);
}

cv::Point LQRFollowController::worldToGrid(double x, double y)
{
  int grid_x = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
  int grid_y = grid_map_.rows - static_cast<int>((y - grid_origin_y_) / grid_resolution_) - 1;
  return cv::Point(grid_x, grid_y);
}

void LQRFollowController::drawPathOnGrid(const nav_msgs::msg::Path& path,
                                          const cv::Scalar& color, int thickness)
{
  if (path.poses.empty() || grid_map_.empty())
  {
    return;
  }

  for (size_t i = 0; i < path.poses.size() - 1; ++i)
  {
    cv::Point pt1 = worldToGrid(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    cv::Point pt2 = worldToGrid(path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y);

    if (isPointInGrid(pt1) && isPointInGrid(pt2))
    {
      cv::line(grid_map_, pt1, pt2, color, thickness);
    }
  }

  // 绘制起点和终点
  if (!path.poses.empty())
  {
    cv::Point start = worldToGrid(path.poses.front().pose.position.x,
                                   path.poses.front().pose.position.y);
    cv::Point end = worldToGrid(path.poses.back().pose.position.x,
                                 path.poses.back().pose.position.y);

    if (isPointInGrid(start))
    {
      cv::circle(grid_map_, start, 3, cv::Scalar(0, 255, 0), -1);  // Green start
    }
    if (isPointInGrid(end))
    {
      cv::circle(grid_map_, end, 3, cv::Scalar(255, 0, 0), -1);  // Blue end
    }
  }
}

void LQRFollowController::drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose)
{
  if (grid_map_.empty())
  {
    return;
  }

  cv::Point robot_pt = worldToGrid(pose.pose.position.x, pose.pose.position.y);
  if (isPointInGrid(robot_pt))
  {
    cv::circle(grid_map_, robot_pt, 2, cv::Scalar(0, 165, 255), -1);  // Orange robot
  }
}

void LQRFollowController::drawLookaheadPointOnGrid(const PathPointWithCurvature& lookahead_point)
{
  if (grid_map_.empty())
  {
    return;
  }

  cv::Point lookahead_pt = worldToGrid(lookahead_point.x, lookahead_point.y);
  if (isPointInGrid(lookahead_pt))
  {
    cv::circle(grid_map_, lookahead_pt, 3, cv::Scalar(255, 0, 255), -1);  // Magenta lookahead
  }
}

void LQRFollowController::drawGridLines()
{
  if (grid_map_.empty())
  {
    return;
  }

  cv::Scalar grid_color(220, 220, 220);

  // Draw vertical grid lines every 10mm
  for (double x = grid_origin_x_; x < grid_origin_x_ + grid_width_; x += 0.01)
  {
    cv::Point pt1 = worldToGrid(x, grid_origin_y_);
    cv::Point pt2 = worldToGrid(x, grid_origin_y_ + grid_height_);
    if (isPointInGrid(pt1) && isPointInGrid(pt2))
    {
      cv::line(grid_map_, pt1, pt2, grid_color, 1);
    }
  }

  // Draw horizontal grid lines every 10mm
  for (double y = grid_origin_y_; y < grid_origin_y_ + grid_height_; y += 0.01)
  {
    cv::Point pt1 = worldToGrid(grid_origin_x_, y);
    cv::Point pt2 = worldToGrid(grid_origin_x_ + grid_width_, y);
    if (isPointInGrid(pt1) && isPointInGrid(pt2))
    {
      cv::line(grid_map_, pt1, pt2, grid_color, 1);
    }
  }
}

void LQRFollowController::saveGridMap()
{
  if (grid_map_.empty() || params_.grid_map_path.empty())
  {
    return;
  }

  std::string filename = params_.grid_map_path + "/lqr_path_tracking.png";
  cv::imwrite(filename, grid_map_);
}

void LQRFollowController::updateGridMapIfNeeded(const geometry_msgs::msg::PoseStamped& current_pose,
                                                 const PathPointWithCurvature& lookahead_point)
{
  if (!params_.enable_grid_map)
  {
    return;
  }

  auto current_time = this->now();
  if ((current_time - last_grid_update_time_).seconds() > 1.0)
  {
    drawRobotOnGrid(current_pose);
    drawLookaheadPointOnGrid(lookahead_point);
    saveGridMap();
    last_grid_update_time_ = current_time;
  }
}

bool LQRFollowController::isPointInGrid(const cv::Point& pt)
{
  return pt.x >= 0 && pt.x < grid_map_.cols && pt.y >= 0 && pt.y < grid_map_.rows;
}

}  // namespace follow_controller
}  // namespace xline
