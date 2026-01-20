#include "xline_follow_controller/lqr_follow/lqr_curve_controller.hpp"
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

LQRCurveController::LQRCurveController()
  : BaseFollowController("lqr_curve_controller")
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

  updateParameters("/config/lqr_curve.yaml");
  initializeFilters();
  initialize();

  RCLCPP_INFO(get_logger(), "LQRFollowController 创建完成");
}

void LQRCurveController::initialize()
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
              "LQRFollowController 初始化完成 - K1=%.2f, K2=%.2f, v_max=%.3f m/s",
              K1_, K2_,
              params_.v_max);
}

void LQRCurveController::updateParameters(const std::string& config_path)
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

bool LQRCurveController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
  if (orig_global_plan.poses.empty())
  {
    RCLCPP_ERROR(get_logger(), "收到空路径，无法设置计划");
    return false;
  }

  // 重置状态
  // reset();
  // updateParameters("/config/lqr_curve.yaml");

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


bool LQRCurveController::computeVelocityCommands(
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

  // 一般路径：同样使用期望速度
  // 保持前馈的稳定性和可预测性
  current_v = params_.v_max;


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

  // 12. 对于椭圆路径，检查是否完成
  bool completed = updateAccumulatedDistance(current_x, current_y);
  if (completed)
  {
    goal_reached_ = true;
    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    RCLCPP_INFO(get_logger(), "椭圆路径完成 - 累计距离: %.4f m",
                accumulated_distance_);
    return true;
  }

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

bool LQRCurveController::isGoalReached()
{
  if (goal_reached_)
  {
    return true;
  }

  // 简化实现：检查是否接近最后一个路径点
  // 实际应用中应在computeVelocityCommands中检查
  return false;
}

bool LQRCurveController::cancel()
{
  reset();
  RCLCPP_INFO(get_logger(), "LQR控制器已取消");
  return true;
}

// ============================================================================
// LQR 特有接口
// ============================================================================

void LQRCurveController::getGains(double& K1, double& K2) const
{
  K1 = K1_;
  K2 = K2_;
}

std::string LQRCurveController::getDebugInfo() const
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

void LQRCurveController::computeLQRGains(double v)
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

void LQRCurveController::computePathCurvature()
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

size_t LQRCurveController::findNearestPoint(double x, double y)
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

PathPointWithCurvature LQRCurveController::findLookaheadPoint(size_t nearest_idx,
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

double LQRCurveController::getLookaheadDistance(double speed)
{
  // 基于时间-速度模型计算前瞻距离
  double lookahead_dist = params_.lookahead_distance +
                          params_.lookahead_time * std::abs(speed);

  // 简单限幅（可根据需要添加 min/max 参数）
  return std::max(0.02, lookahead_dist);  // 最小20mm前瞻
}

void LQRCurveController::computeErrors(double current_x, double current_y,
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

double LQRCurveController::applyLimits(double omega)
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

double LQRCurveController::normalizeAngle(double angle)
{
  // 使用 fmod 实现 O(1) 复杂度，避免 while 循环
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0)
  {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

bool LQRCurveController::performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose,
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

double LQRCurveController::calculateRotationVelocity(double angle_diff)
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

void LQRCurveController::reset()
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

  // 重置椭圆路径参数
  ellipse_a_ = 0.0;
  ellipse_b_ = 0.0;
  ellipse_center_x_ = 0.0;
  ellipse_center_y_ = 0.0;
  ellipse_rotation_ = 0.0;
  target_arc_length_ = 0.0;
  ellipse_perimeter_ = 0.0;

  // 重置距离累计相关状态
  last_position_initialized_ = false;
  last_x_ = 0.0;
  last_y_ = 0.0;
  accumulated_distance_ = 0.0;
  start_print = false;
  stop_print = false;
  print_window_initialized_ = false;
  start_print_distance_ = 0.0;
  stop_print_start_distance_ = 0.0;
  stop_print_end_distance_ = 0.0;

  // 重置位置滤波器（使用配置参数）
  sg_x_filter_.reset(params_.savgol_window, params_.savgol_order);
  sg_y_filter_.reset(params_.savgol_window, params_.savgol_order);
  h_x_filter.reset(params_.hampel_window, params_.hampel_k);
  h_y_filter.reset(params_.hampel_window, params_.hampel_k);


  RCLCPP_DEBUG(get_logger(), "LQR控制器状态已重置");
}

void LQRCurveController::initializeFilters()
{
  // 初始化位置滤波器（从参数配置）
  sg_x_filter_ = SavitzkyGolayFilter(params_.savgol_window, params_.savgol_order);
  sg_y_filter_ = SavitzkyGolayFilter(params_.savgol_window, params_.savgol_order);
  h_x_filter = HampelFilter(params_.hampel_window, params_.hampel_k);
  h_y_filter = HampelFilter(params_.hampel_window, params_.hampel_k);
}

geometry_msgs::msg::PoseStamped LQRCurveController::filterRobotPose(
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
// 椭圆路径辅助函数
// ============================================================================

double LQRCurveController::projectPointToEllipse(double px, double py,
                                                   double center_x, double center_y,
                                                   double a, double b,
                                                   double rotation) const
{
  // 将点转换到椭圆局部坐标系（未旋转状态）
  double dx = px - center_x;
  double dy = py - center_y;

  // 应用逆旋转
  double cos_r = std::cos(-rotation);
  double sin_r = std::sin(-rotation);
  double x_local = dx * cos_r - dy * sin_r;
  double y_local = dx * sin_r + dy * cos_r;

  // 使用迭代法求解最近点（牛顿法）
  // 椭圆参数方程：(a*cos(t), b*sin(t))
  // 初始猜测：使用atan2得到的角度
  double t = std::atan2(y_local / b, x_local / a);

  // 牛顿迭代优化
  for (int i = 0; i < 10; ++i)
  {
    double x_ellipse = a * std::cos(t);
    double y_ellipse = b * std::sin(t);

    // 误差向量
    double ex = x_ellipse - x_local;
    double ey = y_ellipse - y_local;

    // 椭圆切线方向（参数方程的导数）
    double dx_dt = -a * std::sin(t);
    double dy_dt = b * std::cos(t);

    // 误差向量与切线的点积（应该为0）
    double f = ex * dx_dt + ey * dy_dt;

    // 导数
    double df = -a * std::cos(t) * dx_dt - a * std::sin(t) * ex
                - b * std::sin(t) * dy_dt + b * std::cos(t) * ey;

    if (std::abs(df) < 1e-10)
    {
      break;
    }

    // 牛顿更新
    t = t - f / df;
  }

  // 归一化角度到 [0, 2π]
  while (t < 0) t += 2.0 * M_PI;
  while (t >= 2.0 * M_PI) t -= 2.0 * M_PI;

  return t;
}

// ============================================================================
// 栅格图可视化
// ============================================================================

void LQRCurveController::initializeGridMap(const nav_msgs::msg::Path& path)
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

cv::Point LQRCurveController::worldToGrid(double x, double y)
{
  int grid_x = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
  int grid_y = grid_map_.rows - static_cast<int>((y - grid_origin_y_) / grid_resolution_) - 1;
  return cv::Point(grid_x, grid_y);
}

void LQRCurveController::drawPathOnGrid(const nav_msgs::msg::Path& path,
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

void LQRCurveController::drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose)
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

void LQRCurveController::drawLookaheadPointOnGrid(const PathPointWithCurvature& lookahead_point)
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

void LQRCurveController::drawGridLines()
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

void LQRCurveController::saveGridMap()
{
  if (grid_map_.empty() || params_.grid_map_path.empty())
  {
    return;
  }

  std::string filename = params_.grid_map_path + "/lqr_path_tracking.png";
  cv::imwrite(filename, grid_map_);
}

void LQRCurveController::updateGridMapIfNeeded(const geometry_msgs::msg::PoseStamped& current_pose,
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

bool LQRCurveController::isPointInGrid(const cv::Point& pt)
{
  return pt.x >= 0 && pt.x < grid_map_.cols && pt.y >= 0 && pt.y < grid_map_.rows;
}

// ============================================================================
// Spline 和 Ellipse 路径设置
// ============================================================================

bool LQRCurveController::setPlanForSpline(const std::vector<std::pair<double, double>>& vertices,
                                           int degree,
                                           double start_x, double start_y,
                                           double end_x, double end_y)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化，请先调用 initialize()");
    return false;
  }

  // 参数验证
  if (vertices.size() < 2)
  {
    RCLCPP_ERROR(get_logger(), "Spline路径控制点数不足（至少需要2个点），当前: %zu", vertices.size());
    return false;
  }

  // 重置状态
  reset();
  updateParameters("/config/lqr_curve.yaml");

  // 生成 Spline 路径
  nav_msgs::msg::Path spline_path;
  spline_path.header.frame_id = "map";
  spline_path.header.stamp = this->now();

  // 直接使用控制点作为路径点（简化实现，实际可以进行插值）
  for (const auto& vertex : vertices)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = spline_path.header;
    pose.pose.position.x = vertex.first;
    pose.pose.position.y = vertex.second;
    pose.pose.position.z = 0.0;

    // 初始朝向（后续会计算）
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    spline_path.poses.push_back(pose);
  }

  // 为路径点计算朝向（每个点朝向下一个点）
  for (size_t i = 0; i < spline_path.poses.size() - 1; ++i)
  {
    double dx = spline_path.poses[i + 1].pose.position.x - spline_path.poses[i].pose.position.x;
    double dy = spline_path.poses[i + 1].pose.position.y - spline_path.poses[i].pose.position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    spline_path.poses[i].pose.orientation.x = q.x();
    spline_path.poses[i].pose.orientation.y = q.y();
    spline_path.poses[i].pose.orientation.z = q.z();
    spline_path.poses[i].pose.orientation.w = q.w();
  }

  // 最后一个点的朝向与前一个点相同
  if (spline_path.poses.size() > 1)
  {
    spline_path.poses.back().pose.orientation =
        spline_path.poses[spline_path.poses.size() - 2].pose.orientation;
  }

  RCLCPP_INFO(get_logger(),
              "Spline路径已生成 - 控制点数: %zu, 阶数: %d, 路径点数: %zu, 起点(%.3f, %.3f), 终点(%.3f, %.3f)",
              vertices.size(), degree, spline_path.poses.size(), start_x, start_y, end_x, end_y);

  // 调用基类的 setPlan 方法
  return setPlan(spline_path);
}

bool LQRCurveController::setPlanForEllipse(double center_x, double center_y,
                                            double major_axis_x, double major_axis_y,
                                            double ratio, double rotation,
                                            double start_angle, double end_angle,
                                            const geometry_msgs::msg::PoseStamped& robot_pose)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化，请先调用 initialize()");
    return false;
  }

  // 参数验证
  if (ratio <= 0.0 || ratio > 1.0)
  {
    RCLCPP_ERROR(get_logger(), "Ellipse路径比例参数无效（应在0-1之间）: %.3f", ratio);
    return false;
  }

  // 重置状态
  reset();
  updateParameters("/config/lqr_curve.yaml");

  // 计算椭圆的长轴和短轴长度
  double a = std::sqrt(major_axis_x * major_axis_x + major_axis_y * major_axis_y);  // 长轴长度
  double b = a * ratio;  // 短轴长度

  // 1. 计算机器人在椭圆上的投影点角度
  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double projection_angle = projectPointToEllipse(robot_x, robot_y, center_x, center_y, a, b, rotation);

  RCLCPP_INFO(get_logger(), "机器人位置(%.3f, %.3f)在椭圆上的投影角度: %.2f度",
              robot_x, robot_y, projection_angle * 180.0 / M_PI);

  // 2. 路径起始角度就是机器人的投影点（机器人已经在偏移位置）
  double actual_start_angle = projection_angle;

  // 3. 估算椭圆周长（用于后续路径点生成）
  double h = std::pow((a - b) / (a + b), 2);
  double perimeter_approx = M_PI * (a + b) * (1 + 3 * h / (10 + std::sqrt(4 - 3 * h)));

  // 4. 确定角度范围和路径类型
  double target_angle_range = end_angle - start_angle;
  if (target_angle_range < 0)
  {
    target_angle_range += 2.0 * M_PI;
  }

  // 类似圆形路径，路径总角度 = 目标角度 + 1.5π（用于平滑启停）
  double path_total_angle = target_angle_range + 0.5 * M_PI;

  // 5. 判断路径类型（完整椭圆 vs 椭圆弧）
  constexpr double kEllipseThreshold = 1.95 * M_PI;
  bool is_complete_ellipse = (target_angle_range >= kEllipseThreshold);

  RCLCPP_INFO(get_logger(), "路径类型: %s (目标角度: %.2f度, 路径总角度: %.2f度)",
              is_complete_ellipse ? "完整椭圆" : "椭圆弧",
              target_angle_range * 180.0 / M_PI,
              path_total_angle * 180.0 / M_PI);

  // 6. 生成椭圆路径
  nav_msgs::msg::Path ellipse_path;
  ellipse_path.header.frame_id = robot_pose.header.frame_id.empty() ?
                                  std::string("map") : robot_pose.header.frame_id;
  ellipse_path.header.stamp = this->now();

  // 生成入口点（机器人投影点，用于航向预对准）
  geometry_msgs::msg::PoseStamped entry_pose = robot_pose;

  // 计算入口点在椭圆上的切线方向
  double x_ellipse_entry = a * std::cos(projection_angle);
  double y_ellipse_entry = b * std::sin(projection_angle);

  // 切线方向（参数方程的导数）
  double dx_dt_entry = -a * std::sin(projection_angle);
  double dy_dt_entry = b * std::cos(projection_angle);

  // 应用旋转
  double tangent_x = dx_dt_entry * std::cos(rotation) - dy_dt_entry * std::sin(rotation);
  double tangent_y = dx_dt_entry * std::sin(rotation) + dy_dt_entry * std::cos(rotation);
  double entry_yaw = std::atan2(tangent_y, tangent_x);

  tf2::Quaternion entry_q;
  entry_q.setRPY(0, 0, entry_yaw);
  entry_pose.pose.orientation = tf2::toMsg(entry_q);

  ellipse_path.poses.push_back(entry_pose);

  // 7. 生成路径点（从实际起始角度开始）
  // 以"弧长间距"决定采样密度：点间距固定 0.003m（3mm）
  constexpr double kPointSpacingMeters = 0.003;
  double arc_length_total = perimeter_approx * (path_total_angle / (2.0 * M_PI));
  size_t num_segments = static_cast<size_t>(std::ceil(arc_length_total / kPointSpacingMeters));
  num_segments = std::max<size_t>(num_segments, 1);

  constexpr size_t kMaxSegments = 20000;
  if (num_segments > kMaxSegments)
  {
    RCLCPP_WARN(get_logger(),
                "椭圆采样点过多(%zu)，已限制到 %zu 段",
                num_segments, kMaxSegments);
    num_segments = kMaxSegments;
  }

  double angle_step = path_total_angle / static_cast<double>(num_segments);

  // 跳过入口处一小段，减少"入口过密"对控制的干扰
  const size_t start_idx = (num_segments > 5) ? 5 : 1;

  for (size_t idx = 0; idx <= num_segments; ++idx)
  {
    double theta = actual_start_angle + static_cast<double>(idx) * angle_step;

    // 参数方程：椭圆上的点（未旋转）
    double x_local = a * std::cos(theta);
    double y_local = b * std::sin(theta);

    // 应用旋转
    double x_rotated = x_local * std::cos(rotation) - y_local * std::sin(rotation);
    double y_rotated = x_local * std::sin(rotation) + y_local * std::cos(rotation);

    // 平移到椭圆中心
    double x_global = center_x + x_rotated;
    double y_global = center_y + y_rotated;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = ellipse_path.header;
    pose.pose.position.x = x_global;
    pose.pose.position.y = y_global;
    pose.pose.position.z = robot_pose.pose.position.z;

    // 计算切线方向作为朝向
    double dx_dt = -a * std::sin(theta) * std::cos(rotation) - b * std::cos(theta) * std::sin(rotation);
    double dy_dt = -a * std::sin(theta) * std::sin(rotation) + b * std::cos(theta) * std::cos(rotation);
    double tangent_yaw = std::atan2(dy_dt, dx_dt);

    tf2::Quaternion q;
    q.setRPY(0, 0, tangent_yaw);
    pose.pose.orientation = tf2::toMsg(q);

    ellipse_path.poses.push_back(pose);
  }

  RCLCPP_INFO(get_logger(),
              "椭圆路径已生成 - 点数: %zu, 中心(%.3f, %.3f), 长轴=%.3fm, 短轴=%.3fm, "
              "旋转=%.1f度, 起始角度(投影点)=%.1f度",
              ellipse_path.poses.size(), center_x, center_y, a, b,
              rotation * 180.0 / M_PI,
              actual_start_angle * 180.0 / M_PI);

  // 先调用 setPlan()（内部不会调用 reset()，因为已注释）
  bool result = setPlan(ellipse_path);

  // 保存椭圆路径特有的参数
  ellipse_a_ = a;
  ellipse_b_ = b;
  ellipse_center_x_ = center_x;
  ellipse_center_y_ = center_y;
  ellipse_rotation_ = rotation;

  // 计算椭圆周长（使用拉马努金第二近似公式）
  // L ≈ π * [3(a+b) - sqrt((3a+b)(a+3b))]
  ellipse_perimeter_ = M_PI * (3.0 * (a + b) - std::sqrt((3.0 * a + b) * (a + 3.0 * b)));

  // 计算目标弧长（只计算用户指定的角度范围，不包括延长的缓冲部分）
  target_arc_length_ = ellipse_perimeter_ * (target_angle_range / (2.0 * M_PI));

  RCLCPP_INFO(get_logger(), "椭圆路径类型: %s (目标角度: %.2f°, 目标弧长: %.3fm, 周长: %.3fm)",
              is_complete_ellipse ? "完整椭圆" : "椭圆弧",
              target_angle_range * 180.0 / M_PI,
              target_arc_length_,
              ellipse_perimeter_);

  return result;
}

// ============================================================================
// 椭圆路径终点判定（基于距离）
// ============================================================================

bool LQRCurveController::updateAccumulatedDistance(double current_x, double current_y)
{
  // 如果椭圆参数未设置（长轴为0），说明不是椭圆路径，跳过判定
  if (ellipse_a_ <= 1e-6)
  {
    return false;
  }

  if (!last_position_initialized_)
  {
    last_x_ = current_x;
    last_y_ = current_y;
    last_position_initialized_ = true;
    accumulated_distance_ = 0.0;
    RCLCPP_INFO(get_logger(), "椭圆路径跟踪开始，初始位置: (%.3f, %.3f)", current_x, current_y);
    return false;  // 初始化阶段不判定为完成
  }

  // 计算距离增量
  double dx = current_x - last_x_;
  double dy = current_y - last_y_;
  double delta_distance = std::hypot(dx, dy);

  // 累计距离（避免倒退时累计负值）
  if (delta_distance > 1e-6)  // 过滤噪声
  {
    accumulated_distance_ += delta_distance;
  }

  // 更新上次位置，用于下次计算
  last_x_ = current_x;
  last_y_ = current_y;

  constexpr double print_start_delay = 1.0;  // 喷码机出墨延时 (秒)
  constexpr double arc_length_for_start = 0.4;  // 期望开始打印的弧长 0.4m
  const double quarter_perimeter = ellipse_perimeter_ / 4.0;  // 四分之一周长

  // 如果期望距离超过四分之一周长，则取四分之一周长
  const double desired_start_distance = std::min(arc_length_for_start, quarter_perimeter);

  // 计算延时补偿距离：延时期间机器人行驶的距离
  const double start_lead_distance = params_.v_max * print_start_delay;

  // 开始打印的触发距离（提前触发以补偿延时）
  const double start_trigger_distance = std::max(0.0, desired_start_distance - start_lead_distance);

  if (accumulated_distance_ > start_trigger_distance)
  {
    if (!print_window_initialized_)
    {
      // 记录触发"开始打印"信号时的累计距离
      start_print_distance_ = accumulated_distance_;

      // 计算有效开始喷印距离（考虑延时后的实际喷印位置）
      const double effective_start_print_distance = start_print_distance_ + start_lead_distance;

      // 定义停止打印窗口参数
      constexpr double kStopArcLengthMeters = 0.03;        // 停止窗口弧长：3cm
      constexpr double kClosureCompensationMeters = 0.06;  // 闭合补偿弧长：6cm

      // 计算停止打印结束距离
      // 只算目标弧长（不包括延长的缓冲部分）
      stop_print_end_distance_ = effective_start_print_distance + target_arc_length_ + kClosureCompensationMeters;

      // 停止打印开始距离 = 结束距离 - 停止窗口长度
      stop_print_start_distance_ = stop_print_end_distance_ - kStopArcLengthMeters;

      print_window_initialized_ = true;  // 标记窗口已初始化

      RCLCPP_INFO(get_logger(),
                  "打印窗口初始化 - 触发距离: %.3fm, 有效开始: %.3fm, 停止开始: %.3fm, 停止结束: %.3fm",
                  start_print_distance_, effective_start_print_distance,
                  stop_print_start_distance_, stop_print_end_distance_);
    }
  }

  if (print_window_initialized_)
  {
    // 打印窗口已初始化，根据当前累计距离判断打印状态
    if (accumulated_distance_ < stop_print_start_distance_)
    {
      // 情况1：累计距离 < 停止打印开始距离
      // → 应该继续打印
      start_print = true;
      stop_print = false;
    }
    else
    {
      // 情况2：累计距离 >= 停止打印开始距离
      // → 应该停止打印（进入3cm停止窗口）
      start_print = false;
      stop_print = true;
    }

    // 判断椭圆路径是否完成
    if (accumulated_distance_ >= stop_print_end_distance_)
    {
      // 累计距离达到或超过停止打印结束距离，椭圆路径完成
      return true;
    }
  }
  else if (accumulated_distance_ >= ellipse_perimeter_ * 1.3)
  {
    // 若打印窗口未初始化（异常情况），但累计距离已超过周长的1.3倍
    // 则强制认为路径完成，避免无限循环
    start_print = false;
    stop_print = true;
    RCLCPP_WARN(get_logger(), "打印窗口未初始化但距离已超限，强制完成");
    return true;
  }

  // 路径未完成，继续跟踪
  return false;
}

}  // namespace follow_controller
}  // namespace xline
