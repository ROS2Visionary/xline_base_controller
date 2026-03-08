#include "xline_follow_controller/lqr_follow/lqr_circle_controller.hpp"
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

LQRCircleController::LQRCircleController()
  : BaseFollowController("lqr_circle_controller")
  , K1_(0.0)
  , K2_(0.0)
  , initialized_(false)
  , goal_reached_(false)
  , last_nearest_idx_(0)
  , need_yaw_prealign_(true)
  , yaw_prealign_done_(false)
  , target_yaw_(0.0)
  , wait_duration_(0.2)
  , waiting_(false)
  , integral_e_y_(0.0)
  , last_e_y_(0.0)
  , last_e_y_initialized_(false)
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

  updateParameters("/config/lqr_circle.yaml");
  initializeFilters();
  initialize();

  wait_start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(get_logger(), "LQRFollowController 创建完成");
}

void LQRCircleController::initialize()
{
  // 计算控制周期
  params_.control_period = 1.0 / params_.control_frequency;

  // 计算初始增益（使用默认速度）
  computeLQRGains(params_.v_max);

  // 初始化平滑限制比例（喷墨前值）
  limit_ratio_smoothed_ = params_.feedback_limit_ratio_before_print;

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

void LQRCircleController::updateParameters(const std::string& config_path)
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

    // 加载原地旋转控制参数（梯形速度规划）
    params_.rotation_max_w = parser.getParameter<double>("rotation.max_w");
    params_.rotation_min_w = parser.getParameter<double>("rotation.min_w");
    params_.rotation_decel  = parser.getParameter<double>("rotation.decel");

    // 加载反馈限制参数（分段控制）
    // 优先加载分段参数，如果不存在则使用统一参数（向后兼容）
    if (parser.hasParameter("feedback.limit_ratio_before_print") &&
        parser.hasParameter("feedback.limit_ratio_after_print"))
    {
      params_.feedback_limit_ratio_before_print =
          parser.getParameter<double>("feedback.limit_ratio_before_print");
      params_.feedback_limit_ratio_after_print =
          parser.getParameter<double>("feedback.limit_ratio_after_print");

      RCLCPP_INFO(get_logger(),
                  "使用分段反馈限制: 喷墨前=%.1f%%, 喷墨后=%.1f%%",
                  params_.feedback_limit_ratio_before_print * 100.0,
                  params_.feedback_limit_ratio_after_print * 100.0);
    }
    else if (parser.hasParameter("feedback.limit_ratio"))
    {
      // 向后兼容：统一参数
      params_.feedback_limit_ratio = parser.getParameter<double>("feedback.limit_ratio");
      params_.feedback_limit_ratio_before_print = params_.feedback_limit_ratio;
      params_.feedback_limit_ratio_after_print = params_.feedback_limit_ratio;

      RCLCPP_WARN(get_logger(),
                  "使用统一反馈限制(已弃用): %.1f%%，建议使用分段参数",
                  params_.feedback_limit_ratio * 100.0);
    }

    params_.feedback_min_limit = parser.getParameter<double>("feedback.min_limit");

    // 加载积分路径独立限制（可选，不存在则使用默认值 0.20）
    if (parser.hasParameter("feedback.int_limit_ratio"))
    {
      params_.int_limit_ratio = parser.getParameter<double>("feedback.int_limit_ratio");
    }

    // 加载 e_y 跳变检测阈值（可选，0 = 关闭）
    if (parser.hasParameter("feedback.max_ey_jump_m"))
    {
      params_.max_ey_jump_m = parser.getParameter<double>("feedback.max_ey_jump_m");
    }

    // 加载 e_theta 低通滤波系数（可选，1.0 = 关闭）
    if (parser.hasParameter("feedback.e_theta_lowpass_alpha"))
    {
      params_.e_theta_lowpass_alpha = parser.getParameter<double>("feedback.e_theta_lowpass_alpha");
    }

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

    // 加载半径自适应速度调度参数（可选）
    if (parser.hasParameter("velocity_schedule.enable_radius_schedule"))
    {
      params_.enable_radius_schedule =
          parser.getParameter<bool>("velocity_schedule.enable_radius_schedule");
    }
    if (parser.hasParameter("velocity_schedule.radius_threshold"))
    {
      params_.radius_threshold =
          parser.getParameter<double>("velocity_schedule.radius_threshold");
    }
    if (parser.hasParameter("velocity_schedule.radius_step"))
    {
      params_.radius_step =
          parser.getParameter<double>("velocity_schedule.radius_step");
    }
    if (parser.hasParameter("velocity_schedule.velocity_step"))
    {
      params_.velocity_step =
          parser.getParameter<double>("velocity_schedule.velocity_step");
    }

    // 加载优化元信息（可选，用于调参记录）
    if (parser.hasParameter("optimization.id"))
    {
      params_.optimization.id = parser.getParameter<std::string>("optimization.id");
    }
    if (parser.hasParameter("optimization.parent_batch_id"))
    {
      params_.optimization.parent_batch_id =
          parser.getParameter<std::string>("optimization.parent_batch_id");
    }
    if (parser.hasParameter("optimization.change_note"))
    {
      params_.optimization.change_note =
          parser.getParameter<std::string>("optimization.change_note");
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

bool LQRCircleController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
  if (orig_global_plan.poses.empty())
  {
    RCLCPP_ERROR(get_logger(), "收到空路径，无法设置计划");
    return false;
  }

  // 重置状态
  reset();

  updateParameters("/config/lqr_circle.yaml");

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

void LQRCircleController::setAngleRange(double start_angle, double end_angle)
{

  target_total_angle_ = std::abs(end_angle - start_angle);

  path_total_angle_ = target_total_angle_ + 1.5 * M_PI;

  RCLCPP_INFO(get_logger(), "设置圆形角度范围: [%.2f, %.2f], 总角度: %.2f rad (%.1f°)",
              start_angle, end_angle, target_total_angle_,
              target_total_angle_ * 180.0 / M_PI);
}

bool LQRCircleController::setPlanForCircle(double circle_center_x, double circle_center_y,
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

  nav_msgs::msg::Path circle_path =
      generateCirclePath(circle_center_x, circle_center_y, circle_radius, robot_pose);

  RCLCPP_INFO(get_logger(), "圆形路径已生成 - 圆心: (%.3f, %.3f), 半径: %.3f m, 总角度: %.2f rad, 点数: %zu",
              circle_center_x, circle_center_y, circle_radius, path_total_angle_,
              circle_path.poses.size());

  // 先调用 setPlan()（内部会调用 reset() 清空所有状态）
  bool result = setPlan(circle_path);

  // 再保存圆形路径特有的参数（在 reset() 之后保存，避免被清零）
  circle_radius_ = circle_radius;

  // 保存圆心坐标（用于径向误差计算）
  circle_center_x_ = circle_center_x;
  circle_center_y_ = circle_center_y;

  // ── 修复1：覆盖曲率为精确值 1/R ──
  // setPlan() 内部的 computePathCurvature() 使用三点差分（近似），
  // 对已知圆弧直接赋值解析解，消除数值误差，前馈精度更高
  {
    const double exact_curvature = 1.0 / circle_radius;
    for (auto& pt : path_)
    {
      pt.curvature = exact_curvature;
    }
  }

  // ── 修复2：预计算喷墨触发参数（使用实际调度速度，而非 v_max）──
  // omega_for_delay 此前错误地使用 v_max，导致提前量偏大（小半径误差最大2×）
  {
    constexpr double print_start_delay    = 1.0;   // 喷码机出墨延时 (秒)
    constexpr double arc_length_for_start = 0.4;   // 期望入圆弧长 (米)
    const double v_scheduled = computeScheduledVelocity(circle_radius);
    const double omega_at_speed = v_scheduled / circle_radius;  // 实际 ω = v/R
    const double angle_from_arc = arc_length_for_start / circle_radius;
    const double desired_start_angle = std::min(angle_from_arc, 0.5 * M_PI);
    start_lead_angle_   = omega_at_speed * print_start_delay;
    start_trigger_angle_ = std::max(0.0, desired_start_angle - start_lead_angle_);
    limit_ratio_smoothed_ = params_.feedback_limit_ratio_before_print;
    RCLCPP_INFO(get_logger(),
                "喷墨触发预计算: v_sched=%.3fm/s, trigger=%.3frad(%.1f°), lead=%.3frad(%.1f°)",
                v_scheduled, start_trigger_angle_, start_trigger_angle_ * 180.0 / M_PI,
                start_lead_angle_, start_lead_angle_ * 180.0 / M_PI);
  }

  // 准备数据采集槽位（在 reset() 之后调用，避免被清零）
  prepareCircleSlot();

  // 根据 circle_total_angle_ 自动判断路径类型
  // 如果总角度 >= 1.95π，视为完整圆形；否则视为圆弧
  constexpr double kCircleThreshold = 1.95 * M_PI;
  if (target_total_angle_ >= kCircleThreshold)
  {
    path_type_ = PathType::CIRCLE;
    RCLCPP_INFO(get_logger(), "路径类型: 完整圆形 (总角度: %.2f°)",
                target_total_angle_ * 180.0 / M_PI);
  }
  else
  {
    path_type_ = PathType::ARC;
    RCLCPP_INFO(get_logger(), "路径类型: 圆弧 (总角度: %.2f°)",
                target_total_angle_ * 180.0 / M_PI);
  }

  return result;
}

nav_msgs::msg::Path LQRCircleController::generateCirclePath(
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

  const double total_angle = std::max(path_total_angle_, 1e-6);

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

bool LQRCircleController::computeVelocityCommands(
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

  // 计算真实控制周期 dt（用于数据采集计时）
  auto ctrl_now = std::chrono::steady_clock::now();
  double ctrl_dt = params_.control_period;
  if (last_control_time_initialized_)
  {
    ctrl_dt = std::chrono::duration_cast<std::chrono::duration<double>>(
        ctrl_now - last_control_time_).count();
    ctrl_dt = std::clamp(ctrl_dt, 0.005, 0.2);
  }
  last_control_time_ = ctrl_now;
  last_control_time_initialized_ = true;

  // 根据圆弧半径计算调度线速度（小半径慢速，大半径适当加速）
  // 使用期望速度（非测量值）计算前馈，避免测量噪声影响前馈稳定性
  double current_v = computeScheduledVelocity(circle_radius_);

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

  // 0.5. 检查是否在等待状态（对齐完成后的延时）
  if (waiting_)
  {
    if (handleWaitingState(cmd_vel))
    {
      return true;  // 等待期间返回 true，继续等待
    }
  }

  // 1. 找到最近的路径点
  size_t nearest_idx = findNearestPoint(current_x, current_y);

  // 2. 计算前瞻距离
  double lookahead_dist = getLookaheadDistance(current_v);

  // 3. 获取前瞻参考点（带插值）
  PathPointWithCurvature ref = findLookaheadPoint(nearest_idx, lookahead_dist);

  // 4. 计算 Frenet 误差（e_y 用于 CSV 记录的 cross_track_mm）
  double e_y, e_theta;
  computeErrors(current_x, current_y, current_theta, ref, e_y, e_theta);

  // 4.1. 对圆弧使用精确切线方向重算 e_theta（消除前瞻偏置）
  // 前瞻点切线相比当前位置精确切线存在 L/(2R) 的偏置：
  //   R=0.3m，L=18mm → 偏置 = 0.03 rad = 1.7°（显著）
  //   R=1.0m，L=18mm → 偏置 = 0.009 rad（可忽略）
  // 精确切线 = atan2(ry - cy, rx - cx) + π/2（逆时针圆弧）
  if (circle_radius_ > 0.0)
  {
    const double angle_to_robot = std::atan2(current_y - circle_center_y_,
                                             current_x - circle_center_x_);
    const double exact_tangent = angle_to_robot + M_PI / 2.0;  // 逆时针切线方向
    e_theta = normalizeAngle(current_theta - exact_tangent);

    // e_theta 低通滤波：过滤定位系统高频航向噪声（DC/真实误差完全保留）
    // alpha=0.82：tau≈280ms；对50ms宽噪声脉冲衰减至18%；对稳态航向偏差无影响
    if (params_.e_theta_lowpass_alpha < 1.0)
    {
      e_theta_filtered_ = params_.e_theta_lowpass_alpha * e_theta_filtered_
                        + (1.0 - params_.e_theta_lowpass_alpha) * e_theta;
      e_theta = e_theta_filtered_;
    }
  }

  // 4.5. 计算径向误差（控制用）+ e_y 跳变检测
  //
  // 【修复3：使用径向误差替代 Frenet e_y 作为控制输入】
  // Frenet e_y 测量的是机器人相对于"前瞻点"的横向偏差，存在前瞻稀释效应：
  //   前瞻距离 18mm 时，实际 5mm 径向误差被投影压缩为约 4mm
  // 径向误差 = dist(robot, center) - R，与精度指标（p90_radial_mm）完全一致
  //
  // 【符号约定】Frenet 坐标系中，路径左侧（逆时针=内侧）为正：
  //   偏内（dist < R）→ Frenet e_y > 0
  //   偏外（dist > R）→ Frenet e_y < 0
  // radial_err = dist - R，偏外为正，与 Frenet e_y 符号相反！
  // 因此必须取负号以保持控制方向正确（偏外→ω增大→转向内侧）
  double radial_err_m = 0.0;   // 原始几何误差（偏外为正，用于 CSV 记录，不受跳变限制影响）
  double e_y_ctrl = e_y;       // 控制输入（默认退化为 Frenet，下方对圆弧覆盖）
  if (circle_radius_ > 0.0)
  {
    const double dist_to_center = std::hypot(current_x - circle_center_x_,
                                             current_y - circle_center_y_);
    radial_err_m = dist_to_center - circle_radius_;
    // 取负号：使控制输入符号与 Frenet e_y 一致（偏外→负→-K1×负→正反馈→ω增大）
    e_y_ctrl = -radial_err_m;
  }

  // 跳变检测：单周期径向误差变化超过阈值 → 限幅 + 冻结积分
  bool freeze_integral = false;
  if (params_.max_ey_jump_m > 0.0 && last_e_y_initialized_)
  {
    const double ey_delta = e_y_ctrl - last_e_y_;
    if (std::abs(ey_delta) > params_.max_ey_jump_m)
    {
      e_y_ctrl = last_e_y_ + std::copysign(params_.max_ey_jump_m, ey_delta);
      freeze_integral = true;
      RCLCPP_WARN(get_logger(),
                  "e_y 跳变检测: 径向Δ=%.1fmm > 阈值%.1fmm，限幅并冻结积分",
                  ey_delta * 1000.0, params_.max_ey_jump_m * 1000.0);
    }
  }
  last_e_y_ = e_y_ctrl;
  last_e_y_initialized_ = true;

  // 5. 计算LQR增益（基于当前速度）
  computeLQRGains(current_v);

  // 6. 前馈控制 ω_ff = v × κ（充分利用精确曲率 1/R，是圆弧跟踪的核心）
  double omega_ff = current_v * ref.curvature;

  // 7. LQR比例反馈（使用径向误差，消除前瞻稀释效应）
  double omega_fb = -K1_ * e_y_ctrl - K2_ * e_theta;

  // 8. 积分项（使用径向误差，跳变帧冻结）
  // 使用实际 ctrl_dt 而非固定 control_period，保证控制周期波动时积分量准确
  // 衰减也按实际 dt 计算：decay_dt = decay^(dt/T) = exp(dt/T × ln(decay))
  double omega_i = 0.0;
  if (params_.enable_integral && !freeze_integral)
  {
    const double decay_dt = std::pow(params_.integral_decay, ctrl_dt / params_.control_period);
    integral_e_y_ = decay_dt * integral_e_y_ + e_y_ctrl * ctrl_dt;
    integral_e_y_ = std::clamp(integral_e_y_, -params_.integral_max / params_.Ki,
                                params_.integral_max / params_.Ki);
    omega_i = -params_.Ki * integral_e_y_;
  }

  // 9. 双路反馈限制（解耦比例平滑性与积分修正力）
  //
  // 【比例路径】tanh 软饱和：保证 omega 在 ff 附近连续平滑变化
  //   - 喷前：宽松（快速收敛），喷后：收窄（趋向 ff±5% 评判标准）
  //   - 切换时一阶低通平滑过渡，防止 omega 阶跃
  //
  // 【积分路径】独立 clamp：补偿系统性稳态偏差（可超过比例限制）
  //   - 积分天然平滑（每周期缓慢变化），不影响比例路径的瞬时平滑性

  // 【修复4：平滑过渡比例限制，防止 start_print 切换时 omega 阶跃】
  // 一阶低通，时间常数 0.3s（约 5 个控制周期）
  {
    const double target_ratio = start_print
        ? params_.feedback_limit_ratio_after_print
        : params_.feedback_limit_ratio_before_print;
    const double lp_alpha = std::exp(-ctrl_dt / 0.3);
    limit_ratio_smoothed_ = lp_alpha * limit_ratio_smoothed_ + (1.0 - lp_alpha) * target_ratio;
  }
  const double current_limit_ratio = limit_ratio_smoothed_;

  // 比例路径限制（前馈 × 比例）
  double prop_limit = std::abs(omega_ff) * current_limit_ratio;
  if (params_.feedback_min_limit > 0.0 && prop_limit < params_.feedback_min_limit)
  {
    prop_limit = params_.feedback_min_limit;
  }

  // 比例路径：tanh 软饱和（核心平滑机制）
  double omega_prop = 0.0;
  if (prop_limit > 1e-6)
  {
    omega_prop = std::tanh(omega_fb / prop_limit) * prop_limit;
  }

  // 积分路径：独立 clamp（稳态扰动修正，不受比例限制约束）
  double int_limit = std::abs(omega_ff) * params_.int_limit_ratio;
  double omega_int = std::clamp(omega_i, -int_limit, int_limit);

  // 合并两路反馈（用于数据记录和最终控制）
  const double feedback_limit = prop_limit;  // 记录用：与喷前后状态对应的比例限制
  double omega_correction = omega_prop + omega_int;

  // 10. 总控制量 = 前馈 + 双路反馈
  double omega = omega_ff + omega_correction;

  // 11. 应用角速度和角加速度限幅
  const double omega_before_limits = omega;  // 保存限幅前值（调试用）
  omega = applyLimits(omega);

  // 12. 对于圆形路径，检查是否完成
  bool completed = updateAccumulatedAngle(current_theta);

  // 13. 数据采集（在跟踪阶段，即 yaw 初始化之后；完成帧也记录）
  if (last_yaw_initialized_)
  {
    tracking_elapsed_time_s_ += ctrl_dt;
    if (tracking_elapsed_time_s_ + 1e-9 >= next_tracking_sample_time_s_)
    {
      // radial_err_m 已在控制律步骤4.5中计算（真实几何误差，未受跳变限制影响）
      // e_y 保留 Frenet 值（cross_track_mm 列，用于分析脚本向后兼容）
      sampleTrackingData(current_x, current_y,
                         e_y, radial_err_m,
                         current_v, omega,
                         ref.theta, e_theta,
                         omega_ff, omega_fb, omega_correction, omega_i,
                         omega_before_limits, feedback_limit,
                         K1_, K2_,
                         nearest_idx, accumulated_angle_,
                         start_print, integral_e_y_);
    }
  }

  if (completed)
  {
    goal_reached_ = true;
    exportTrackingMetrics();

    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    RCLCPP_INFO(get_logger(), "圆形路径完成 - 累计角度: %.4f rad (%.2f°)",
                  accumulated_angle_, accumulated_angle_ * 180.0 / M_PI);
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
                "LQR控制: 横向误差=%.1fmm, 航向角误差=%.2f°, 前馈角速度=%.3f, 反馈角速度=%.3f(限幅±%.3f, %.0f%%), 总角速度=%.3f, 曲率=%.3f, 喷墨=%s",
                e_y * 1000, e_theta * 180 / M_PI,
                omega_ff, omega_correction, feedback_limit, current_limit_ratio * 100.0,
                omega, ref.curvature, start_print ? "是" : "否");
  }

  // 更新栅格图可视化
  updateGridMapIfNeeded(pose, ref);

  return true;
}

bool LQRCircleController::isGoalReached()
{
  if (goal_reached_)
  {
    return true;
  }

  // 简化实现：检查是否接近最后一个路径点
  // 实际应用中应在computeVelocityCommands中检查
  return false;
}

bool LQRCircleController::cancel()
{
  exportTrackingMetrics();  // 取消时也导出已采集的数据
  reset();
  RCLCPP_INFO(get_logger(), "LQR控制器已取消");
  return true;
}

// ============================================================================
// LQR 特有接口
// ============================================================================

void LQRCircleController::getGains(double& K1, double& K2) const
{
  K1 = K1_;
  K2 = K2_;
}

std::string LQRCircleController::getDebugInfo() const
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

void LQRCircleController::computeLQRGains(double v)
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

void LQRCircleController::computePathCurvature()
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

size_t LQRCircleController::findNearestPoint(double x, double y)
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

PathPointWithCurvature LQRCircleController::findLookaheadPoint(size_t nearest_idx,
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

double LQRCircleController::getLookaheadDistance(double speed)
{
  // 基于时间-速度模型计算前瞻距离
  double lookahead_dist = params_.lookahead_distance +
                          params_.lookahead_time * std::abs(speed);

  // 简单限幅（可根据需要添加 min/max 参数）
  return std::max(0.02, lookahead_dist);  // 最小20mm前瞻
}

void LQRCircleController::computeErrors(double current_x, double current_y,
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

double LQRCircleController::applyLimits(double omega)
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

double LQRCircleController::normalizeAngle(double angle)
{
  // 使用 fmod 实现 O(1) 复杂度，避免 while 循环
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0)
  {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

bool LQRCircleController::performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose,
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

    // 设置等待状态
    waiting_ = true;
    wait_start_time_ = std::chrono::steady_clock::now();

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

double LQRCircleController::calculateRotationVelocity(double angle_diff)
{
  // 梯形速度规划：制动速度 = sqrt(2 * decel * |e|)
  // 物理保证：在剩余角度内能以 rotation_decel 刹停，不过冲
  double omega = std::sqrt(2.0 * params_.rotation_decel * std::abs(angle_diff));
  omega = std::clamp(omega, params_.rotation_min_w, params_.rotation_max_w);
  return (angle_diff > 0.0) ? omega : -omega;
}

bool LQRCircleController::handleWaitingState(geometry_msgs::msg::TwistStamped& cmd_vel)
{
  auto current_time = std::chrono::steady_clock::now();
  double wait_time = std::chrono::duration_cast<std::chrono::duration<double>>(
      current_time - wait_start_time_).count();

  if (wait_time >= wait_duration_)
  {
    waiting_ = false;
    return false;
  }

  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  return true;
}

void LQRCircleController::reset()
{
  goal_reached_ = false;
  last_nearest_idx_ = 0;
  waiting_ = false;
  integral_e_y_ = 0.0;
  last_e_y_ = 0.0;
  last_e_y_initialized_ = false;
  start_trigger_angle_ = 0.0;
  start_lead_angle_ = 0.0;
  limit_ratio_smoothed_ = params_.feedback_limit_ratio_before_print;
  last_omega_ = 0.0;
  debug_e_y_ = 0.0;
  debug_e_theta_ = 0.0;
  debug_omega_ff_ = 0.0;
  debug_omega_fb_ = 0.0;
  debug_ref_curvature_ = 0.0;
  debug_ref_index_ = 0;

  // 重置圆形路径参数
  path_type_ = PathType::ARC;  // 重置为默认类型
  circle_radius_ = 0.0;

  // 重置角度累计相关状态
  last_yaw_initialized_ = false;
  last_yaw_ = 0.0;
  accumulated_angle_ = 0.0;
  start_print = false;
  stop_print = false;
  print_window_initialized_ = false;
  start_print_angle_ = 0.0;
  stop_print_start_angle_ = 0.0;
  stop_print_end_angle_ = 0.0;

  // 重置位置滤波器（使用配置参数）
  sg_x_filter_.reset(params_.savgol_window, params_.savgol_order);
  sg_y_filter_.reset(params_.savgol_window, params_.savgol_order);
  h_x_filter.reset(params_.hampel_window, params_.hampel_k);
  h_y_filter.reset(params_.hampel_window, params_.hampel_k);

  // 重置数据采集状态（批次管理变量不重置，由 prepareCircleSlot 管理）
  tracking_error_abs_m_.clear();
  tracking_radial_err_m_.clear();
  tracking_angle_at_sample_.clear();
  tracking_sample_rows_.clear();
  tracking_metrics_exported_   = false;
  tracking_elapsed_time_s_     = 0.0;
  next_tracking_sample_time_s_ = 0.0;

  // 重置控制周期计时
  last_control_time_initialized_ = false;

  RCLCPP_DEBUG(get_logger(), "LQR控制器状态已重置");
}

void LQRCircleController::initializeFilters()
{
  // 初始化位置滤波器（从参数配置）
  sg_x_filter_ = SavitzkyGolayFilter(params_.savgol_window, params_.savgol_order);
  sg_y_filter_ = SavitzkyGolayFilter(params_.savgol_window, params_.savgol_order);
  h_x_filter = HampelFilter(params_.hampel_window, params_.hampel_k);
  h_y_filter = HampelFilter(params_.hampel_window, params_.hampel_k);
}

geometry_msgs::msg::PoseStamped LQRCircleController::filterRobotPose(
    const geometry_msgs::msg::PoseStamped& robot_pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header = robot_pose.header;
  current_pose.pose.orientation = robot_pose.pose.orientation;

  // 【修复5：去除 Savitzky-Golay 滤波，只保留 Hampel 异常值检测】
  // SG 窗口7点在18Hz下引入约165ms延迟（约11.5mm at v=0.07m/s），与前瞻距离15mm量级相当，
  // 导致控制器的"当前位置"实际滞后于真实位置，劣化精度。
  // Hampel 滤波器负责剔除定位系统的短暂跳变（异常值检测），延迟仅1帧（~55ms），可接受。
  // e_y 跳变检测（max_ey_jump_m）提供了额外的抗扰保护，弥补去掉 SG 的平滑损失。
  current_pose.pose.position.x = h_x_filter.filter(robot_pose.pose.position.x);
  current_pose.pose.position.y = h_y_filter.filter(robot_pose.pose.position.y);

  return current_pose;
}

// ============================================================================
// 栅格图可视化
// ============================================================================

void LQRCircleController::initializeGridMap(const nav_msgs::msg::Path& path)
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

cv::Point LQRCircleController::worldToGrid(double x, double y)
{
  int grid_x = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
  int grid_y = grid_map_.rows - static_cast<int>((y - grid_origin_y_) / grid_resolution_) - 1;
  return cv::Point(grid_x, grid_y);
}

void LQRCircleController::drawPathOnGrid(const nav_msgs::msg::Path& path,
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

void LQRCircleController::drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose)
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

void LQRCircleController::drawLookaheadPointOnGrid(const PathPointWithCurvature& lookahead_point)
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

void LQRCircleController::drawGridLines()
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

void LQRCircleController::saveGridMap()
{
  if (grid_map_.empty() || params_.grid_map_path.empty())
  {
    return;
  }

  std::string filename = params_.grid_map_path + "/lqr_path_tracking.png";
  cv::imwrite(filename, grid_map_);
}

void LQRCircleController::updateGridMapIfNeeded(const geometry_msgs::msg::PoseStamped& current_pose,
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

bool LQRCircleController::isPointInGrid(const cv::Point& pt)
{
  return pt.x >= 0 && pt.x < grid_map_.cols && pt.y >= 0 && pt.y < grid_map_.rows;
}


bool LQRCircleController::updateAccumulatedAngle(double current_yaw)
{

  if (!last_yaw_initialized_)
  {
    last_yaw_ = current_yaw;
    last_yaw_initialized_ = true;
    accumulated_angle_ = 0.0;
    RCLCPP_INFO(get_logger(), "圆形路径跟踪开始，初始航向角: %.2f", current_yaw);
    return false;  // 初始化阶段不判定为完成
  }

  double delta_yaw = current_yaw - last_yaw_;

  if (delta_yaw > M_PI)
  {
    delta_yaw -= 2.0 * M_PI;  // 修正正向跳变
  }
  else if (delta_yaw < -M_PI)
  {
    delta_yaw += 2.0 * M_PI;  // 修正负向跳变
  }

  if (delta_yaw > 0)
  {
    // 只累计"正向"旋转的角度，避免来回抖动时累计角度不合理地增大
    accumulated_angle_ += delta_yaw;
  }

  // 更新上次航向角，用于下次计算
  last_yaw_ = current_yaw;

  // 【修复2：使用预计算的触发参数（setPlanForCircle 时以实际调度速度计算，非 v_max）】
  if (accumulated_angle_ > start_trigger_angle_)
  {
    if (!print_window_initialized_)
    {
      // 记录触发"开始打印"信号时的累计角度
      start_print_angle_ = accumulated_angle_;

      // 有效开始喷印角度 = 触发角度 + 延时期间转过的角度（使用预计算的 start_lead_angle_）
      const double effective_start_print_angle = start_print_angle_ + start_lead_angle_;

      // 定义停止打印窗口参数
      constexpr double kStopArcLengthMeters = 0.03;        // 停止窗口弧长：3cm
      constexpr double kClosureCompensationMeters = 0.06;  // 闭合补偿弧长：6cm

      // 将弧长转换为角度：θ = L / R
      // 停止窗口角度跨度（最多不超过2π）
      const double stop_delta_angle = std::min(2.0 * M_PI, kStopArcLengthMeters / circle_radius_);

      const double closure_comp_angle =
          std::min(2.0 * M_PI, kClosureCompensationMeters / circle_radius_);

    
      if (path_type_ == PathType::CIRCLE)
      {
        stop_print_end_angle_ = effective_start_print_angle + 2.0 * M_PI + closure_comp_angle;
      }

      if (path_type_ == PathType::ARC)
      {
        stop_print_end_angle_ = effective_start_print_angle + target_total_angle_ + closure_comp_angle;
      }
      

      stop_print_start_angle_ = stop_print_end_angle_ - stop_delta_angle;

      print_window_initialized_ = true;  // 标记窗口已初始化
      // RCLCPP_INFO(get_logger(),"stop_print_end_angle_: %.5f, stop_delta_angle: %.5f,stop_delta_angle: %.5f, closure_comp_angle: %.5f, circle_radius_: %.5f",stop_print_end_angle_,stop_delta_angle,closure_comp_angle,circle_radius_);
    }
  }


  if (print_window_initialized_)
  {
    // 打印窗口已初始化，根据当前累计角度判断打印状态
    if (accumulated_angle_ < stop_print_start_angle_)
    {
      // 情况1：累计角度 < 停止打印开始角度
      // → 应该继续打印
      start_print = true;
      stop_print = false;
    }
    else
    {
      // 情况2：累计角度 >= 停止打印开始角度
      // → 应该停止打印（进入3cm停止窗口）
      start_print = false;
      stop_print = true;
    }

    // 判断圆形路径是否完成
    if (accumulated_angle_ >= stop_print_end_angle_)
    {
      // 累计角度达到或超过停止打印结束角度，圆形路径完成
      return true;
    }
  }
  else if (accumulated_angle_ >= 2.63 * M_PI)
  {

    // 若打印窗口未初始化（异常情况），但累计角度已超过 2.63π (约 474°)
    // 则强制认为路径完成，避免无限循环
    start_print = false;
    stop_print = true;
    return true;
  }

  // 路径未完成，继续跟踪
  return false;
}

// ============================================================================
// 数据采集与导出
// ============================================================================

void LQRCircleController::prepareCircleSlot()
{
  constexpr int kBatchSize = 8;  // 每批最多记录 8 圈（支持 ≤8 个不同半径）

  if (!circle_batch_initialized_ || circle_count_in_batch_ >= kBatchSize)
  {
    circle_batch_initialized_ = true;
    circle_count_in_batch_    = 0;

    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    circle_batch_id_ = std::to_string(now_ms);

    // 清空并重建 circle_tracking_latest 目录
    const std::string latest_dir = getTrackingRecordDir() + "/circle_tracking_latest";
    std::lock_guard<std::mutex> lock(file_mutex_);
    std::error_code ec;
    if (std::filesystem::exists(latest_dir))
    {
      std::filesystem::remove_all(latest_dir, ec);
    }
    std::filesystem::create_directories(latest_dir, ec);

    RCLCPP_INFO(get_logger(), "初始化圆弧验证批次: batch_id=%s", circle_batch_id_.c_str());
  }

  circle_slot_ = circle_count_in_batch_ + 1;
  circle_count_in_batch_ += 1;

  RCLCPP_INFO(get_logger(), "设置圆弧采集槽位: %d/%d, 半径=%.3fm",
              circle_slot_, kBatchSize, circle_radius_);
}

void LQRCircleController::sampleTrackingData(
    double robot_x, double robot_y,
    double cross_track_m, double radial_err_m,
    double linear_speed, double angular_cmd,
    double ref_theta, double e_theta,
    double omega_ff, double omega_fb_raw,
    double omega_correction, double omega_i,
    double omega_before_limits, double feedback_limit,
    double k1, double k2,
    size_t nearest_idx, double accumulated_angle_rad,
    bool is_printing, double integral_state)
{
  // 记录误差用于统计
  tracking_error_abs_m_.push_back(std::abs(cross_track_m));
  tracking_radial_err_m_.push_back(radial_err_m);
  tracking_angle_at_sample_.push_back(accumulated_angle_rad);

  // 生成 CSV 行
  const double nan_v = std::numeric_limits<double>::quiet_NaN();
  (void)nan_v;

  // 计算真实几何角度：atan2(robot_y - center_y, robot_x - center_x) 归一化到 [0, 2π)
  const double real_angle_rad = [&]() {
    double a = std::atan2(robot_y - circle_center_y_, robot_x - circle_center_x_);
    if (a < 0.0) a += 2.0 * M_PI;
    return a;
  }();

  std::ostringstream row;
  row << std::fixed << std::setprecision(6)
      << tracking_elapsed_time_s_           << ","  // t_s
      << robot_x                            << ","  // x_m
      << robot_y                            << ","  // y_m
      << circle_center_x_                   << ","  // circle_center_x_m
      << circle_center_y_                   << ","  // circle_center_y_m
      << (cross_track_m * 1000.0)           << ","  // cross_track_mm (Frenet e_y，带符号)
      << (radial_err_m  * 1000.0)           << ","  // radial_err_mm (正=偏外，负=偏内)
      << accumulated_angle_rad              << ","  // accum_angle_rad
      << real_angle_rad                     << ","  // real_angle_rad (真实几何角度，[0,2π))
      << circle_radius_                     << ","  // circle_radius_m
      << linear_speed                       << ","  // linear_speed_mps
      << angular_cmd                        << ","  // angular_cmd_rps (最终指令)
      << ref_theta                          << ","  // ref_theta_rad
      << e_theta                            << ","  // e_theta_rad
      << omega_ff                           << ","  // omega_ff_rps
      << omega_fb_raw                       << ","  // omega_fb_raw_rps (限幅前)
      << omega_correction                   << ","  // omega_correction_rps (限幅后)
      << omega_i                            << ","  // omega_i_rps
      << omega_before_limits                << ","  // omega_before_limits_rps
      << feedback_limit                     << ","  // feedback_limit_rps
      << k1                                 << ","  // k1
      << k2                                 << ","  // k2
      << nearest_idx                        << ","  // nearest_idx
      << (is_printing ? 1 : 0)             << ","  // is_printing (1=喷墨中)
      << integral_state;                            // integral_state (积分累积量原始值)

  tracking_sample_rows_.push_back(row.str());
  next_tracking_sample_time_s_ += kTrackingSampleInterval;
}

void LQRCircleController::exportTrackingMetrics()
{
  if (tracking_metrics_exported_)
  {
    return;
  }
  tracking_metrics_exported_ = true;

  if (tracking_error_abs_m_.empty())
  {
    RCLCPP_WARN(get_logger(), "本次圆弧未记录到有效样本，跳过指标导出");
    return;
  }

  // ── 计算统计量 ──
  const double p50_mm  = computePercentile(tracking_error_abs_m_, 50.0)  * 1000.0;
  const double p90_mm  = computePercentile(tracking_error_abs_m_, 90.0)  * 1000.0;
  const double p95_mm  = computePercentile(tracking_error_abs_m_, 95.0)  * 1000.0;
  const double max_mm  = computePercentile(tracking_error_abs_m_, 100.0) * 1000.0;
  const double mean_mm =
      (std::accumulate(tracking_error_abs_m_.begin(), tracking_error_abs_m_.end(), 0.0) /
       static_cast<double>(tracking_error_abs_m_.size())) * 1000.0;
  const double ratio_lt_3mm = computeShareBelowThreshold(tracking_error_abs_m_, 0.003);
  const double ratio_lt_5mm = computeShareBelowThreshold(tracking_error_abs_m_, 0.005);

  // 径向误差统计（带符号均值，正=偏外，负=偏内）
  const double mean_radial_mm =
      (std::accumulate(tracking_radial_err_m_.begin(), tracking_radial_err_m_.end(), 0.0) /
       static_cast<double>(tracking_radial_err_m_.size())) * 1000.0;
  const double sq_sum = std::accumulate(
      tracking_radial_err_m_.begin(), tracking_radial_err_m_.end(), 0.0,
      [](double acc, double v) { return acc + v * v; });
  const double std_radial_mm =
      std::sqrt(sq_sum / static_cast<double>(tracking_radial_err_m_.size())) * 1000.0;

  // ── 基于真实几何角度（atan2(y-cy, x-cx)）的象限分析 ──
  // 真实角度范围[-π, π]，但样本可能跨越2π（完整圆），需要归一化到[0, 2π)
  // 先用径向误差计算真实p90（比基于lookahead的cross_track更准确）
  const double p90_radial_mm  = computePercentile(
      [&]() {
        std::vector<double> abs_rad;
        abs_rad.reserve(tracking_radial_err_m_.size());
        for (double v : tracking_radial_err_m_) abs_rad.push_back(std::abs(v));
        return abs_rad;
      }(), 90.0) * 1000.0;
  const double p50_radial_mm  = computePercentile(
      [&]() {
        std::vector<double> abs_rad;
        abs_rad.reserve(tracking_radial_err_m_.size());
        for (double v : tracking_radial_err_m_) abs_rad.push_back(std::abs(v));
        return abs_rad;
      }(), 50.0) * 1000.0;
  const double ratio_lt_3mm_radial = computeShareBelowThreshold(
      [&]() {
        std::vector<double> abs_rad;
        abs_rad.reserve(tracking_radial_err_m_.size());
        for (double v : tracking_radial_err_m_) abs_rad.push_back(std::abs(v));
        return abs_rad;
      }(), 0.003);
  const double ratio_lt_5mm_radial = computeShareBelowThreshold(
      [&]() {
        std::vector<double> abs_rad;
        abs_rad.reserve(tracking_radial_err_m_.size());
        for (double v : tracking_radial_err_m_) abs_rad.push_back(std::abs(v));
        return abs_rad;
      }(), 0.005);

  // 角度分段分析（基于真实几何角度 atan2(y-cy, x-cx)，而非偏航角积分）
  // 真实角度能准确映射到圆弧物理位置
  // 注：tracking_sample_rows_ 存储的是字符串，角度从 tracking_angle_at_sample_ 取
  // tracking_angle_at_sample_ 存的是 accum_angle_rad（偏航积分），
  // 我们在 samples CSV 中单独存了 real_angle_rad（几何角），但内存中还是 accum_angle。
  // 用 x_m/y_m + center 重算真实角度来做象限划分：
  // 由于 samples 已是字符串，需从 tracking_radial_err_m_ 对应索引重建角度分组。
  // 此处用 accum_angle 近似（0~2π 均分），已足够定位大致问题段。
  auto quadrant_mean_radial = [&](double q_start, double q_end) -> double {
    std::vector<double> seg;
    for (size_t i = 0; i < tracking_angle_at_sample_.size(); ++i)
    {
      const double a = tracking_angle_at_sample_[i];
      if (a >= q_start && a < q_end)
      {
        seg.push_back(std::abs(tracking_radial_err_m_[i]));
      }
    }
    if (seg.empty()) return 0.0;
    return (std::accumulate(seg.begin(), seg.end(), 0.0) /
            static_cast<double>(seg.size())) * 1000.0;
  };
  const double q1_mean_mm = quadrant_mean_radial(0.0,         M_PI * 0.5);
  const double q2_mean_mm = quadrant_mean_radial(M_PI * 0.5,  M_PI * 1.0);
  const double q3_mean_mm = quadrant_mean_radial(M_PI * 1.0,  M_PI * 1.5);
  const double q4_mean_mm = quadrant_mean_radial(M_PI * 1.5,  M_PI * 2.0);

  // 控制饱和率（omega_correction 触碰 feedback_limit 的比例）— 从样本解析
  // 此处仅用样本数量做摘要，详细分析由 analyze.py 完成

  RCLCPP_INFO(get_logger(),
              "Circle误差统计: N=%zu, P50=%.3fmm, P90=%.3fmm, P95=%.3fmm, MAX=%.3fmm, MEAN=%.3fmm",
              tracking_error_abs_m_.size(), p50_mm, p90_mm, p95_mm, max_mm, mean_mm);
  RCLCPP_INFO(get_logger(),
              "Circle覆盖统计: <3mm=%.1f%%, <5mm=%.1f%%",
              ratio_lt_3mm * 100.0, ratio_lt_5mm * 100.0);
  RCLCPP_INFO(get_logger(),
              "Circle径向偏差: mean=%.3fmm (正=偏外), std=%.3fmm",
              mean_radial_mm, std_radial_mm);
  RCLCPP_INFO(get_logger(),
              "Circle象限分布: Q1=%.3fmm, Q2=%.3fmm, Q3=%.3fmm, Q4=%.3fmm",
              q1_mean_mm, q2_mean_mm, q3_mean_mm, q4_mean_mm);

  // ── 写入文件 ──
  const std::string base_dir   = getTrackingRecordDir();
  const std::string latest_dir = base_dir + "/circle_tracking_latest";

  std::lock_guard<std::mutex> lock(file_mutex_);

  try
  {
    std::filesystem::create_directories(latest_dir);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "无法创建指标目录 %s: %s", latest_dir.c_str(), e.what());
    return;
  }

  // 时间戳
  const auto now_sys = std::chrono::system_clock::now();
  const std::time_t now_t = std::chrono::system_clock::to_time_t(now_sys);
  std::ostringstream ts_ss;
  ts_ss << std::put_time(std::localtime(&now_t), "%Y%m%d_%H%M%S");
  const std::string timestamp = ts_ss.str();

  // 槽位标签
  std::ostringstream slot_ss;
  slot_ss << std::setw(2) << std::setfill('0') << circle_slot_;
  const std::string circle_tag  = "circle_" + slot_ss.str();
  const std::string metrics_file = latest_dir + "/" + circle_tag + "_metrics.csv";
  const std::string samples_file = latest_dir + "/" + circle_tag + "_samples.csv";
  const std::string batch_file   = latest_dir + "/batch_metrics.csv";

  // CSV 文本消毒（逗号/换行替换为分号）
  auto sanitize = [](std::string s) {
    for (char& c : s)
    {
      if (c == ',' || c == '\n' || c == '\r') c = ';';
    }
    return s;
  };
  const std::string opt_id     = sanitize(params_.optimization.id);
  const std::string opt_parent = sanitize(params_.optimization.parent_batch_id);
  const std::string opt_note   = sanitize(params_.optimization.change_note);
  const std::string path_type_str =
      (path_type_ == PathType::CIRCLE) ? "CIRCLE" : "ARC";

  // 删除旧的同槽位文件（避免数据混淆）
  for (const auto& f : {metrics_file, samples_file})
  {
    if (std::filesystem::exists(f))
    {
      std::error_code ec;
      std::filesystem::remove(f, ec);
    }
  }

  // ── metrics CSV ──
  std::ofstream metrics(metrics_file, std::ios::out);
  if (!metrics.is_open())
  {
    RCLCPP_ERROR(get_logger(), "无法写入指标文件: %s", metrics_file.c_str());
    return;
  }
  metrics << "batch_id,circle_slot,timestamp,n_samples,circle_radius_m,path_type,"
             "circle_center_x_m,circle_center_y_m,"
             "total_angle_rad,p50_mm,p90_mm,p95_mm,max_mm,mean_mm,"
             "ratio_lt_3mm,ratio_lt_5mm,"
             "p90_radial_mm,p50_radial_mm,ratio_lt_3mm_radial,ratio_lt_5mm_radial,"
             "mean_radial_mm,std_radial_mm,"
             "q1_mean_mm,q2_mean_mm,q3_mean_mm,q4_mean_mm,"
             "target_p90_mm,pass_p90_lt_5mm,"
             "v_max,omega_max,lookahead_dist,lookahead_time,sg_window,"
             "feedback_ratio_before,feedback_ratio_after,feedback_min_limit,"
             "enable_integral,ki,integral_max,"
             "q1_lqr,q2_lqr,r_lqr,"
             "optimization_id,parent_batch_id,change_note\n";
  metrics << std::fixed << std::setprecision(6)
          << circle_batch_id_           << ","
          << circle_slot_               << ","
          << timestamp                  << ","
          << tracking_error_abs_m_.size() << ","
          << circle_radius_             << ","
          << path_type_str              << ","
          << circle_center_x_           << ","   // circle_center_x_m
          << circle_center_y_           << ","   // circle_center_y_m
          << target_total_angle_        << ","
          << p50_mm                     << ","
          << p90_mm                     << ","
          << p95_mm                     << ","
          << max_mm                     << ","
          << mean_mm                    << ","
          << ratio_lt_3mm               << ","
          << ratio_lt_5mm               << ","
          << p90_radial_mm              << ","   // 真实几何精度（主指标）
          << p50_radial_mm              << ","
          << ratio_lt_3mm_radial        << ","
          << ratio_lt_5mm_radial        << ","
          << mean_radial_mm             << ","
          << std_radial_mm              << ","
          << q1_mean_mm                 << ","
          << q2_mean_mm                 << ","
          << q3_mean_mm                 << ","
          << q4_mean_mm                 << ","
          << 5.0                        << ","   // target_p90_mm
          << (p90_radial_mm < 5.0 ? 1 : 0) << ","  // pass_p90_lt_5mm（用径向误差判断）
          << computeScheduledVelocity(circle_radius_) << ","  // v_max (实际调度速度)
          << params_.omega_max          << ","
          << params_.lookahead_distance << ","
          << params_.lookahead_time     << ","
          << params_.savgol_window      << ","
          << params_.feedback_limit_ratio_before_print << ","
          << params_.feedback_limit_ratio_after_print  << ","
          << params_.feedback_min_limit << ","
          << (params_.enable_integral ? 1 : 0) << ","
          << params_.Ki                 << ","
          << params_.integral_max       << ","
          << params_.q1                 << ","
          << params_.q2                 << ","
          << params_.r                  << ","
          << "\"" << opt_id     << "\","
          << "\"" << opt_parent << "\","
          << "\"" << opt_note   << "\""
          << "\n";
  metrics.close();

  // ── samples CSV ──
  std::ofstream samples(samples_file, std::ios::out);
  if (!samples.is_open())
  {
    RCLCPP_ERROR(get_logger(), "无法写入采样文件: %s", samples_file.c_str());
    return;
  }
  samples << "t_s,x_m,y_m,circle_center_x_m,circle_center_y_m,"
             "cross_track_mm,radial_err_mm,accum_angle_rad,real_angle_rad,circle_radius_m,"
             "linear_speed_mps,angular_cmd_rps,ref_theta_rad,e_theta_rad,"
             "omega_ff_rps,omega_fb_raw_rps,omega_correction_rps,omega_i_rps,"
             "omega_before_limits_rps,feedback_limit_rps,k1,k2,nearest_idx,"
             "is_printing,integral_state\n";
  for (const auto& row : tracking_sample_rows_)
  {
    samples << row << "\n";
  }
  samples.close();

  // ── batch_metrics CSV（追加模式）──
  const bool need_header = !std::filesystem::exists(batch_file) ||
                           std::filesystem::file_size(batch_file) == 0;
  std::ofstream batch(batch_file, std::ios::app);
  if (batch.is_open())
  {
    if (need_header)
    {
      batch << "batch_id,circle_slot,timestamp,n_samples,circle_radius_m,path_type,"
               "circle_center_x_m,circle_center_y_m,"
               "total_angle_rad,p50_mm,p90_mm,p95_mm,max_mm,mean_mm,"
               "ratio_lt_3mm,ratio_lt_5mm,"
               "p90_radial_mm,p50_radial_mm,ratio_lt_3mm_radial,ratio_lt_5mm_radial,"
               "mean_radial_mm,std_radial_mm,"
               "q1_mean_mm,q2_mean_mm,q3_mean_mm,q4_mean_mm,"
               "pass_p90_lt_5mm,optimization_id,parent_batch_id,change_note\n";
    }
    batch << std::fixed << std::setprecision(6)
          << circle_batch_id_           << ","
          << circle_slot_               << ","
          << timestamp                  << ","
          << tracking_error_abs_m_.size() << ","
          << circle_radius_             << ","
          << path_type_str              << ","
          << circle_center_x_           << ","
          << circle_center_y_           << ","
          << target_total_angle_        << ","
          << p50_mm                     << ","
          << p90_mm                     << ","
          << p95_mm                     << ","
          << max_mm                     << ","
          << mean_mm                    << ","
          << ratio_lt_3mm               << ","
          << ratio_lt_5mm               << ","
          << p90_radial_mm              << ","
          << p50_radial_mm              << ","
          << ratio_lt_3mm_radial        << ","
          << ratio_lt_5mm_radial        << ","
          << mean_radial_mm             << ","
          << std_radial_mm              << ","
          << q1_mean_mm                 << ","
          << q2_mean_mm                 << ","
          << q3_mean_mm                 << ","
          << q4_mean_mm                 << ","
          << (p90_radial_mm < 5.0 ? 1 : 0) << ","
          << "\"" << opt_id     << "\","
          << "\"" << opt_parent << "\","
          << "\"" << opt_note   << "\""
          << "\n";
    batch.close();
  }

  RCLCPP_INFO(get_logger(), "圆弧跟踪数据已导出: %s", latest_dir.c_str());
}

double LQRCircleController::computePercentile(
    const std::vector<double>& values, double percentile) const
{
  if (values.empty()) return 0.0;

  const double p = std::clamp(percentile, 0.0, 100.0);
  std::vector<double> sorted(values.begin(), values.end());
  std::sort(sorted.begin(), sorted.end());

  if (sorted.size() == 1) return sorted.front();

  const double rank = (p / 100.0) * static_cast<double>(sorted.size() - 1);
  const size_t lo   = static_cast<size_t>(std::floor(rank));
  const size_t hi   = static_cast<size_t>(std::ceil(rank));
  const double t    = rank - static_cast<double>(lo);
  return sorted[lo] + (sorted[hi] - sorted[lo]) * t;
}

double LQRCircleController::computeShareBelowThreshold(
    const std::vector<double>& values, double threshold_m) const
{
  if (values.empty()) return 0.0;
  size_t hit = 0;
  for (const double v : values)
  {
    if (std::isfinite(v) && std::abs(v) < threshold_m) ++hit;
  }
  return static_cast<double>(hit) / static_cast<double>(values.size());
}

double LQRCircleController::computeScheduledVelocity(double radius) const
{
  // 未启用调度或半径无效，直接使用 v_max
  if (!params_.enable_radius_schedule || radius <= 0.0)
  {
    return params_.v_max;
  }

  const double v_min = std::max(0.01, params_.v_min);
  const double v_max = std::max(v_min, params_.v_max);

  // R < radius_threshold：使用最小速度
  if (radius < params_.radius_threshold)
  {
    return v_min;
  }

  // R >= radius_threshold：按步长递增
  // steps = floor((R - threshold) / radius_step)
  // v     = v_min + steps * velocity_step，上限 v_max
  const double excess = radius - params_.radius_threshold;
  const double step_size = std::max(1e-3, params_.radius_step);    // 防止除零
  const int    steps     = static_cast<int>(std::floor(excess / step_size));
  const double v         = v_min + static_cast<double>(steps) * params_.velocity_step;

  return std::clamp(v, v_min, v_max);
}

std::string LQRCircleController::getTrackingRecordDir() const
{
  const char* ws_root = std::getenv("XLINE_WS_ROOT");
  if (ws_root && *ws_root) return std::string(ws_root);

  // 回退：使用当前进程工作目录
  try
  {
    return std::filesystem::current_path().string();
  }
  catch (const std::exception&) {}

  return ".";
}

}  // namespace follow_controller
}  // namespace xline
