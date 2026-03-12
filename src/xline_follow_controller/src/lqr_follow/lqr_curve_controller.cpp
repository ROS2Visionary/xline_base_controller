#include "xline_follow_controller/lqr_follow/lqr_curve_controller.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sstream>
#include <iomanip>
#include <ctime>

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
  , prev_rotation_omega_(0.0)
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

    // 加载原地旋转控制参数（梯形速度规划 + slew rate 阻尼）
    params_.rotation_max_w = parser.getParameter<double>("rotation.max_w");
    params_.rotation_min_w = parser.getParameter<double>("rotation.min_w");
    params_.rotation_decel  = parser.getParameter<double>("rotation.decel");
    params_.rotation_accel  = parser.getParameter<double>("rotation.accel");
    params_.rotation_pre_stop_angle = parser.hasParameter("rotation.pre_stop_angle") ?
        parser.getParameter<double>("rotation.pre_stop_angle") : 0.05;

    // 加载反馈限制参数
    params_.feedback_limit_ratio = parser.getParameter<double>("feedback.limit_ratio");
    params_.feedback_min_limit = parser.getParameter<double>("feedback.min_limit");

    // 加载精细反馈参数（圆弧控制器移植）
    if (parser.hasParameter("feedback.e_theta_lowpass_alpha"))
      params_.e_theta_lowpass_alpha = parser.getParameter<double>("feedback.e_theta_lowpass_alpha");
    if (parser.hasParameter("feedback.max_ey_jump_m"))
      params_.max_ey_jump_m = parser.getParameter<double>("feedback.max_ey_jump_m");
    if (parser.hasParameter("feedback.int_limit_ratio"))
      params_.int_limit_ratio = parser.getParameter<double>("feedback.int_limit_ratio");
    if (parser.hasParameter("feedback.limit_ratio_before_print"))
      params_.feedback_limit_ratio_before_print =
          parser.getParameter<double>("feedback.limit_ratio_before_print");
    if (parser.hasParameter("feedback.limit_ratio_after_print"))
      params_.feedback_limit_ratio_after_print =
          parser.getParameter<double>("feedback.limit_ratio_after_print");

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

    // 加载优化元信息
    if (parser.hasParameter("optimization.id"))
      params_.optimization.id = parser.getParameter<std::string>("optimization.id");
    if (parser.hasParameter("optimization.parent_batch_id"))
      params_.optimization.parent_batch_id = parser.getParameter<std::string>("optimization.parent_batch_id");
    if (parser.hasParameter("optimization.change_note"))
      params_.optimization.change_note = parser.getParameter<std::string>("optimization.change_note");

    // 加载曲线追踪编号
    if (parser.hasParameter("tracking.curve_slot"))
      curve_slot_ = parser.getParameter<int>("tracking.curve_slot");

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
  last_nearest_idx_ = 0;  // 新路径点数可能不同，必须重置，否则越界访问
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
    prev_rotation_omega_ = 0.0;
  }

  RCLCPP_INFO(get_logger(), "LQR路径设置完成 - 点数: %zu, 总长度: %.3fm, 起始朝向: %.2f°",
              path_.size(), accumulated_arc_length, target_yaw_ * 180.0 / M_PI);

  // 初始化数据追踪
  {
    // 生成批次 ID（毫秒时间戳）
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    curve_batch_id_ = std::to_string(now_ms);

    // 记录路径总长度（供段分析使用）
    curve_total_length_ = accumulated_arc_length;

    // 清空缓冲
    tracking_error_abs_m_.clear();
    tracking_error_signed_m_.clear();
    tracking_arc_length_at_sample_.clear();
    tracking_sample_rows_.clear();
    tracking_start_time_ = -1.0;
    is_tracking_ = false;

    // 若 slot==0，清除旧的 curve_tracking_latest 目录（新一轮批次）
    if (curve_slot_ == 0)
    {
      const std::string latest_dir = getTrackingRecordDir() + "/curve_tracking_latest";
      std::lock_guard<std::mutex> lock(file_mutex_);
      std::error_code ec;
      if (std::filesystem::exists(latest_dir))
      {
        std::filesystem::remove_all(latest_dir, ec);
      }
    }
    RCLCPP_INFO(get_logger(), "曲线追踪批次初始化: batch_id=%s, slot=%d, 总长度=%.3fm",
                curve_batch_id_.c_str(), curve_slot_, curve_total_length_);
  }

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

  // 预对准完成后启动数据追踪
  if (!is_tracking_)
  {
    is_tracking_ = true;
    tracking_start_time_ = this->now().seconds();
    tracking_elapsed_time_s_ = 0.0;
    next_tracking_sample_time_s_ = kTrackingSampleInterval;
  }

  // 实际控制周期测量（用于时变积分衰减和平滑切换）
  double ctrl_dt = params_.control_period;  // 默认值：防止首次调用异常
  if (ctrl_time_initialized_)
  {
    double dt = (this->now() - last_ctrl_time_).seconds();
    ctrl_dt = std::clamp(dt, 0.005, 0.2);  // 5ms ~ 200ms 合理范围
  }
  last_ctrl_time_ = this->now();
  ctrl_time_initialized_ = true;

  // 1. 找到最近的路径点
  size_t nearest_idx = findNearestPoint(current_x, current_y);

  // 2. 计算前瞻距离
  double lookahead_dist = getLookaheadDistance(current_v);

  // 3. 获取前瞻参考点（带插值）
  PathPointWithCurvature ref = findLookaheadPoint(nearest_idx, lookahead_dist);

  // 4. 计算误差（使用最近点，omega_ff 仍用前瞻点曲率）
  //
  // 【设计决策】LQR 状态反馈必须基于当前实际偏差（最近点），而非前瞻点：
  //   - 前瞻点切线 theta_la = theta_nn + κ·L，当机器人完美在路径上时仍有
  //     e_theta = -κ·L ≠ 0 的系统性偏置（κ=5/m,L=20mm → 0.10rad=5.7°）
  //   - 该偏置经 K2=1.5 放大后进入 tanh，导致持续向曲线内侧推力
  //   - 稳态内侧偏差：κ·L·K2/K1 ≈ 0.01~0.09m（κ=2~7.5/m），超出精度目标
  // 前馈 omega_ff = v·κ_la 保留前瞻曲率（曲率预见性），不受影响。
  double e_y, e_theta;
  computeErrors(current_x, current_y, current_theta, path_[nearest_idx], e_y, e_theta);

  // 4a. e_y 跳变检测（防止积分 windup，如传感器突变或滤波器暂态）
  bool freeze_integral = false;
  if (params_.max_ey_jump_m > 0.0 && last_e_y_initialized_)
  {
    if (std::abs(e_y - last_e_y_) > params_.max_ey_jump_m)
      freeze_integral = true;
  }
  last_e_y_ = e_y;
  last_e_y_initialized_ = true;

  // 4b. e_theta IIR 低通滤波（α=0.82 时 τ ≈ 280ms，抑制高频抖动）
  e_theta_filtered_ = params_.e_theta_lowpass_alpha * e_theta_filtered_
                    + (1.0 - params_.e_theta_lowpass_alpha) * e_theta;

  // 5. 计算LQR增益（基于当前速度）
  computeLQRGains(current_v);

  // 6. 前馈控制 ω_ff = v × κ
  double omega_ff = current_v * ref.curvature;

  // 7. 平滑切换反馈限制比例（打印前/打印中，τ=0.3s）
  {
    double target_ratio = start_print
        ? params_.feedback_limit_ratio_after_print
        : params_.feedback_limit_ratio_before_print;
    const double alpha_r = std::exp(-ctrl_dt / 0.3);
    limit_ratio_smoothed_ = alpha_r * limit_ratio_smoothed_ + (1.0 - alpha_r) * target_ratio;
  }

  // 8. 比例反馈限制（基于平滑后的比例）
  double prop_limit = std::abs(omega_ff) * limit_ratio_smoothed_;
  if (params_.feedback_min_limit > 0.0 && prop_limit < params_.feedback_min_limit)
    prop_limit = params_.feedback_min_limit;

  // 9. LQR 比例反馈（tanh 软饱和，替代硬 clamp）
  double omega_fb_raw = -K1_ * e_y - K2_ * e_theta_filtered_;
  double omega_prop = (prop_limit > 1e-9)
      ? std::tanh(omega_fb_raw / prop_limit) * prop_limit
      : 0.0;

  // 10. 积分项（可选，双路径独立限幅）
  double omega_i = 0.0;
  if (params_.enable_integral)
  {
    if (!freeze_integral)
    {
      // 时变衰减：decay^(ctrl_dt/T)，对控制频率抖动不敏感
      double decay_dt = std::pow(params_.integral_decay,
                                 ctrl_dt / params_.control_period);
      integral_e_y_ = decay_dt * integral_e_y_ + e_y * ctrl_dt;
      // 防止累积量无界增长
      double max_accum = params_.integral_max / std::max(params_.Ki, 1e-9);
      integral_e_y_ = std::clamp(integral_e_y_, -max_accum, max_accum);
    }
    // 积分输出独立限幅（与比例路径解耦）
    double int_limit = std::abs(omega_ff) * params_.int_limit_ratio;
    if (params_.feedback_min_limit > 0.0 && int_limit < params_.feedback_min_limit)
      int_limit = params_.feedback_min_limit;
    omega_i = -params_.Ki * integral_e_y_;
    omega_i = std::clamp(omega_i, -int_limit, int_limit);
  }

  // 11. 总反馈量（比例+积分，用于调试和饱和率分析）
  double omega_correction = omega_prop + omega_i;
  // feedback_limit 在新算法中为比例路径的 prop_limit（兼容 analyze.py）
  double feedback_limit = prop_limit;

  // 12. 总控制量 = 前馈 + 比例反馈 + 积分
  double omega_before_limits = omega_ff + omega_prop + omega_i;

  // 13. 应用角速度和角加速度限幅（传入实测 ctrl_dt）
  double omega = applyLimits(omega_before_limits, ctrl_dt);

  // 14. 根据路径类型检查是否完成
  bool completed = false;
  if (is_spline_path_)
  {
    completed = updateAccumulatedDistanceForSpline(current_x, current_y);
    if (completed)
    {
      goal_reached_ = true;
      cmd_vel.header.stamp = this->now();
      cmd_vel.header.frame_id = pose.header.frame_id;
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;

      // 导出追踪指标
      if (is_tracking_) exportTrackingMetrics();

      RCLCPP_INFO(get_logger(), "Spline路径完成 - 累计距离: %.4f m",
                  accumulated_distance_);
      return true;
    }
  }
  else
  {
    completed = updateAccumulatedDistance(current_x, current_y);
    if (completed)
    {
      goal_reached_ = true;
      cmd_vel.header.stamp = this->now();
      cmd_vel.header.frame_id = pose.header.frame_id;
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;

      // 导出追踪指标
      if (is_tracking_) exportTrackingMetrics();

      RCLCPP_INFO(get_logger(), "椭圆路径完成 - 累计距离: %.4f m",
                  accumulated_distance_);
      return true;
    }
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

  // 14. 记录追踪采样（50ms 间隔，减少 CSV 体积并对齐圆弧控制器数据格式）
  if (is_tracking_)
  {
    tracking_elapsed_time_s_ += ctrl_dt;
    if (tracking_elapsed_time_s_ >= next_tracking_sample_time_s_)
    {
      next_tracking_sample_time_s_ += kTrackingSampleInterval;

      // 误差统计（供 exportTrackingMetrics 计算 p90/mean 等）
      tracking_error_abs_m_.push_back(std::abs(e_y));
      tracking_error_signed_m_.push_back(e_y);
      tracking_arc_length_at_sample_.push_back(accumulated_distance_);

      double t_s = this->now().seconds() - tracking_start_time_;
      double path_progress = (curve_total_length_ > 1e-6)
          ? std::clamp(accumulated_distance_ / curve_total_length_, 0.0, 1.0)
          : 0.0;

      std::ostringstream row;
      row << std::fixed << std::setprecision(6)
          << t_s                      << ","
          << current_x                << ","
          << current_y                << ","
          << accumulated_distance_    << ","
          << path_progress            << ","
          << e_y * 1000.0             << ","   // cross_track_mm
          << current_v                << ","
          << omega                    << ","
          << path_[nearest_idx].theta << ","   // nearest point theta（误差计算参考系）
          << e_theta                  << ","
          << omega_ff                 << ","
          << omega_correction         << ","   // omega_prop + omega_i
          << feedback_limit           << ","   // prop_limit
          << K1_                      << ","
          << K2_                      << ","
          << nearest_idx              << ","
          << integral_e_y_            << ","
          << omega_fb_raw             << ","   // 原始 LQR 反馈（tanh 前）
          << omega_i                  << ","   // 积分输出
          << omega_before_limits      << ","   // applyLimits 前的总量
          << (start_print ? 1 : 0);            // 打印状态标志
      tracking_sample_rows_.push_back(row.str());
    }
  }

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

double LQRCurveController::applyLimits(double omega, double ctrl_dt)
{
  // 角速度限幅
  omega = std::clamp(omega, -params_.omega_max, params_.omega_max);

  // 角加速度限幅（使用实测 ctrl_dt，而非固定 control_period）
  // 实测周期在 18Hz 时约 ±10ms 抖动，固定值会导致 ±18% 限幅误差
  double omega_change = omega - last_omega_;
  double max_change = params_.omega_dot_max * ctrl_dt;

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
  const double abs_e = std::abs(angle_diff);
  const double dt = params_.control_period;

  // 1. 速度上限随角度差线性缩放：接近 π 才用最大速度，小角度自动压低上限
  double omega_ceiling = params_.rotation_min_w +
      (params_.rotation_max_w - params_.rotation_min_w) * (abs_e / M_PI);
  omega_ceiling = std::clamp(omega_ceiling, params_.rotation_min_w, params_.rotation_max_w);

  // 2. 梯形制动：提前 pre_stop_angle 降至 min_w，在裕量角度内以 min_w 匀速滑行
  double effective_e = std::max(0.0, abs_e - params_.rotation_pre_stop_angle);
  double omega_brake = std::sqrt(2.0 * params_.rotation_decel * effective_e);

  // 3. 目标速度取两者最小值
  double omega_target = std::min(omega_ceiling, omega_brake);
  omega_target = std::clamp(omega_target, params_.rotation_min_w, params_.rotation_max_w);

  // 4. Slew rate 阻尼：限制每周期速度变化量，平滑加减速过程
  double omega = std::clamp(omega_target,
                            prev_rotation_omega_ - params_.rotation_accel * dt,
                            prev_rotation_omega_ + params_.rotation_accel * dt);
  prev_rotation_omega_ = omega;

  return (angle_diff > 0.0) ? omega : -omega;
}

void LQRCurveController::reset()
{
  goal_reached_ = false;
  last_nearest_idx_ = 0;
  prev_rotation_omega_ = 0.0;
  integral_e_y_ = 0.0;
  last_omega_ = 0.0;

  // 重置圆弧控制器移植算法状态
  e_theta_filtered_ = 0.0;
  last_e_y_ = 0.0;
  last_e_y_initialized_ = false;
  limit_ratio_smoothed_ = params_.feedback_limit_ratio_before_print;
  ctrl_time_initialized_ = false;
  tracking_elapsed_time_s_ = 0.0;
  next_tracking_sample_time_s_ = 0.0;

  debug_e_y_ = 0.0;
  debug_e_theta_ = 0.0;
  debug_omega_ff_ = 0.0;
  debug_omega_fb_ = 0.0;
  debug_ref_curvature_ = 0.0;
  debug_ref_index_ = 0;

  // 重置数据追踪
  tracking_error_abs_m_.clear();
  tracking_error_signed_m_.clear();
  tracking_arc_length_at_sample_.clear();
  tracking_sample_rows_.clear();
  tracking_start_time_ = -1.0;
  is_tracking_ = false;
  curve_total_length_ = 0.0;

  // 重置椭圆路径参数
  ellipse_a_ = 0.0;
  ellipse_b_ = 0.0;
  ellipse_center_x_ = 0.0;
  ellipse_center_y_ = 0.0;
  ellipse_rotation_ = 0.0;
  target_arc_length_ = 0.0;
  ellipse_perimeter_ = 0.0;

  // 重置 Spline 路径参数
  is_spline_path_ = false;
  spline_total_length_ = 0.0;

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

  // 仅保留 Hampel 异常值检测（约1帧延迟 ≈ 55ms），移除 SG 平滑滤波。
  // 原因（对齐圆弧控制器修复7）：SG 窗口=7 @ 18Hz 引入 ~194ms 延迟，
  // 相当于 v=0.05m/s 下 ~9.7mm 位置滞后，与 20mm 前瞻量级相当，劣化控制精度。
  // e_y 跳变检测（5mm/周期）已承担异常值保护职能，无需 SG 冗余。
  double filtered_x = h_x_filter.filter(robot_pose.pose.position.x);
  double filtered_y = h_y_filter.filter(robot_pose.pose.position.y);

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

// B-spline 基函数（Cox-de Boor 递归公式）
static double bsplineBasis(int i, int p, double t, const std::vector<double>& knots)
{
  if (p == 0)
  {
    // 0 阶基函数
    if (knots[i] <= t && t < knots[i + 1])
    {
      return 1.0;
    }
    // 特殊处理：当 t 等于最后一个节点时
    if (i == static_cast<int>(knots.size()) - p - 2 && t == knots[i + 1])
    {
      return 1.0;
    }
    return 0.0;
  }

  double left = 0.0;
  double right = 0.0;

  double denom_left = knots[i + p] - knots[i];
  if (denom_left > 1e-10)
  {
    left = (t - knots[i]) / denom_left * bsplineBasis(i, p - 1, t, knots);
  }

  double denom_right = knots[i + p + 1] - knots[i + 1];
  if (denom_right > 1e-10)
  {
    right = (knots[i + p + 1] - t) / denom_right * bsplineBasis(i + 1, p - 1, t, knots);
  }

  return left + right;
}

// 计算 B-spline 曲线上的点
static std::pair<double, double> evaluateBspline(
    double t,
    const std::vector<std::pair<double, double>>& control_points,
    int degree,
    const std::vector<double>& knots)
{
  double x = 0.0;
  double y = 0.0;
  int n = static_cast<int>(control_points.size());

  for (int i = 0; i < n; ++i)
  {
    double basis = bsplineBasis(i, degree, t, knots);
    x += basis * control_points[i].first;
    y += basis * control_points[i].second;
  }

  return {x, y};
}

// 生成均匀 B-spline 节点向量（clamped）
static std::vector<double> generateClampedKnots(int num_control_points, int degree)
{
  int n = num_control_points;
  int num_knots = n + degree + 1;
  std::vector<double> knots(num_knots);

  // Clamped B-spline: 前 (degree+1) 个节点为 0，后 (degree+1) 个节点为 1
  for (int i = 0; i <= degree; ++i)
  {
    knots[i] = 0.0;
  }
  for (int i = degree + 1; i < n; ++i)
  {
    knots[i] = static_cast<double>(i - degree) / static_cast<double>(n - degree);
  }
  for (int i = n; i < num_knots; ++i)
  {
    knots[i] = 1.0;
  }

  return knots;
}

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

  // 确保阶数不超过控制点数-1
  int actual_degree = std::min(degree, static_cast<int>(vertices.size()) - 1);
  if (actual_degree < 1)
  {
    actual_degree = 1;
  }

  // 重置状态
  reset();
  updateParameters("/config/lqr_curve.yaml");

  // 生成 clamped B-spline 节点向量
  std::vector<double> knots = generateClampedKnots(static_cast<int>(vertices.size()), actual_degree);

  // 第一步：粗采样计算曲线总长度
  constexpr int kCoarseSamples = 1000;
  std::vector<std::pair<double, double>> coarse_points;
  coarse_points.reserve(kCoarseSamples + 1);

  for (int i = 0; i <= kCoarseSamples; ++i)
  {
    double t = static_cast<double>(i) / static_cast<double>(kCoarseSamples);
    coarse_points.push_back(evaluateBspline(t, vertices, actual_degree, knots));
  }

  // 计算粗采样曲线总长度
  double total_length = 0.0;
  for (size_t i = 1; i < coarse_points.size(); ++i)
  {
    double dx = coarse_points[i].first - coarse_points[i - 1].first;
    double dy = coarse_points[i].second - coarse_points[i - 1].second;
    total_length += std::hypot(dx, dy);
  }

  // 第二步：按固定弧长间距重新采样
  constexpr double kPointSpacingMeters = 0.003;  // 3mm 间距
  size_t num_points = static_cast<size_t>(std::ceil(total_length / kPointSpacingMeters)) + 1;
  num_points = std::max<size_t>(num_points, 2);

  constexpr size_t kMaxPoints = 20000;
  if (num_points > kMaxPoints)
  {
    RCLCPP_WARN(get_logger(), "Spline采样点过多(%zu)，已限制到 %zu 点", num_points, kMaxPoints);
    num_points = kMaxPoints;
  }

  // 生成 Spline 路径
  nav_msgs::msg::Path spline_path;
  spline_path.header.frame_id = "map";
  spline_path.header.stamp = this->now();
  spline_path.poses.reserve(num_points);

  // 使用弧长参数化重新采样
  double target_spacing = total_length / static_cast<double>(num_points - 1);
  double accumulated_length = 0.0;
  size_t coarse_idx = 0;
  double segment_progress = 0.0;  // 当前段内的进度

  for (size_t i = 0; i < num_points; ++i)
  {
    double target_length = static_cast<double>(i) * target_spacing;

    // 找到目标长度对应的位置
    while (coarse_idx < coarse_points.size() - 1)
    {
      double dx = coarse_points[coarse_idx + 1].first - coarse_points[coarse_idx].first;
      double dy = coarse_points[coarse_idx + 1].second - coarse_points[coarse_idx].second;
      double segment_length = std::hypot(dx, dy);

      if (accumulated_length + segment_length >= target_length || coarse_idx == coarse_points.size() - 2)
      {
        // 在当前段内插值
        double remaining = target_length - accumulated_length;
        segment_progress = (segment_length > 1e-10) ? (remaining / segment_length) : 0.0;
        segment_progress = std::clamp(segment_progress, 0.0, 1.0);
        break;
      }

      accumulated_length += segment_length;
      ++coarse_idx;
    }

    // 线性插值得到精确位置
    double px = coarse_points[coarse_idx].first +
                segment_progress * (coarse_points[coarse_idx + 1].first - coarse_points[coarse_idx].first);
    double py = coarse_points[coarse_idx].second +
                segment_progress * (coarse_points[coarse_idx + 1].second - coarse_points[coarse_idx].second);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = spline_path.header;
    pose.pose.position.x = px;
    pose.pose.position.y = py;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

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
    spline_path.poses[i].pose.orientation = tf2::toMsg(q);
  }

  // 最后一个点的朝向与前一个点相同
  if (spline_path.poses.size() > 1)
  {
    spline_path.poses.back().pose.orientation =
        spline_path.poses[spline_path.poses.size() - 2].pose.orientation;
  }

  // 标记为 Spline 路径并保存路径长度
  is_spline_path_ = true;
  spline_total_length_ = total_length;

  RCLCPP_INFO(get_logger(),
              "B-Spline路径已生成 - 控制点数: %zu, 阶数: %d, 路径点数: %zu, 总长度: %.3fm",
              vertices.size(), actual_degree, spline_path.poses.size(), spline_total_length_);

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

// ============================================================================
// Spline 路径终点判定（基于距离）
// ============================================================================

bool LQRCurveController::updateAccumulatedDistanceForSpline(double current_x, double current_y)
{
  // 如果不是 Spline 路径，跳过判定
  if (!is_spline_path_ || spline_total_length_ <= 1e-6)
  {
    return false;
  }

  if (!last_position_initialized_)
  {
    last_x_ = current_x;
    last_y_ = current_y;
    last_position_initialized_ = true;
    accumulated_distance_ = 0.0;
    RCLCPP_INFO(get_logger(), "Spline路径跟踪开始，初始位置: (%.3f, %.3f)", current_x, current_y);
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

  // Spline 路径的打印参数（固定开始距离 0.4m）
  constexpr double print_start_delay = 1.0;       // 喷码机出墨延时 (秒)
  constexpr double arc_length_for_start = 0.4;    // 开始打印的弧长 0.4m（固定值）

  // 计算延时补偿距离：延时期间机器人行驶的距离
  const double start_lead_distance = params_.v_max * print_start_delay;

  // 开始打印的触发距离（提前触发以补偿延时）
  const double start_trigger_distance = std::max(0.0, arc_length_for_start - start_lead_distance);

  if (accumulated_distance_ > start_trigger_distance)
  {
    if (!print_window_initialized_)
    {
      // 记录触发"开始打印"信号时的累计距离
      start_print_distance_ = accumulated_distance_;

      // 计算有效开始喷印距离（考虑延时后的实际喷印位置）
      const double effective_start_print_distance = start_print_distance_ + start_lead_distance;

      // 定义停止打印窗口参数
      constexpr double kStopArcLengthMeters = 0.03;  // 停止窗口弧长：3cm

      // 计算目标弧长（从开始打印位置到路径终点的距离）
      const double target_print_length = spline_total_length_ - arc_length_for_start;

      // 计算停止打印结束距离
      stop_print_end_distance_ = effective_start_print_distance + target_print_length;

      // 停止打印开始距离 = 结束距离 - 停止窗口长度
      stop_print_start_distance_ = stop_print_end_distance_ - kStopArcLengthMeters;

      print_window_initialized_ = true;  // 标记窗口已初始化

      RCLCPP_INFO(get_logger(),
                  "Spline打印窗口初始化 - 触发距离: %.3fm, 有效开始: %.3fm, 停止开始: %.3fm, 停止结束: %.3fm",
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

    // 判断 Spline 路径是否完成
    if (accumulated_distance_ >= stop_print_end_distance_)
    {
      // 累计距离达到或超过停止打印结束距离，Spline 路径完成
      return true;
    }
  }
  else if (accumulated_distance_ >= spline_total_length_ * 1.3)
  {
    // 若打印窗口未初始化（异常情况），但累计距离已超过路径长度的1.3倍
    // 则强制认为路径完成，避免无限循环
    start_print = false;
    stop_print = true;
    RCLCPP_WARN(get_logger(), "Spline打印窗口未初始化但距离已超限，强制完成");
    return true;
  }

  // 路径未完成，继续跟踪
  return false;
}

// ============================================================================
// 数据采集与导出
// ============================================================================

std::string LQRCurveController::getTrackingRecordDir() const
{
  const char* ws_root = std::getenv("XLINE_WS_ROOT");
  if (ws_root && *ws_root) return std::string(ws_root);
  return ".";
}

double LQRCurveController::computePercentile(
    const std::vector<double>& values, double percentile) const
{
  if (values.empty()) return 0.0;
  const double p = std::clamp(percentile, 0.0, 100.0);
  std::vector<double> sorted(values.begin(), values.end());
  std::sort(sorted.begin(), sorted.end());
  if (sorted.size() == 1) return sorted.front();
  const double rank = (p / 100.0) * static_cast<double>(sorted.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(rank));
  const size_t hi = static_cast<size_t>(std::ceil(rank));
  const double t  = rank - static_cast<double>(lo);
  return sorted[lo] + (sorted[hi] - sorted[lo]) * t;
}

double LQRCurveController::computeShareBelow(
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

void LQRCurveController::exportTrackingMetrics()
{
  if (tracking_error_abs_m_.empty())
  {
    RCLCPP_WARN(get_logger(), "曲线追踪数据为空，跳过导出");
    return;
  }

  // ── 计算统计指标（单位：m） ──
  const size_t n = tracking_error_abs_m_.size();
  double p50_mm = computePercentile(tracking_error_abs_m_, 50) * 1000.0;
  double p90_mm = computePercentile(tracking_error_abs_m_, 90) * 1000.0;
  double p95_mm = computePercentile(tracking_error_abs_m_, 95) * 1000.0;
  double max_mm = *std::max_element(tracking_error_abs_m_.begin(),
                                     tracking_error_abs_m_.end()) * 1000.0;
  double mean_mm = std::accumulate(tracking_error_abs_m_.begin(),
                                    tracking_error_abs_m_.end(), 0.0) / n * 1000.0;
  double ratio_lt_3mm = computeShareBelow(tracking_error_abs_m_, 0.003);
  double ratio_lt_5mm = computeShareBelow(tracking_error_abs_m_, 0.005);

  // 带符号均值（偏向诊断）
  double mean_signed_mm = std::accumulate(tracking_error_signed_m_.begin(),
                                           tracking_error_signed_m_.end(), 0.0) / n * 1000.0;

  // 标准差
  double variance = 0.0;
  for (const double v : tracking_error_signed_m_)
    variance += (v - mean_signed_mm / 1000.0) * (v - mean_signed_mm / 1000.0);
  double std_mm = std::sqrt(variance / n) * 1000.0;

  // 路径段分析（S1-S4，按弧长进度均分）
  auto section_mean_mm = [&](double lo, double hi) -> double {
    std::vector<double> sec_errs;
    for (size_t i = 0; i < tracking_arc_length_at_sample_.size() && i < n; ++i)
    {
      double progress = (curve_total_length_ > 1e-6)
          ? (tracking_arc_length_at_sample_[i] / curve_total_length_) : 0.0;
      if (progress >= lo && progress < hi)
        sec_errs.push_back(tracking_error_abs_m_[i]);
    }
    if (sec_errs.empty()) return 0.0;
    return std::accumulate(sec_errs.begin(), sec_errs.end(), 0.0) / sec_errs.size() * 1000.0;
  };
  double s1_mean_mm = section_mean_mm(0.0,  0.25);
  double s2_mean_mm = section_mean_mm(0.25, 0.50);
  double s3_mean_mm = section_mean_mm(0.50, 0.75);
  double s4_mean_mm = section_mean_mm(0.75, 1.01);

  RCLCPP_INFO(get_logger(),
              "曲线追踪统计 n=%zu: P50=%.3fmm P90=%.3fmm P95=%.3fmm MAX=%.3fmm",
              n, p50_mm, p90_mm, p95_mm, max_mm);
  RCLCPP_INFO(get_logger(),
              "曲线覆盖统计: <3mm=%.1f%% <5mm=%.1f%%",
              ratio_lt_3mm * 100.0, ratio_lt_5mm * 100.0);
  RCLCPP_INFO(get_logger(),
              "曲线段分布: S1=%.3fmm S2=%.3fmm S3=%.3fmm S4=%.3fmm",
              s1_mean_mm, s2_mean_mm, s3_mean_mm, s4_mean_mm);

  // ── 确定数据目录 ──
  const std::string base_dir   = getTrackingRecordDir();
  const std::string latest_dir = base_dir + "/curve_tracking_latest";

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

  // 时间戳字符串
  const auto now_sys = std::chrono::system_clock::now();
  const std::time_t now_t = std::chrono::system_clock::to_time_t(now_sys);
  std::ostringstream ts_ss;
  ts_ss << std::put_time(std::localtime(&now_t), "%Y%m%d_%H%M%S");
  const std::string timestamp = ts_ss.str();

  // 槽位标签
  std::ostringstream slot_ss;
  slot_ss << std::setw(2) << std::setfill('0') << curve_slot_;
  const std::string curve_tag    = "curve_" + slot_ss.str();
  const std::string metrics_file = latest_dir + "/" + curve_tag + "_metrics.csv";
  const std::string samples_file = latest_dir + "/" + curve_tag + "_samples.csv";
  const std::string batch_file   = latest_dir + "/batch_metrics.csv";

  // CSV 文本消毒
  auto sanitize = [](std::string s) {
    for (char& c : s)
      if (c == ',' || c == '\n' || c == '\r') c = ';';
    return s;
  };
  const std::string opt_id     = sanitize(params_.optimization.id);
  const std::string opt_parent = sanitize(params_.optimization.parent_batch_id);
  const std::string opt_note   = sanitize(params_.optimization.change_note);
  const std::string path_type_str = is_spline_path_ ? "SPLINE" : "ELLIPSE";

  // 删除旧同槽位文件
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
  metrics << "batch_id,curve_slot,timestamp,n_samples,path_type,path_total_length_m,"
             "p50_mm,p90_mm,p95_mm,max_mm,mean_mm,"
             "ratio_lt_3mm,ratio_lt_5mm,"
             "mean_signed_mm,std_mm,"
             "s1_mean_mm,s2_mean_mm,s3_mean_mm,s4_mean_mm,"
             "target_p90_mm,pass_p90_lt_5mm,"
             "v_max,omega_max,lookahead_dist,lookahead_time,sg_window,"
             "feedback_limit_ratio,feedback_min_limit,"
             "enable_integral,ki,integral_max,"
             "q1_lqr,q2_lqr,r_lqr,"
             "optimization_id,parent_batch_id,change_note\n";
  metrics << std::fixed << std::setprecision(6)
          << curve_batch_id_          << ","
          << curve_slot_              << ","
          << timestamp                << ","
          << n                        << ","
          << path_type_str            << ","
          << curve_total_length_      << ","
          << p50_mm                   << ","
          << p90_mm                   << ","
          << p95_mm                   << ","
          << max_mm                   << ","
          << mean_mm                  << ","
          << ratio_lt_3mm             << ","
          << ratio_lt_5mm             << ","
          << mean_signed_mm           << ","
          << std_mm                   << ","
          << s1_mean_mm               << ","
          << s2_mean_mm               << ","
          << s3_mean_mm               << ","
          << s4_mean_mm               << ","
          << 5.0                      << ","
          << (p90_mm < 5.0 ? 1 : 0)  << ","
          << params_.v_max            << ","
          << params_.omega_max        << ","
          << params_.lookahead_distance << ","
          << params_.lookahead_time   << ","
          << params_.savgol_window    << ","
          << params_.feedback_limit_ratio << ","
          << params_.feedback_min_limit   << ","
          << (params_.enable_integral ? 1 : 0) << ","
          << params_.Ki               << ","
          << params_.integral_max     << ","
          << params_.q1               << ","
          << params_.q2               << ","
          << params_.r                << ","
          << "\"" << opt_id     << "\","
          << "\"" << opt_parent << "\","
          << "\"" << opt_note   << "\""
          << "\n";
  metrics.close();

  // ── samples CSV ──
  std::ofstream samples(samples_file, std::ios::out);
  if (!samples.is_open())
  {
    RCLCPP_ERROR(get_logger(), "无法写入样本文件: %s", samples_file.c_str());
    return;
  }
  samples << "t_s,x_m,y_m,arc_length_m,path_progress,"
             "cross_track_mm,"
             "linear_speed_mps,angular_cmd_rps,ref_theta_rad,e_theta_rad,"
             "omega_ff_rps,omega_correction_rps,"
             "feedback_limit_rps,k1,k2,nearest_idx,"
             "integral_state,"
             "omega_fb_raw_rps,omega_i_rps,omega_before_limits_rps,is_printing\n";
  for (const auto& row : tracking_sample_rows_)
    samples << row << "\n";
  samples.close();

  // ── batch_metrics CSV（追加模式）──
  const bool need_header = !std::filesystem::exists(batch_file) ||
                           std::filesystem::file_size(batch_file) == 0;
  std::ofstream batch(batch_file, std::ios::app);
  if (batch.is_open())
  {
    if (need_header)
    {
      batch << "batch_id,curve_slot,timestamp,n_samples,path_type,path_total_length_m,"
               "p50_mm,p90_mm,p95_mm,max_mm,mean_mm,"
               "ratio_lt_3mm,ratio_lt_5mm,"
               "mean_signed_mm,std_mm,"
               "s1_mean_mm,s2_mean_mm,s3_mean_mm,s4_mean_mm,"
               "pass_p90_lt_5mm,optimization_id,parent_batch_id,change_note\n";
    }
    batch << std::fixed << std::setprecision(6)
          << curve_batch_id_          << ","
          << curve_slot_              << ","
          << timestamp                << ","
          << n                        << ","
          << path_type_str            << ","
          << curve_total_length_      << ","
          << p50_mm                   << ","
          << p90_mm                   << ","
          << p95_mm                   << ","
          << max_mm                   << ","
          << mean_mm                  << ","
          << ratio_lt_3mm             << ","
          << ratio_lt_5mm             << ","
          << mean_signed_mm           << ","
          << std_mm                   << ","
          << s1_mean_mm               << ","
          << s2_mean_mm               << ","
          << s3_mean_mm               << ","
          << s4_mean_mm               << ","
          << (p90_mm < 5.0 ? 1 : 0)  << ","
          << "\"" << opt_id     << "\","
          << "\"" << opt_parent << "\","
          << "\"" << opt_note   << "\""
          << "\n";
    batch.close();
  }

  RCLCPP_INFO(get_logger(), "曲线追踪数据已导出: %s", latest_dir.c_str());
}

}  // namespace follow_controller
}  // namespace xline
