/**
 * @file transition_controller.cpp
 * @brief 动态伺服转场控制器实现
 *
 * 核心算法：
 * 1. 计算位置误差和航向误差
 * 2. 比例控制生成速度指令
 * 3. 大转角时适度减速
 * 4. 接近目标时渐进减速
 * 5. 多次确认防止抖动
 */

#include "xline_follow_controller/transition/transition_controller.hpp"
#include "xline_follow_controller/common/path_utils.hpp"

namespace xline
{
namespace follow_controller
{

// ============================================================================
// 构造与析构
// ============================================================================

TransitionController::TransitionController()
  : BaseFollowController("transition_controller")
  , goal_x_(0.0)
  , goal_y_(0.0)
  , goal_theta_(0.0)
  , goal_set_(false)
  , goal_theta_set_(false)
  , goal_reached_(false)
  , arrival_count_(0)
  , use_backward_(false)
  , backward_decided_(false)
  , backward_switched_to_forward_(false)
  , prev_distance_(0.0)
  , distance_increasing_count_(0)
  , min_distance_reached_(std::numeric_limits<double>::max())
  , prev_linear_vel_(0.0)
  , prev_angular_vel_(0.0)
  , first_compute_after_goal_(false)
  , stall_detection_active_(false)
  , stall_check_start_distance_(0.0)
  , last_stage2_yaw_error_(0.0)
  , stage2_entry_distance_(0.0)
  , stage2_position_relaxed_(false)
  , stage1_reentry_mode_(false)
{
  updateParameters();
  last_time_ = std::chrono::steady_clock::now();
  last_log_output_time_ = this->now();

  LOG_INFO("TransitionController 初始化完成");
}

TransitionController::~TransitionController()
{
  LOG_INFO("TransitionController 析构");
}

// ============================================================================
// 参数加载
// ============================================================================

void TransitionController::updateParameters()
{
  // 从ament_index获取包路径
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("xline_follow_controller");
  std::string config_file_path = package_share_directory + "/config/transition.yaml";

  xline::YamlParser::YamlParser parser(config_file_path);

  try {
    // 速度限制
    params_.max_velocity = parser.getParameter<double>("transition.max_velocity");
    params_.max_angular_vel = parser.getParameter<double>("transition.max_angular_vel");
    params_.min_velocity = parser.getParameter<double>("transition.min_velocity");

    // 线速度加速限制
    params_.linear_accel_limit = parser.hasParameter("transition.linear_accel_limit") ?
        parser.getParameter<double>("transition.linear_accel_limit") : 0.25;

    // 减速安全系数（减速限制由 max_velocity 和 slow_down_distance 自动计算，在 slow_down_distance 加载后计算）
    params_.decel_safety_factor = parser.hasParameter("transition.decel_safety_factor") ?
        parser.getParameter<double>("transition.decel_safety_factor") : 2.0;
    params_.decel_safety_factor = std::max(1.0, params_.decel_safety_factor);

    // 控制增益
    params_.k_linear = parser.getParameter<double>("transition.k_linear");
    params_.k_angular = parser.getParameter<double>("transition.k_angular");

    // 到达判定
    params_.arrival_tolerance = parser.getParameter<double>("transition.arrival_tolerance");
    params_.arrival_angle_tolerance = parser.getParameter<double>("transition.arrival_angle_tolerance");
    params_.arrival_confirm_count = parser.getParameter<int>("transition.arrival_confirm_count");

    // 减速控制
    params_.slow_down_distance = parser.getParameter<double>("transition.slow_down_distance");
    params_.creep_distance = parser.hasParameter("transition.creep_distance") ?
        parser.getParameter<double>("transition.creep_distance") : 0.1;
    params_.creep_velocity = parser.getParameter<double>("transition.creep_velocity");
    params_.hard_creep_min_enabled = parser.hasParameter("transition.hard_creep_min.enabled") ?
        parser.getParameter<bool>("transition.hard_creep_min.enabled") : false;

    // 自动计算减速限制：decel_limit = max_vel² / slow_down_distance × safety_factor
    // 物理意义：保证速度曲线在减速段能被 slew-rate 限制器完整跟踪，safety_factor 提供裕量
    params_.linear_decel_limit = (params_.slow_down_distance > 1e-6) ?
        (params_.max_velocity * params_.max_velocity / params_.slow_down_distance * params_.decel_safety_factor)
        : (params_.max_velocity * params_.decel_safety_factor);
    params_.linear_decel_limit = std::max(0.0, params_.linear_decel_limit);

    // 大转角处理
    params_.large_angle_threshold = parser.getParameter<double>("transition.large_angle_threshold");
    params_.large_angle_vel_ratio = parser.getParameter<double>("transition.large_angle_vel_ratio");

    // 姿态伺服
    params_.pose_servo_distance = parser.getParameter<double>("transition.pose_servo_distance");

    // 阶段1重入控制
    params_.allow_stage2_exit = parser.hasParameter("transition.allow_stage2_exit") ?
        parser.getParameter<bool>("transition.allow_stage2_exit") : true;
    params_.stage1_reentry_max_velocity = parser.hasParameter("transition.stage1_reentry_max_velocity") ?
        parser.getParameter<double>("transition.stage1_reentry_max_velocity") : params_.max_velocity * 0.5;
    params_.stage1_reentry_max_angular_vel = parser.hasParameter("transition.stage1_reentry_max_angular_vel") ?
        parser.getParameter<double>("transition.stage1_reentry_max_angular_vel") : 0.08;

    // 阶段2航向对准（原地旋转，rotation 方式）
    params_.rotation_enabled = parser.hasParameter("transition.rotation.enabled") ?
        parser.getParameter<bool>("transition.rotation.enabled") : true;
    params_.rotation_max_w = parser.hasParameter("transition.rotation.max_w") ?
        parser.getParameter<double>("transition.rotation.max_w") : params_.max_angular_vel;
    params_.rotation_min_w = parser.hasParameter("transition.rotation.min_w") ?
        parser.getParameter<double>("transition.rotation.min_w") : std::min(0.1, params_.rotation_max_w);
    params_.rotation_absolute_min_w = parser.hasParameter("transition.rotation.absolute_min_w") ?
        parser.getParameter<double>("transition.rotation.absolute_min_w") : 0.03;
    params_.rotation_factor = parser.hasParameter("transition.rotation.factor") ?
        parser.getParameter<double>("transition.rotation.factor") : 1.2;
    params_.rotation_angle_threshold = parser.hasParameter("transition.rotation.angle_threshold") ?
        parser.getParameter<double>("transition.rotation.angle_threshold") : 0.6;
    params_.rotation_smooth_factor = parser.hasParameter("transition.rotation.smooth_factor") ?
        parser.getParameter<double>("transition.rotation.smooth_factor") : 0.6;
    params_.rotation_stop_tolerance = parser.hasParameter("transition.rotation.stop_tolerance") ?
        parser.getParameter<double>("transition.rotation.stop_tolerance") : params_.arrival_angle_tolerance;
    params_.rotation_use_crossing_stop = parser.hasParameter("transition.rotation.use_crossing_stop") ?
        parser.getParameter<bool>("transition.rotation.use_crossing_stop") : true;
    params_.rotation_crossing_window = parser.hasParameter("transition.rotation.crossing_window") ?
        parser.getParameter<double>("transition.rotation.crossing_window") : std::max(params_.rotation_stop_tolerance, 0.2);

    // 保护性限幅：避免配置把控制器锁死或输出超限
    params_.max_velocity = std::max(0.0, params_.max_velocity);
    params_.min_velocity = std::clamp(params_.min_velocity, 0.0, params_.max_velocity);
    params_.max_angular_vel = std::max(0.0, params_.max_angular_vel);
    params_.linear_accel_limit = std::max(0.0, params_.linear_accel_limit);
    params_.linear_decel_limit = std::max(0.0, params_.linear_decel_limit);
    params_.slow_down_distance = std::max(0.0, params_.slow_down_distance);
    params_.creep_distance = std::clamp(params_.creep_distance, 0.0, params_.slow_down_distance);

    params_.rotation_max_w = std::clamp(params_.rotation_max_w, 0.0, params_.max_angular_vel);
    params_.rotation_min_w = std::clamp(params_.rotation_min_w, 0.0, params_.rotation_max_w);
    params_.rotation_absolute_min_w = std::clamp(params_.rotation_absolute_min_w, 0.0, params_.rotation_min_w);
    params_.rotation_factor = std::max(0.0, params_.rotation_factor);
    params_.rotation_angle_threshold = std::max(0.0, params_.rotation_angle_threshold);
    params_.rotation_smooth_factor = std::max(0.0, params_.rotation_smooth_factor);
    params_.rotation_stop_tolerance = std::clamp(
        params_.rotation_stop_tolerance, 0.0, params_.arrival_angle_tolerance);
    params_.rotation_crossing_window = std::max(params_.rotation_crossing_window, params_.rotation_stop_tolerance);

    // 阶段1重入速度限制
    params_.stage1_reentry_max_velocity = std::clamp(params_.stage1_reentry_max_velocity, 0.0, params_.max_velocity);
    params_.stage1_reentry_max_angular_vel = std::clamp(params_.stage1_reentry_max_angular_vel, 0.0, params_.max_angular_vel);

    // 后退模式
    params_.enable_backward = parser.getParameter<bool>("transition.enable_backward");
    params_.backward_angle_threshold = parser.getParameter<double>("transition.backward_angle_threshold");

    // 平滑控制
    params_.velocity_smooth_alpha = parser.getParameter<double>("transition.velocity_smooth_alpha");

    // 动态速度重置
    params_.dynamic_vel_reset_max_vel = parser.hasParameter("transition.dynamic_velocity_reset.max_vel") ?
        parser.getParameter<double>("transition.dynamic_velocity_reset.max_vel") : params_.max_velocity;
    params_.dynamic_vel_reset_min_vel = parser.hasParameter("transition.dynamic_velocity_reset.min_vel") ?
        parser.getParameter<double>("transition.dynamic_velocity_reset.min_vel") : params_.min_velocity;
    params_.dynamic_vel_threshold_distance = parser.hasParameter("transition.dynamic_velocity_reset.threshold_distance") ?
        parser.getParameter<double>("transition.dynamic_velocity_reset.threshold_distance") : 2.0;
    params_.dynamic_vel_dist_step = parser.hasParameter("transition.dynamic_velocity_reset.dist_step") ?
        parser.getParameter<double>("transition.dynamic_velocity_reset.dist_step") : 0.2;
    params_.dynamic_vel_vel_step = parser.hasParameter("transition.dynamic_velocity_reset.vel_step") ?
        parser.getParameter<double>("transition.dynamic_velocity_reset.vel_step") : 0.1;

    // 保护性限幅：dynamic_velocity_reset 直接替代静态 max_velocity，不受其约束
    // （若需全局硬上限，由调用方 setSpeedLimit 控制）
    params_.dynamic_vel_reset_max_vel = std::max(0.0, params_.dynamic_vel_reset_max_vel);
    params_.dynamic_vel_reset_min_vel = std::clamp(params_.dynamic_vel_reset_min_vel,
                                                    0.0, params_.dynamic_vel_reset_max_vel);
    params_.dynamic_vel_threshold_distance = std::max(0.0, params_.dynamic_vel_threshold_distance);
    params_.dynamic_vel_dist_step = std::max(1e-3, params_.dynamic_vel_dist_step);
    params_.dynamic_vel_vel_step  = std::max(0.0, params_.dynamic_vel_vel_step);

    // 角速度大时线速度衰减
    params_.angular_vel_decel_scale = parser.hasParameter("transition.angular_vel_decel.scale") ?
        parser.getParameter<double>("transition.angular_vel_decel.scale") : 0.0;
    params_.angular_vel_decel_min_factor = parser.hasParameter("transition.angular_vel_decel.min_factor") ?
        parser.getParameter<double>("transition.angular_vel_decel.min_factor") : 0.2;
    params_.angular_vel_decel_scale = std::clamp(params_.angular_vel_decel_scale, 0.0, 1.0);
    params_.angular_vel_decel_min_factor = std::clamp(params_.angular_vel_decel_min_factor, 0.0, 1.0);

    // 蠕动段停滞检测
    params_.stall_detection_enabled = parser.hasParameter("transition.stall_detection.enabled") ?
        parser.getParameter<bool>("transition.stall_detection.enabled") : true;
    params_.stall_distance_threshold = parser.hasParameter("transition.stall_detection.distance_threshold") ?
        parser.getParameter<double>("transition.stall_detection.distance_threshold") : params_.creep_distance;
    params_.stall_timeout = parser.hasParameter("transition.stall_detection.timeout") ?
        parser.getParameter<double>("transition.stall_detection.timeout") : 3.0;
    params_.stall_min_progress = parser.hasParameter("transition.stall_detection.min_progress") ?
        parser.getParameter<double>("transition.stall_detection.min_progress") : 0.003;
    params_.stall_distance_threshold = std::max(0.0, params_.stall_distance_threshold);
    params_.stall_timeout            = std::max(0.1, params_.stall_timeout);
    params_.stall_min_progress       = std::max(0.0, params_.stall_min_progress);

    // 调试
    params_.debug_enabled = parser.getParameter<bool>("transition.debug_enabled");

    LOG_INFO("参数加载成功 | 速度限制: v_max=%.2fm/s, ω_max=%.2frad/s | 控制增益: k_lin=%.1f, k_ang=%.1f",
             params_.max_velocity, params_.max_angular_vel, params_.k_linear, params_.k_angular);
    LOG_INFO("  加减速: accel=%.3fm/s², decel=%.3fm/s²(%.1f²/%.2f×%.1f)",
             params_.linear_accel_limit, params_.linear_decel_limit,
             params_.max_velocity, params_.slow_down_distance, params_.decel_safety_factor);
    LOG_INFO("  精度: 位置±%.1fcm, 角度±%.1f° | 姿态伺服:%.2fm | 后退模式:%s(阈值%.0f°)",
             params_.arrival_tolerance * 100.0,
             params_.arrival_angle_tolerance * 57.3,
             params_.pose_servo_distance,
             params_.enable_backward ? "启用" : "禁用",
             params_.backward_angle_threshold * 57.3);
    LOG_INFO("  阶段2控制: %s | 重入速度限制: v=%.3fm/s, ω=%.2frad/s",
             params_.allow_stage2_exit ? "允许退出(高精度)" : "不退出(快速)",
             params_.stage1_reentry_max_velocity,
             params_.stage1_reentry_max_angular_vel);

  } catch (const std::exception& e) {
    LOG_WARN("参数加载失败，使用默认值: %s", e.what());

    // 默认参数
    params_.max_velocity = 0.5;
    params_.max_angular_vel = 1.0;
    params_.min_velocity = 0.05;
    params_.linear_accel_limit = 0.25;
    params_.decel_safety_factor = 2.0;
    params_.k_linear = 1.0;
    params_.k_angular = 2.5;
    params_.arrival_tolerance = 0.03;
    params_.arrival_angle_tolerance = 0.1;  // 5.7度
    params_.arrival_confirm_count = 3;
    params_.slow_down_distance = 0.5;
    // decel_limit 在 slow_down_distance 赋值后计算
    params_.linear_decel_limit = params_.max_velocity * params_.max_velocity
                                  / params_.slow_down_distance * params_.decel_safety_factor;
    params_.creep_distance = 0.1;
    params_.creep_velocity = 0.05;
    params_.hard_creep_min_enabled = false;
    params_.large_angle_threshold = M_PI / 3.0;  // 60度
    params_.large_angle_vel_ratio = 0.4;
    params_.pose_servo_distance = 0.5;
    params_.rotation_enabled = true;
    params_.rotation_max_w = params_.max_angular_vel;
    params_.rotation_min_w = 0.1;
    params_.rotation_factor = 1.2;
    params_.rotation_angle_threshold = 0.6;
    params_.rotation_smooth_factor = 0.6;
    params_.rotation_stop_tolerance = params_.arrival_angle_tolerance;
    params_.rotation_use_crossing_stop = true;
    params_.rotation_crossing_window = 0.35;
    params_.enable_backward = true;
    params_.backward_angle_threshold = M_PI / 6.0;  // 30度
    params_.velocity_smooth_alpha = 0.7;
    params_.debug_enabled = false;
  }
}

// ============================================================================
// 公共接口实现
// ============================================================================

bool TransitionController::setGoal(double goal_x, double goal_y)
{

  updateParameters();

  goal_x_ = goal_x;
  goal_y_ = goal_y;
  goal_set_ = true;
  goal_theta_set_ = false;  // 不控制朝向
  goal_reached_ = false;
  arrival_count_ = 0;
  use_backward_ = false;  // 只到点模式不使用后退
  backward_decided_ = false;
  backward_switched_to_forward_ = false;
  first_compute_after_goal_ = true;  // 标记：下次 computeVelocityCommands 时按距离重置 max_velocity
  stall_detection_active_ = false;
  last_time_ = std::chrono::steady_clock::now();  // 重置时间基准，避免首帧 dt 异常大导致 slew-rate 跳过

  // 重置后退安全检测
  prev_distance_ = 0.0;
  distance_increasing_count_ = 0;
  min_distance_reached_ = std::numeric_limits<double>::max();
  last_stage2_yaw_error_ = 0.0;
  stage2_entry_distance_ = 0.0;
  stage2_position_relaxed_ = false;

  LOG_INFO("设置目标点: (%.3f, %.3f) [只到点模式]", goal_x_, goal_y_);

  return true;
}

bool TransitionController::setGoal(double goal_x, double goal_y, double goal_theta)
{

  updateParameters();

  goal_x_ = goal_x;
  goal_y_ = goal_y;
  goal_theta_ = goal_theta;
  goal_set_ = true;
  goal_theta_set_ = true;  // 控制朝向
  goal_reached_ = false;
  arrival_count_ = 0;
  use_backward_ = false;  // 将在首次计算时判断
  backward_decided_ = false;
  backward_switched_to_forward_ = false;
  first_compute_after_goal_ = true;  // 标记：下次 computeVelocityCommands 时按距离重置 max_velocity
  stall_detection_active_ = false;

  // 重置后退安全检测
  prev_distance_ = 0.0;
  distance_increasing_count_ = 0;
  min_distance_reached_ = std::numeric_limits<double>::max();
  last_stage2_yaw_error_ = 0.0;
  stage2_entry_distance_ = 0.0;
  stage2_position_relaxed_ = false;

  LOG_INFO("设置目标姿态: (%.3f, %.3f, %.2f°) [姿态伺服模式]",
           goal_x_, goal_y_, goal_theta_ * 180.0 / M_PI);

  return true;
}

bool TransitionController::setPlan(const nav_msgs::msg::Path& path)
{
  if (path.poses.empty()) {
    LOG_ERROR("路径为空");
    return false;
  }

  // 提取终点作为目标
  const auto& goal_pose = path.poses.back();
  return setGoal(goal_pose.pose.position.x, goal_pose.pose.position.y);
}

bool TransitionController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    geometry_msgs::msg::TwistStamped& cmd_vel)
{
  (void)velocity;  // 未使用当前速度

  // dt（用于平滑/加减速限制）
  auto now_tp = std::chrono::steady_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now_tp - last_time_).count();
  last_time_ = now_tp;
  if (dt <= 0.0 || std::isnan(dt) || std::isinf(dt) || dt > 1.0) {
    dt = 1.0 / 18.0;  // 约 55ms，对应 18Hz；原写法 1/18 为整数除法结果为 0，导致 slew-rate 失效
  }

  // 1. 检查是否设置了目标
  if (!goal_set_) {
    LOG_WARN("未设置目标点");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return false;
  }

  // 2. 检查是否已到达
  if (goal_reached_) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return true;
  }

  // 3. 获取当前位姿
  double curr_x = pose.pose.position.x;
  double curr_y = pose.pose.position.y;
  double curr_theta = tf2::getYaw(pose.pose.orientation);

  // 4. 计算位置误差
  double distance = computeDistance(curr_x, curr_y);

  // 4.1 接收新目标后第一次执行：根据初始距离动态重置最大线速度
  // 策略：< threshold_distance 直接用 min_vel；
  //       >= threshold_distance 每超出 dist_step 增加 vel_step，上限 max_vel
  if (first_compute_after_goal_) {
    first_compute_after_goal_ = false;
    double dynamic_max;
    if (distance < params_.dynamic_vel_threshold_distance) {
      dynamic_max = params_.dynamic_vel_reset_min_vel;
    } else {
      double extra = distance - params_.dynamic_vel_threshold_distance;
      double added_vel = (extra / params_.dynamic_vel_dist_step) * params_.dynamic_vel_vel_step;
      dynamic_max = params_.dynamic_vel_reset_min_vel + added_vel;
    }
    params_.max_velocity = std::clamp(dynamic_max,
                                      params_.dynamic_vel_reset_min_vel,
                                      params_.dynamic_vel_reset_max_vel);
    // 同步更新减速限制（decel_limit 与当前 max_velocity 和 slow_down_distance 挂钩）
    if (params_.slow_down_distance > 1e-6) {
      params_.linear_decel_limit = params_.max_velocity * params_.max_velocity
                                    / params_.slow_down_distance * params_.decel_safety_factor;
    }
    LOG_INFO("动态速度重置: 初始距离=%.3fm → max_vel=%.3fm/s "
             "(阈值%.1fm, 步长%.2fm/%.3fm/s, 范围[%.3f, %.3f])",
             distance, params_.max_velocity,
             params_.dynamic_vel_threshold_distance,
             params_.dynamic_vel_dist_step, params_.dynamic_vel_vel_step,
             params_.dynamic_vel_reset_min_vel, params_.dynamic_vel_reset_max_vel);
  }

  // 4.2 停滞检测（仅姿态伺服模式，且尚未进入阶段2）
  // 当距离长时间无法收敛（< stall_distance_threshold 但未到 arrival_tolerance）时，
  // 放弃位置精对准，强制进入阶段2直接对准航向
  if (params_.stall_detection_enabled && goal_theta_set_ && !stage2_position_relaxed_ && !goal_reached_) {
    if (distance < params_.stall_distance_threshold && distance > params_.arrival_tolerance) {
      if (!stall_detection_active_) {
        // 首次进入阈值区域，开始计时
        stall_detection_active_ = true;
        stall_check_start_time_ = this->now();
        stall_check_start_distance_ = distance;
      } else {
        double elapsed = (this->now() - stall_check_start_time_).seconds();
        double progress = stall_check_start_distance_ - distance;  // 正数表示距离在减小（在靠近）
        if (progress >= params_.stall_min_progress) {
          // 有实质进步，更新基准重新计时
          stall_check_start_time_ = this->now();
          stall_check_start_distance_ = distance;
        } else if (elapsed >= params_.stall_timeout) {
          // 超时且无进步 → 强制进入阶段2，放弃位置精对准
          LOG_WARN("⚠ 蠕动停滞超时 %.1fs（距离 %.1fmm → %.1fmm，进步 %.1fmm < %.1fmm阈值），"
                   "放弃位置对准，强制进入阶段2对准航向",
                   elapsed,
                   stall_check_start_distance_ * 1000.0, distance * 1000.0,
                   progress * 1000.0, params_.stall_min_progress * 1000.0);
          stage2_position_relaxed_ = true;
          stage2_entry_distance_ = distance;
          stage1_reentry_mode_ = false;
          arrival_count_ = 0;
          stall_detection_active_ = false;
          prev_linear_vel_ = 0.0;
          prev_angular_vel_ = 0.0;
        }
      }
    } else {
      // 距离 >= 阈值（离得远了）或已到达，重置计时器
      stall_detection_active_ = false;
    }
  }

  // 5. 根据模式选择控制策略
  double v, omega;
  bool reached = false;

  if (!goal_theta_set_) {
    // ============================================================================
    // 模式1：只到点（不控制朝向）
    // ============================================================================
    double target_heading = computeTargetHeading(curr_x, curr_y);
    double heading_error = normalizeAngle(target_heading - curr_theta);

    // 到达判定：只检查位置
    if (distance < params_.arrival_tolerance) {
      arrival_count_++;
      if (arrival_count_ >= params_.arrival_confirm_count) {
        reached = true;
      }
    } else {
      arrival_count_ = 0;
    }

    // 计算速度指令
    v = computeLinearVelocity(distance, heading_error);
    omega = computeAngularVelocity(heading_error);

    // 接近目标时逐渐减小角速度，避免到达后无意义旋转
    // 当距离 < 2倍到达容差时，角速度随距离线性衰减
    double angle_decay_distance = params_.arrival_tolerance * 2.0;
    if (distance < angle_decay_distance && distance > params_.arrival_tolerance * 0.5) {
      double decay_ratio = distance / angle_decay_distance;
      omega *= decay_ratio;
      // 角速度衰减信息已包含在统一状态日志中
    }

  } else {
    // ============================================================================
    // 模式2：姿态伺服 - 分段控制策略
    // 阶段1：位置控制 - 朝向目标位置移动（无需关心最终朝向）
    // 阶段2：朝向控制 - 原地旋转对准目标朝向
    // ============================================================================

    double final_heading_error = normalizeAngle(goal_theta_ - curr_theta);

    // ============================================================================
    // 判断控制阶段（阶段2粘滞：避免原地旋转引起的距离微漂移导致频繁跳回阶段1）
    // ============================================================================
    const double stage2_exit_distance = params_.arrival_tolerance * 5.0;  // 5倍退出阈值（足够大的滞后）
    bool use_stage2 = false;

    if (stage2_position_relaxed_) {
      // 已经进入过阶段2：根据配置决定是否允许退出
      if (distance <= stage2_exit_distance) {
        use_stage2 = true;
      } else if (params_.allow_stage2_exit) {
        // 允许退出阶段2：漂移过大时回到阶段1重新收敛位置
        LOG_INFO("⚠ 退出阶段2，位置漂移过大: %.4fm > %.4fm（阈值）| 重新进入阶段1收敛位置 | 启用速度限制",
                 distance, stage2_exit_distance);
        stage2_position_relaxed_ = false;
        stage2_entry_distance_ = 0.0;
        last_stage2_yaw_error_ = 0.0;
        arrival_count_ = 0;
        stage1_reentry_mode_ = true;  // 启用阶段1重入模式
      } else {
        // 不允许退出阶段2：继续保持阶段2，专注朝向对准（允许位置漂移）
        use_stage2 = true;
        // 每2秒输出一次警告（避免日志刷屏）
        static rclcpp::Time last_drift_warn_time = this->now();
        auto now = this->now();
        if ((now - last_drift_warn_time).seconds() >= 2.0) {
          LOG_WARN("阶段2位置漂移: %.4fm > %.4fm（阈值）| 继续朝向对准（allow_stage2_exit=false）",
                   distance, stage2_exit_distance);
          last_drift_warn_time = now;
        }
      }
    } else if (distance <= params_.arrival_tolerance) {
      // 首次满足到点精度，进入阶段2
      use_stage2 = true;
    }

    if (!use_stage2) {
      // ------------------------------------------------------------
      // 阶段1：位置控制 - 朝向目标位置移动
      // ------------------------------------------------------------
      last_stage2_yaw_error_ = 0.0;

      // 判断是否使用后退模式（使用配置的阈值）
      bool should_use_backward = false;
      if (params_.enable_backward && !backward_decided_) {
        should_use_backward = shouldUseBackward(curr_x, curr_y, curr_theta);

        if (should_use_backward) {
          // 计算目标在机器人坐标系中的位置（用于日志输出）
          double dx_world = goal_x_ - curr_x;
          double dy_world = goal_y_ - curr_y;
          double cos_theta = std::cos(curr_theta);
          double sin_theta = std::sin(curr_theta);
          double dx_local = dx_world * cos_theta + dy_world * sin_theta;
          double dy_local = -dx_world * sin_theta + dy_world * cos_theta;

          // 计算背向目标位置的方向误差（与判断逻辑一致）
          double approach_heading = std::atan2(dy_world, dx_world);
          double backward_desired_heading = normalizeAngle(approach_heading + M_PI);
          double backward_heading_error = std::abs(normalizeAngle(backward_desired_heading - curr_theta));

          LOG_INFO("✓ 使用后退模式 - 目标在后方(dx=%.2fm,dy=%.2fm)，背向方向对准(%.1f°<%.1f°)",
                   dx_local, dy_local,
                   backward_heading_error * 180.0 / M_PI,
                   params_.backward_angle_threshold * 180.0 / M_PI);
        }
        backward_decided_ = true;
      }

      bool use_backward_mode = should_use_backward || (use_backward_ && backward_decided_);
      if (use_backward_mode) {
        const double dx_world = goal_x_ - curr_x;
        const double dy_world = goal_y_ - curr_y;
        const double cos_theta = std::cos(curr_theta);
        const double sin_theta = std::sin(curr_theta);
        const double dx_local = dx_world * cos_theta + dy_world * sin_theta;

        // 近目标时若目标已切换到车体前方，后退会造成来回穿越；仅允许单向切换一次到前进模式
        const double switch_forward_threshold = 0.003;  // 3mm
        if (!backward_switched_to_forward_ && distance <= params_.creep_distance && dx_local > switch_forward_threshold) {
          use_backward_mode = false;
          use_backward_ = false;
          backward_switched_to_forward_ = true;
          LOG_INFO("↻ 后退模式切换为前进模式（近目标穿越保护）: dist=%.1fcm, dx_local=%.1fmm",
                   distance * 100.0, dx_local * 1000.0);
        }
      }

      if (use_backward_mode) {
        // 后退模式（两步法中的“到达位置”阶段）：
        // - 阶段1目标是“到点”，因此航向应基于目标位置计算
        // - 使用“背向目标点的朝向”作为期望朝向，使得负线速度能朝目标收敛
        double approach_heading = computeTargetHeading(curr_x, curr_y);
        double backward_desired_heading = normalizeAngle(approach_heading + M_PI);
        double backward_heading_error = normalizeAngle(backward_desired_heading - curr_theta);

        // 如果偏差过大，先原地旋转对准"后退期望朝向"，避免横向倒车导致绕圈/离目标更远
        // 但在近目标时允许蠕动，避免打转
        if (std::abs(backward_heading_error) > M_PI / 2.0) {
          omega = params_.rotation_enabled ?
              computeRotationVelocity(backward_heading_error) :
              computeAngularVelocity(backward_heading_error);
          // 扩大蠕动区域：当距离 < 10cm 时允许后退蠕动
          // 使用固定阈值 0.1m，确保即使 arrival_tolerance 很小（0.5cm）也能生效
          if (distance <= params_.creep_distance) {
            const double dx_world = goal_x_ - curr_x;
            const double dy_world = goal_y_ - curr_y;
            const double cos_theta = std::cos(curr_theta);
            const double sin_theta = std::sin(curr_theta);
            const double dx_local = dx_world * cos_theta + dy_world * sin_theta;

            if (std::abs(dx_local) < 0.002) {
              v = 0.0;
            } else {
              // 后退模式：蠕动速度只能为负（避免在"后退"模式下前进）
              double creep_speed = params_.creep_velocity;
              // 仅在“目标确实在后方（dx_local<0）”时允许后退蠕动；
              // 避免在大角度旋转时叠加平移导致绕圈（v/ω≈常数，形成极限环）。
              if (dx_local < 0.0) {
                v = std::clamp(dx_local, -creep_speed, 0.0);  // 速度 <= 0
              } else {
                v = 0.0;
              }
            }
          } else {
            v = 0.0;
          }
        } else {
          v = -computeLinearVelocity(distance, backward_heading_error);
          omega = computeAngularVelocity(backward_heading_error);
        }
        use_backward_ = true;
      } else {
        // 前进模式：朝向目标位置
        double approach_heading = computeTargetHeading(curr_x, curr_y);
        double approach_error = normalizeAngle(approach_heading - curr_theta);

        // 如果航向误差过大（>90°），先原地旋转对准"前进期望朝向"
        // 但在近目标时（cm级）纯旋转会导致距离来回漂移，容易卡在严格阈值附近；
        // 因此允许做较大的前后蠕动来继续压缩距离，避免陷入打转。
        if (std::abs(approach_error) > M_PI / 2.0) {
          omega = params_.rotation_enabled ?
              computeRotationVelocity(approach_error) :
              computeAngularVelocity(approach_error);
          // 扩大蠕动区域：当距离 < 10cm 时允许蠕动（避免在目标附近打转）
          // 使用固定阈值 0.1m，确保即使 arrival_tolerance 很小（0.5cm）也能生效
          if (distance <= params_.creep_distance) {
            const double dx_world = goal_x_ - curr_x;
            const double dy_world = goal_y_ - curr_y;
            const double cos_theta = std::cos(curr_theta);
            const double sin_theta = std::sin(curr_theta);
            const double dx_local = dx_world * cos_theta + dy_world * sin_theta;

            if (std::abs(dx_local) < 0.002) {
              v = 0.0;
            } else {
              // 前进模式：蠕动速度只能为正（避免在"前进"模式下后退）
              double creep_speed = params_.creep_velocity;
              // 仅在“目标确实在前方（dx_local>0）”时允许前进蠕动；
              // 否则大角度旋转 + 恒定 v 会在目标附近形成绕圈/打转。
              if (dx_local > 0.0) {
                v = std::clamp(dx_local, 0.0, creep_speed);  // 速度 >= 0
              } else {
                v = 0.0;
              }
            }
          } else {
            v = 0.0;
          }
        } else {
          v = computeLinearVelocity(distance, approach_error);
          omega = computeAngularVelocity(approach_error);
        }
      }

      // ============================================================================
      // 阶段1重入速度限制（从阶段2退回时，使用更小的速度）
      // ============================================================================
      if (stage1_reentry_mode_) {
        v = std::clamp(v, -params_.stage1_reentry_max_velocity, params_.stage1_reentry_max_velocity);
        omega = std::clamp(omega, -params_.stage1_reentry_max_angular_vel, params_.stage1_reentry_max_angular_vel);

        // 当距离再次小于阈值时，准备进入阶段2，清除重入模式
        if (distance <= params_.arrival_tolerance) {
          stage1_reentry_mode_ = false;
        }
      }

      // ============================================================================
      // 阶段1近距离角速度衰减（避免快速旋转导致位置漂移）
      // ============================================================================
      // 当距离 < 5cm 时，角速度随距离线性衰减，避免在目标附近旋转过快导致位置不断漂移
      const double angular_decay_distance = 0.05;  // 5cm
      if (distance < angular_decay_distance) {
        // 衰减比例：距离越小，衰减越多
        // 距离 5cm → 100% 角速度
        // 距离 2cm → 40% 角速度
        // 距离 0.5cm → 10% 角速度
        double decay_ratio = distance / angular_decay_distance;
        decay_ratio = std::max(decay_ratio, 0.1);  // 最小保留10%，避免完全无法转向
        omega *= decay_ratio;
      }

    } else {
      // ------------------------------------------------------------
      // 阶段2：朝向控制 - 原地旋转对准目标朝向
      // ------------------------------------------------------------

      // 记录进入阶段2时的距离（仅记录一次）
      if (!stage2_position_relaxed_) {
        stage2_entry_distance_ = distance;
        stage2_position_relaxed_ = true;
        stage1_reentry_mode_ = false;  // 清除重入模式标志
        LOG_INFO("✓ 进入阶段2（朝向对齐），基准距离: %.4fm | 策略：允许微小蠕动保持位置 | 回退阈值: %.1fcm",
                 stage2_entry_distance_, stage2_exit_distance * 100.0);
      }

      v = 0.0;  // 默认停止移动

      // 若原地旋转导致距离略微漂移出阈值，允许小幅蠕动把位置拉回（避免永远达不到 1cm）
      if (distance > params_.arrival_tolerance) {
        const double dx_world = goal_x_ - curr_x;
        const double dy_world = goal_y_ - curr_y;
        const double cos_theta = std::cos(curr_theta);
        const double sin_theta = std::sin(curr_theta);
        const double dx_local = dx_world * cos_theta + dy_world * sin_theta;

        if (std::abs(dx_local) >= 0.002) {
          v = std::clamp(dx_local, -params_.creep_velocity, params_.creep_velocity);
        }
      }
      // 与 LineFollowController 的"对齐阶段"一致：误差小于阈值时直接置零（避免最小角速度导致抖动计数失败）
      bool has_crossed = false;
      if (params_.rotation_use_crossing_stop) {
        bool crossed_sign = (last_stage2_yaw_error_ > 0.0 && final_heading_error < 0.0) ||
                            (last_stage2_yaw_error_ < 0.0 && final_heading_error > 0.0);
        bool within_window = (std::abs(last_stage2_yaw_error_) < params_.rotation_crossing_window) &&
                             (std::abs(final_heading_error) < params_.rotation_crossing_window);
        has_crossed = crossed_sign && within_window;
      }

      if (std::abs(final_heading_error) < params_.rotation_stop_tolerance || has_crossed) {
        omega = 0.0;
        prev_linear_vel_ = 0.0;
        prev_angular_vel_ = 0.0;
      } else {
        omega = params_.rotation_enabled ?
            computeRotationVelocity(final_heading_error) :
            computeAngularVelocity(final_heading_error);
      }
      last_stage2_yaw_error_ = final_heading_error;
    }

    // 到达判定：阶段1只检查位置，阶段2只检查朝向（分离精度控制）
    // - 阶段1：确保位置精度达标（不关心朝向）
    // - 阶段2：确保朝向精度达标（位置已在阶段1保证）
    bool position_ok = false;
    bool heading_ok = false;

    if (stage2_position_relaxed_) {
      // 阶段2：只检查朝向，位置由阶段1保证
      position_ok = true;  // 不检查位置
      heading_ok = std::abs(final_heading_error) < params_.arrival_angle_tolerance;

      // 安全监控：若漂移过大，给出警告（会在上面的 use_stage2 判定中回退阶段1）
      if (distance > stage2_exit_distance) {
        static auto last_warning_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_warning_time).count();
        if (elapsed >= 5) {
          LOG_WARN("⚠ 阶段2位置漂移过大: 当前=%.1fmm > 阈值=%.1fmm，将回退阶段1",
                   distance * 1000.0, stage2_exit_distance * 1000.0);
          last_warning_time = now;
        }
      }
    } else {
      // 阶段1：只检查位置，不关心朝向（朝向由阶段2保证）
      position_ok = distance < params_.arrival_tolerance;
      heading_ok = true;  // 不检查朝向
    }

    if (position_ok && heading_ok) {
      arrival_count_++;
      if (arrival_count_ >= params_.arrival_confirm_count) {
        reached = true;
      }
    } else {
      arrival_count_ = 0;
    }
  }

  // 6. 检查是否到达
  if (reached) {
    goal_reached_ = true;
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    if (goal_theta_set_) {
      // 显示阶段2入口距离（若有）
      if (stage2_position_relaxed_) {
        double position_drift = distance - stage2_entry_distance_;
        LOG_INFO("✓ 到达目标姿态！最终距离: %.1fmm（入口: %.1fmm，漂移: %+.1fmm）, 朝向误差: %.2f° [确认: %d/%d]",
                 distance * 1000.0, stage2_entry_distance_ * 1000.0, position_drift * 1000.0,
                 normalizeAngle(goal_theta_ - curr_theta) * 180.0 / M_PI,
                 arrival_count_, params_.arrival_confirm_count);
      } else {
        LOG_INFO("✓ 到达目标姿态！最终距离: %.4fm, 朝向误差: %.2f° [确认次数: %d/%d]",
                 distance, normalizeAngle(goal_theta_ - curr_theta) * 180.0 / M_PI,
                 arrival_count_, params_.arrival_confirm_count);
      }
    } else {
      LOG_INFO("✓ 到达目标点！最终距离: %.4fm [确认次数: %d/%d]",
               distance, arrival_count_, params_.arrival_confirm_count);
    }
    return true;
  }

  // 7. 计算速度指令（已在上面计算）

  // 7.5 角速度大时降低线速度（防止高角速度引起漂移）
  // 原理：omega 越大（相对于 max_angular_vel），线速度按比例衰减，衰减量由 scale 控制
  if (params_.angular_vel_decel_scale > 0.0 && params_.max_angular_vel > 1e-6 && std::abs(v) > 1e-12) {
    double abs_omega_ratio = std::abs(omega) / params_.max_angular_vel;
    abs_omega_ratio = std::clamp(abs_omega_ratio, 0.0, 1.0);
    double decel_factor = 1.0 - params_.angular_vel_decel_scale * abs_omega_ratio;
    decel_factor = std::clamp(decel_factor, params_.angular_vel_decel_min_factor, 1.0);
    v *= decel_factor;
  }

  // 8. 速度平滑
  smoothVelocity(v, omega, dt);

  // 8.1 近目标蠕动速度硬下限（优先级高于平滑/角度衰减）
  enforceHardCreepMin(v, distance, curr_x, curr_y, curr_theta);

  // 9. 输出
  cmd_vel.twist.linear.x = v;
  cmd_vel.twist.angular.z = omega;

  // 10. 调试输出（每0.5秒输出一次统一状态日志）
  if (params_.debug_enabled) {
    auto current_time = this->now();
    if ((current_time - last_log_output_time_).seconds() >= 0.5) {
      // 构建统一的状态日志
      std::string mode_str;
      std::string phase_str;
      std::string status_str;
      double heading_error = 0.0;
      double final_heading_error = 0.0;

      if (goal_theta_set_) {
        // 姿态伺服模式 - 分段控制
        final_heading_error = normalizeAngle(goal_theta_ - curr_theta);

        // 判断阶段（与实际控制逻辑一致，考虑阶段2粘滞）
        const double stage2_exit_distance = params_.arrival_tolerance * 5.0;
        bool in_stage2 = stage2_position_relaxed_ && (distance <= stage2_exit_distance);
        bool in_stage1 = !in_stage2;

        if (in_stage1) {
          // 阶段1：位置控制
          phase_str = "位置控制";
          double approach_heading = computeTargetHeading(curr_x, curr_y);
          double approach_error = normalizeAngle(approach_heading - curr_theta);

          // 运动方向
          std::string direction = use_backward_ ? "后退" : "前进";
          mode_str = "姿态·" + direction + "·" + phase_str;

          // 日志中的航向误差，使用“阶段1真实用于控制的误差”
          if (use_backward_) {
            double backward_desired_heading = normalizeAngle(approach_heading + M_PI);
            heading_error = normalizeAngle(backward_desired_heading - curr_theta);
          } else {
            heading_error = approach_error;
          }

          // 状态信息
          if (std::abs(heading_error) > M_PI / 2.0) {
            status_str = "仅旋转";
          } else if (std::abs(heading_error) > params_.large_angle_threshold) {
            status_str = "大角度减速";
          } else {
            status_str = "正常";
          }
        } else {
          // 阶段2：朝向控制
          phase_str = "朝向控制";
          mode_str = "姿态·原地旋转·" + phase_str;
          heading_error = final_heading_error;

          if (std::abs(heading_error) < params_.rotation_stop_tolerance) {
            status_str = "接近对准";
          } else if (std::abs(heading_error) > params_.large_angle_threshold) {
            status_str = "大角度旋转";
          } else {
            status_str = "正常旋转";
          }
        }
      } else {
        // 只到点模式
        mode_str = "到点";
        double approach_heading = computeTargetHeading(curr_x, curr_y);
        heading_error = normalizeAngle(approach_heading - curr_theta);

        if (distance < params_.arrival_tolerance * 2.0) {
          status_str = "接近目标";
        } else if (std::abs(heading_error) > params_.large_angle_threshold) {
          status_str = "大角度减速";
        } else {
          status_str = "正常";
        }
      }

      // 计算进度百分比（基于到达判定阈值）
      double progress_dist = std::max(0.0, 1.0 - distance / (params_.pose_servo_distance * 1.5));
      double progress_angle = 1.0;
      if (goal_theta_set_ && distance <= params_.arrival_tolerance) {
        // 进入阶段2（原地对准）后，进度再叠加终点朝向收敛
        progress_angle = std::max(0.0, 1.0 - std::abs(final_heading_error) / M_PI);
      }
      int progress_pct = static_cast<int>(progress_dist * progress_angle * 100.0);

      // 输出优化的统一状态日志
      std::string stage_indicator;
      std::string extra_info;

      if (goal_theta_set_) {
        // 重新判断阶段（与上面逻辑一致）
        const double stage2_exit_distance_log = params_.arrival_tolerance * 5.0;
        bool in_stage2_log = stage2_position_relaxed_ && (distance <= stage2_exit_distance_log);

        if (!in_stage2_log) {
          stage_indicator = "[S1]";
          // 阶段1：显示距离和航向误差
          extra_info = "";
        } else {
          stage_indicator = "[S2]";
          // 阶段2：显示进入时基准距离和当前漂移量
          double drift = distance - stage2_entry_distance_;
          extra_info = " | 漂移:" + std::to_string(drift * 1000.0).substr(0, 5) + "mm";
        }
      } else {
        stage_indicator = "[到点]";
        extra_info = "";
      }

      LOG_INFO("%s[%s·%s] 位置:(%.3f,%.3f,%.0f°) | 误差: 距%.1fcm/角%.1f° | 速度: v=%.3f/w=%.3f%s | 进度:%d%% | 到达计数:%d/%d",
               stage_indicator.c_str(),
               mode_str.c_str(), status_str.c_str(),
               curr_x, curr_y, curr_theta * 57.3,
               distance * 100.0, heading_error * 57.3,
               v, omega,
               extra_info.c_str(),
               progress_pct,
               arrival_count_, params_.arrival_confirm_count);

      last_log_output_time_ = current_time;
    }
  }

  return true;
}

bool TransitionController::isGoalReached()
{
  return goal_reached_;
}

bool TransitionController::cancel()
{
  reset();
  LOG_INFO("转场任务已取消");
  return true;
}

void TransitionController::setSpeedLimit(const double& speed_limit)
{
  if (speed_limit > 0.0) {
    params_.max_velocity = std::min(params_.max_velocity, speed_limit);
    LOG_INFO("速度限制已设置: %.3f m/s", params_.max_velocity);
  }
}

void TransitionController::reset()
{
  goal_set_ = false;
  goal_theta_set_ = false;
  goal_reached_ = false;
  arrival_count_ = 0;
  prev_linear_vel_ = 0.0;
  prev_angular_vel_ = 0.0;
  first_compute_after_goal_ = false;
  stall_detection_active_ = false;
  use_backward_ = false;
  backward_decided_ = false;
  backward_switched_to_forward_ = false;
  last_stage2_yaw_error_ = 0.0;
  stage2_entry_distance_ = 0.0;
  stage2_position_relaxed_ = false;
  stage1_reentry_mode_ = false;

  // 重置后退安全检测
  prev_distance_ = 0.0;
  distance_increasing_count_ = 0;
  min_distance_reached_ = std::numeric_limits<double>::max();
}

// ============================================================================
// 私有方法实现
// ============================================================================

bool TransitionController::shouldUseBackward(double curr_x, double curr_y, double curr_theta)
{
  // 必须满足的前提条件
  if (!params_.enable_backward || !goal_theta_set_) {
    return false;
  }

  // 将目标点转换到机器人当前坐标系
  double dx_world = goal_x_ - curr_x;
  double dy_world = goal_y_ - curr_y;

  // 旋转到机器人坐标系（前方为+x，左方为+y）
  double cos_theta = std::cos(curr_theta);
  double sin_theta = std::sin(curr_theta);
  double dx_local = dx_world * cos_theta + dy_world * sin_theta;

  // 判断1：目标点在机器人后方（x_local < 0）
  bool target_behind = dx_local < 0;

  // 判断2：背向目标位置的方向与当前朝向接近（这样后退时不需要大幅度旋转）
  // 计算指向目标的方向
  double approach_heading = std::atan2(dy_world, dx_world);
  // 计算背向目标的方向
  double backward_desired_heading = normalizeAngle(approach_heading + M_PI);
  // 计算当前朝向与背向目标方向的误差
  double backward_heading_error = std::abs(normalizeAngle(backward_desired_heading - curr_theta));
  bool backward_direction_aligned = backward_heading_error < params_.backward_angle_threshold;

  // 两个条件都满足才使用后退
  // 后退模式的选择会在第一次判断后在主日志中显示（"姿态伺服-后退xxx"）
  return target_behind && backward_direction_aligned;
}


double TransitionController::computeDistance(double curr_x, double curr_y) const
{
  double dx = goal_x_ - curr_x;
  double dy = goal_y_ - curr_y;
  return std::hypot(dx, dy);
}

double TransitionController::computeTargetHeading(double curr_x, double curr_y) const
{
  double dx = goal_x_ - curr_x;
  double dy = goal_y_ - curr_y;
  return std::atan2(dy, dx);
}

double TransitionController::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double TransitionController::computeLinearVelocity(double distance, double heading_error)
{
  double v;

  // 0. 边界条件：距离极小时直接停止，避免振荡
  if (distance < params_.arrival_tolerance * 0.5) {
    // 距离极小信息已包含在统一状态日志中
    return 0.0;
  }

  // 1. 分段速度规划
  if (distance > params_.slow_down_distance) {
    // 远距离：最大速度
    v = params_.max_velocity;
  } else if (distance > params_.creep_distance) {
    // 中距离：线性减速
    double ratio = distance / params_.slow_down_distance;
    v = params_.max_velocity * ratio;
  } else {
    // 近距离：蠕动速度
    v = params_.creep_velocity;
  }

  // 2. 应用比例增益（整体速度缩放因子）
  v *= params_.k_linear;

  // 3. 大转角减速
  if (std::abs(heading_error) > params_.large_angle_threshold) {
    v *= params_.large_angle_vel_ratio;
    // 大转角状态已包含在统一状态日志的"大角度减速"标记中
  }

  // 3.5 航向误差连续衰减：避免在误差接近 90° 时出现“v 很小但不为 0 + ω 较大”导致的绕圈极限环
  // 0°: 不衰减；90°: 衰减到 0（与上层“>90°仅旋转”的逻辑一致）
  const double abs_heading_error = std::abs(heading_error);
  if (abs_heading_error >= M_PI / 2.0) {
    v = 0.0;
  } else {
    v *= std::max(0.0, std::cos(abs_heading_error));
  }

  // 4. 最终限幅（防止缩放后超出范围）
  // 注意：配置中的 min_velocity 仅用于“正常行驶”避免过慢；
  // 近距离（<10cm）精对位阶段必须允许更小速度，否则会被抬到 min_velocity 导致来回穿越目标点。
  const double min_v = (distance <= params_.creep_distance) ? 0.0 : params_.min_velocity;
  v = std::clamp(v, min_v, params_.max_velocity);

  return v;
}

double TransitionController::computeAngularVelocity(double heading_error)
{
  // 比例控制
  double omega = params_.k_angular * heading_error;

  // 限幅
  omega = std::max(omega, -params_.max_angular_vel);
  omega = std::min(omega, params_.max_angular_vel);

  return omega;
}

void TransitionController::enforceHardCreepMin(
    double& v, double distance, double curr_x, double curr_y, double curr_theta)
{
  if (!params_.hard_creep_min_enabled) {
    return;
  }
  if (!(params_.creep_velocity > 0.0)) {
    return;
  }
  // 仅在蠕动段内生效；且避免在到达容差内仍强推，导致来回穿越阈值
  if (distance > params_.creep_distance || distance <= params_.arrival_tolerance) {
    return;
  }

  // 距离越接近到达阈值，强制下限越小，避免 cm 级目标附近以固定速度来回穿越
  const double denom = std::max(1e-6, params_.creep_distance - params_.arrival_tolerance);
  const double proximity = std::clamp((distance - params_.arrival_tolerance) / denom, 0.0, 1.0);
  const double forced_min_velocity = params_.creep_velocity * std::max(0.2, proximity);

  if (std::abs(v) >= forced_min_velocity) {
    return;
  }

  const double dx_world = goal_x_ - curr_x;
  const double dy_world = goal_y_ - curr_y;
  const double cos_theta = std::cos(curr_theta);
  const double sin_theta = std::sin(curr_theta);
  const double dx_local = dx_world * cos_theta + dy_world * sin_theta;

  const double dx_sign_deadband = 0.003;  // 3mm
  double desired_sign = 0.0;
  if (goal_theta_set_ && !stage2_position_relaxed_) {
    // 阶段1姿态伺服：保持与运动模式一致，避免"后退模式"被硬下限推成正速度
    if (use_backward_) {
      if (dx_local > dx_sign_deadband) {
        v = 0.0;
        prev_linear_vel_ = 0.0;
        return;
      }
      desired_sign = -1.0;
    } else {
      if (dx_local < -dx_sign_deadband) {
        v = 0.0;
        prev_linear_vel_ = 0.0;
        return;
      }
      desired_sign = 1.0;
    }
  } else if (dx_local > dx_sign_deadband) {
    desired_sign = 1.0;
  } else if (dx_local < -dx_sign_deadband) {
    desired_sign = -1.0;
  } else if (std::abs(v) > 1e-12) {
    desired_sign = (v >= 0.0) ? 1.0 : -1.0;
  } else if (std::abs(prev_linear_vel_) > 1e-12) {
    desired_sign = (prev_linear_vel_ >= 0.0) ? 1.0 : -1.0;
  } else {
    desired_sign = 0.0;
  }

  if (std::abs(desired_sign) < 1e-12) {
    return;
  }

  v = desired_sign * forced_min_velocity;
  v = std::clamp(v, -params_.max_velocity, params_.max_velocity);

  // 与 smoothVelocity 的内部状态保持一致（避免下一周期又被认为“冷启动”或被 slew-rate 压低）
  prev_linear_vel_ = v;
}

double TransitionController::computeRotationVelocity(double angle_diff)
{
  const double abs_diff = std::abs(angle_diff);

  // sigmoid：|diff| 越大越接近 1，越小越接近 0.5
  double factor = 1.0 / (1.0 + std::exp(-params_.rotation_factor * abs_diff));

  // 近零区域再做一次平滑衰减，避免接近目标时仍然高速旋转
  if (abs_diff < params_.rotation_angle_threshold && params_.rotation_angle_threshold > 1e-6) {
    double cosine_factor = params_.rotation_smooth_factor *
        (1.0 - std::cos(M_PI * abs_diff / params_.rotation_angle_threshold));
    factor *= cosine_factor;
  }

  double rot_vel = params_.rotation_max_w * factor;
  // 动态最小角速度：当误差很小时，固定 min_w 会导致"抖动/震动"（过冲→反向→过冲…）
  // 这里把 min_w 随误差线性缩小，既保留大误差时的克服静摩擦能力，又降低小误差时的过冲风险。
  double min_w = params_.rotation_min_w;
  if (params_.rotation_angle_threshold > 1e-6) {
    const double scale = std::clamp(abs_diff / params_.rotation_angle_threshold, 0.0, 1.0);
    min_w *= scale;
  }
  // 确保不低于绝对最小角速度（保证机器人始终能动）
  min_w = std::max(min_w, params_.rotation_absolute_min_w);
  rot_vel = std::clamp(rot_vel, min_w, params_.rotation_max_w);

  if (angle_diff > 0.0) {
    return rot_vel;
  }
  if (angle_diff < 0.0) {
    return -rot_vel;
  }
  return 0.0;
}

void TransitionController::smoothVelocity(double& v, double& omega, double dt)
{
  const double alpha = std::clamp(params_.velocity_smooth_alpha, 0.0, 1.0);

  // 记录滤波前的历史输出，用于 slew-rate 限制
  const double prev_v = prev_linear_vel_;
  const double prev_omega = prev_angular_vel_;

  const bool stop_linear = (std::abs(v) < 1e-12);
  const bool stop_angular = (std::abs(omega) < 1e-12);

  // 线速度：若本周期明确要求停止，则必须输出 0（避免低通造成残余速度，破坏 min/max 约束）
  if (stop_linear) {
    v = 0.0;
    prev_linear_vel_ = 0.0;
  } else {
    // 低通滤波（冷启动直通）
    bool is_first_call = (std::abs(prev_v) < 1e-12);
    if (!is_first_call) {
      v = alpha * v + (1.0 - alpha) * prev_v;
    }

    // dt-based 加减速限制（让加速/减速更平滑）
    // 注意：加速/减速应基于速度大小是否增大来判断，而非 dv 的符号。
    // 后退模式下速度为负，减速（绝对值减小）时 dv > 0，若用 dv 符号会错误应用 accel_limit。
    if (dt > 1e-6) {
      double dv = v - prev_v;
      bool is_same_sign = (v * prev_v >= 0.0);
      bool is_decel = is_same_sign ? (std::abs(v) < std::abs(prev_v)) : true;
      double limit = is_decel ? (params_.linear_decel_limit * dt) : (params_.linear_accel_limit * dt);
      if (limit > 0.0) {
        dv = std::clamp(dv, -limit, limit);
        v = prev_v + dv;
      }
    }

    const double sign = (v >= 0.0) ? 1.0 : -1.0;
    const double abs_v = std::abs(v);
    // 仅限幅最大速度；最小速度由上层规划决定（否则低通+限幅会把近目标速度抬升，造成穿越/绕圈）
    v = sign * std::min(abs_v, params_.max_velocity);

    prev_linear_vel_ = v;
  }

  // 角速度：同样避免“要求停止却有残余角速度”
  if (stop_angular) {
    omega = 0.0;
    prev_angular_vel_ = 0.0;
  } else {
    bool is_first_call = (std::abs(prev_omega) < 1e-12);
    if (!is_first_call) {
      omega = alpha * omega + (1.0 - alpha) * prev_omega;
    }
    omega = std::clamp(omega, -params_.max_angular_vel, params_.max_angular_vel);
    prev_angular_vel_ = omega;
  }
}

}  // namespace follow_controller
}  // namespace xline
