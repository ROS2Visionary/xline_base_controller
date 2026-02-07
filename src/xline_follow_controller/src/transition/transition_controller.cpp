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
  , goal_set_(false)
  , goal_reached_(false)
  , arrival_count_(0)
  , prev_linear_vel_(0.0)
  , prev_angular_vel_(0.0)
  , goal_x_(0.0)
  , goal_y_(0.0)
  , goal_theta_(0.0)
  , goal_theta_set_(false)
  , use_backward_(false)
  , backward_decided_(false)
  , backward_switched_to_forward_(false)
  , prev_distance_(0.0)
  , distance_increasing_count_(0)
  , min_distance_reached_(std::numeric_limits<double>::max())
  , grid_resolution_(0.0003)  // 0.3mm per pixel (高分辨率)
  , grid_width_(0.0)
  , grid_height_(0.0)
  , grid_origin_x_(0.0)
  , grid_origin_y_(0.0)
  , last_stage2_yaw_error_(0.0)
{
  // 基于环境变量初始化默认的栅格图路径
  const char* ws_root = std::getenv("XLINE_WS_ROOT");
  if (ws_root && *ws_root) {
    params_.grid_map_path = ws_root;
  }

  updateParameters();
  last_time_ = std::chrono::steady_clock::now();
  last_grid_update_time_ = this->now();
  last_log_output_time_ = this->now();

  // 创建栅格图保存目录
  if (params_.enable_grid_map) {
    std::filesystem::create_directories(params_.grid_map_path);
    LOG_INFO("栅格图将保存到: %s", params_.grid_map_path.c_str());
  }

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

    // 线速度加减速限制
    params_.linear_accel_limit = parser.hasParameter("transition.linear_accel_limit") ?
        parser.getParameter<double>("transition.linear_accel_limit") : 0.25;
    params_.linear_decel_limit = parser.hasParameter("transition.linear_decel_limit") ?
        parser.getParameter<double>("transition.linear_decel_limit") : 0.35;

    // 控制增益
    params_.k_linear = parser.getParameter<double>("transition.k_linear");
    params_.k_angular = parser.getParameter<double>("transition.k_angular");

    // 到达判定
    params_.arrival_tolerance = parser.getParameter<double>("transition.arrival_tolerance");
    params_.arrival_angle_tolerance = parser.getParameter<double>("transition.arrival_angle_tolerance");
    params_.arrival_confirm_count = parser.getParameter<int>("transition.arrival_confirm_count");

    // 减速控制
    params_.slow_down_distance = parser.getParameter<double>("transition.slow_down_distance");
    params_.creep_velocity = parser.getParameter<double>("transition.creep_velocity");

    // 大转角处理
    params_.large_angle_threshold = parser.getParameter<double>("transition.large_angle_threshold");
    params_.large_angle_vel_ratio = parser.getParameter<double>("transition.large_angle_vel_ratio");

    // 姿态伺服
    params_.pose_servo_distance = parser.getParameter<double>("transition.pose_servo_distance");

    // 阶段2航向对准（原地旋转，rotation 方式）
    params_.rotation_enabled = parser.hasParameter("transition.rotation.enabled") ?
        parser.getParameter<bool>("transition.rotation.enabled") : true;
    params_.rotation_max_w = parser.hasParameter("transition.rotation.max_w") ?
        parser.getParameter<double>("transition.rotation.max_w") : params_.max_angular_vel;
    params_.rotation_min_w = parser.hasParameter("transition.rotation.min_w") ?
        parser.getParameter<double>("transition.rotation.min_w") : std::min(0.1, params_.rotation_max_w);
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

    params_.rotation_max_w = std::clamp(params_.rotation_max_w, 0.0, params_.max_angular_vel);
    params_.rotation_min_w = std::clamp(params_.rotation_min_w, 0.0, params_.rotation_max_w);
    params_.rotation_factor = std::max(0.0, params_.rotation_factor);
    params_.rotation_angle_threshold = std::max(0.0, params_.rotation_angle_threshold);
    params_.rotation_smooth_factor = std::max(0.0, params_.rotation_smooth_factor);
    params_.rotation_stop_tolerance = std::clamp(
        params_.rotation_stop_tolerance, 0.0, params_.arrival_angle_tolerance);
    params_.rotation_crossing_window = std::max(params_.rotation_crossing_window, params_.rotation_stop_tolerance);

    // 后退模式
    params_.enable_backward = parser.getParameter<bool>("transition.enable_backward");
    params_.backward_angle_threshold = parser.getParameter<double>("transition.backward_angle_threshold");

    // 平滑控制
    params_.velocity_smooth_alpha = parser.getParameter<double>("transition.velocity_smooth_alpha");

    // 调试
    params_.debug_enabled = parser.getParameter<bool>("transition.debug_enabled");

    // 栅格图可视化
    params_.enable_grid_map = parser.getParameter<bool>("transition.enable_grid_map");
    if (parser.hasParameter("transition.grid_map_path")) {
      params_.grid_map_path = xline::path_utils::resolve_path(
          parser.getParameter<std::string>("transition.grid_map_path"));
    }

    LOG_INFO("参数加载成功 | 速度限制: v_max=%.2fm/s, ω_max=%.2frad/s | 控制增益: k_lin=%.1f, k_ang=%.1f",
             params_.max_velocity, params_.max_angular_vel, params_.k_linear, params_.k_angular);
    LOG_INFO("  精度: 位置±%.1fcm, 角度±%.1f° | 姿态伺服:%.2fm | 后退模式:%s(阈值%.0f°)",
             params_.arrival_tolerance * 100.0,
             params_.arrival_angle_tolerance * 57.3,
             params_.pose_servo_distance,
             params_.enable_backward ? "启用" : "禁用",
             params_.backward_angle_threshold * 57.3);

  } catch (const std::exception& e) {
    LOG_WARN("参数加载失败，使用默认值: %s", e.what());

    // 默认参数
    params_.max_velocity = 0.5;
    params_.max_angular_vel = 1.0;
    params_.min_velocity = 0.05;
    params_.linear_accel_limit = 0.25;
    params_.linear_decel_limit = 0.35;
    params_.k_linear = 1.0;
    params_.k_angular = 2.5;
    params_.arrival_tolerance = 0.03;
    params_.arrival_angle_tolerance = 0.1;  // 5.7度
    params_.arrival_confirm_count = 3;
    params_.slow_down_distance = 0.5;
    params_.creep_velocity = 0.05;
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
    params_.enable_grid_map = false;
    params_.grid_map_path = "grid_maps";
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

  // 重置后退安全检测
  prev_distance_ = 0.0;
  distance_increasing_count_ = 0;
  min_distance_reached_ = std::numeric_limits<double>::max();
  last_stage2_yaw_error_ = 0.0;

  // 初始化栅格图
  if (params_.enable_grid_map) {
    initializeGridMap();
  }

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

  // 重置后退安全检测
  prev_distance_ = 0.0;
  distance_increasing_count_ = 0;
  min_distance_reached_ = std::numeric_limits<double>::max();
  last_stage2_yaw_error_ = 0.0;

  // 初始化栅格图
  if (params_.enable_grid_map) {
    initializeGridMap();
  }

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
    dt = 0.025;
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
    // 判断控制阶段
    // ============================================================================
    if (distance > params_.arrival_tolerance) {
      // ------------------------------------------------------------
      // 阶段1：位置控制 - 朝向目标位置移动
      // ------------------------------------------------------------
      last_stage2_yaw_error_ = 0.0;

      // 判断是否使用后退模式（可选优化，仅在特定条件下）
      bool should_use_backward = false;
      if (params_.enable_backward && !backward_decided_) {
        // 计算目标在机器人坐标系中的位置
        double dx_world = goal_x_ - curr_x;
        double dy_world = goal_y_ - curr_y;
        double cos_theta = std::cos(curr_theta);
        double sin_theta = std::sin(curr_theta);
        double dx_local = dx_world * cos_theta + dy_world * sin_theta;
        double dy_local = -dx_world * sin_theta + dy_world * cos_theta;

        // 严格条件：目标在正后方且朝向已对准
        bool target_straight_behind = (dx_local < -0.1) &&                                       // 明确在后方
                                       (std::abs(dy_local) < std::abs(dx_local) * 0.5);  // 接近正后方
        bool heading_well_aligned = std::abs(final_heading_error) < M_PI / 6.0;  // 朝向对准<30°

        should_use_backward = target_straight_behind && heading_well_aligned;

        if (should_use_backward) {
          LOG_INFO("✓ 使用后退模式 - 目标在正后方(dx=%.2f,dy=%.2f)，朝向对准(%.1f°)",
                   dx_local, dy_local, final_heading_error * 180.0 / M_PI);
        }
        backward_decided_ = true;
      }

      if (should_use_backward || (use_backward_ && backward_decided_)) {
        // 后退模式（两步法中的“到达位置”阶段）：
        // - 阶段1目标是“到点”，因此航向应基于目标位置计算
        // - 使用“背向目标点的朝向”作为期望朝向，使得负线速度能朝目标收敛
        double approach_heading = computeTargetHeading(curr_x, curr_y);
        double backward_desired_heading = normalizeAngle(approach_heading + M_PI);
        double backward_heading_error = normalizeAngle(backward_desired_heading - curr_theta);

        // 如果偏差过大，先原地旋转对准“后退期望朝向”，避免横向倒车导致绕圈/离目标更远
        if (std::abs(backward_heading_error) > M_PI / 2.0) {
          v = 0.0;
          omega = params_.rotation_enabled ?
              computeRotationVelocity(backward_heading_error) :
              computeAngularVelocity(backward_heading_error);
        } else {
          v = -computeLinearVelocity(distance, backward_heading_error);
          omega = computeAngularVelocity(backward_heading_error);
        }
        use_backward_ = true;
      } else {
        // 前进模式：朝向目标位置
        double approach_heading = computeTargetHeading(curr_x, curr_y);
        double approach_error = normalizeAngle(approach_heading - curr_theta);

        v = computeLinearVelocity(distance, approach_error);
        omega = computeAngularVelocity(approach_error);

        // 如果航向误差过大（>90°），停止前进，只旋转
        if (std::abs(approach_error) > M_PI / 2.0) {
          v = 0.0;
          omega = params_.rotation_enabled ?
              computeRotationVelocity(approach_error) :
              computeAngularVelocity(approach_error);
        }
      }

    } else {
      // ------------------------------------------------------------
      // 阶段2：朝向控制 - 原地旋转对准目标朝向
      // ------------------------------------------------------------
      v = 0.0;  // 停止移动
      // 与 LineFollowController 的“对齐阶段”一致：误差小于阈值时直接置零（避免最小角速度导致抖动计数失败）
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

    // 到达判定：阶段1只检查位置，阶段2同时检查位置和朝向
    double arrival_tolerance_used = params_.arrival_tolerance;

    if (distance < arrival_tolerance_used &&
        std::abs(final_heading_error) < params_.arrival_angle_tolerance) {
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
      LOG_INFO("✓ 到达目标姿态！最终距离: %.4fm, 朝向误差: %.2f° [确认次数: %d/%d]",
               distance, normalizeAngle(goal_theta_ - curr_theta) * 180.0 / M_PI,
               arrival_count_, params_.arrival_confirm_count);
    } else {
      LOG_INFO("✓ 到达目标点！最终距离: %.4fm [确认次数: %d/%d]",
               distance, arrival_count_, params_.arrival_confirm_count);
    }
    return true;
  }

  // 7. 计算速度指令（已在上面计算）

  // 8. 速度平滑
  smoothVelocity(v, omega, dt);

  // 9. 输出
  cmd_vel.twist.linear.x = v;
  cmd_vel.twist.angular.z = omega;

  // 10. 调试输出（每1秒输出一次统一状态日志）
  if (params_.debug_enabled) {
    auto current_time = this->now();
    if ((current_time - last_log_output_time_).seconds() >= 1.0) {
      // 构建统一的状态日志
      std::string mode_str;
      std::string phase_str;
      std::string status_str;
      double heading_error = 0.0;
      double final_heading_error = 0.0;

      if (goal_theta_set_) {
        // 姿态伺服模式 - 分段控制
        final_heading_error = normalizeAngle(goal_theta_ - curr_theta);

        // 判断阶段
        if (distance > params_.arrival_tolerance) {
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
      LOG_INFO("[%s·%s] 位置:(%.3f,%.3f,%.0f°) | 误差: %.3fm/%.1f° | 速度: %.3f,%.3f | 进度:%d%% | 计数:%d/%d",
               mode_str.c_str(), status_str.c_str(),
               curr_x, curr_y, curr_theta * 57.3,
               distance, heading_error * 57.3,
               v, omega,
               progress_pct,
               arrival_count_, params_.arrival_confirm_count);

      last_log_output_time_ = current_time;
    }
  }

  // 11. 更新栅格图可视化
  updateGridMapIfNeeded(pose);

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
  use_backward_ = false;
  backward_decided_ = false;
  backward_switched_to_forward_ = false;
  last_stage2_yaw_error_ = 0.0;

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
  double dy_local = -dx_world * sin_theta + dy_world * cos_theta;

  // 判断1：目标点在机器人后方（x_local < 0）
  bool target_behind = dx_local < 0;

  // 判断2：目标朝向与当前朝向接近
  double heading_error = std::abs(normalizeAngle(goal_theta_ - curr_theta));
  bool heading_aligned = heading_error < params_.backward_angle_threshold;

  // 两个条件都满足才使用后退
  // 后退模式的选择会在第一次判断后在主日志中显示（"姿态伺服-后退xxx"）
  return target_behind && heading_aligned;
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
  } else if (distance > 0.1) {
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

  // 4. 最终限幅（防止缩放后超出范围）
  v = std::clamp(v, params_.min_velocity, params_.max_velocity);

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
  rot_vel = std::clamp(rot_vel, params_.rotation_min_w, params_.rotation_max_w);

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
    if (dt > 1e-6) {
      double dv = v - prev_v;
      double limit = (dv >= 0.0) ? (params_.linear_accel_limit * dt) : (params_.linear_decel_limit * dt);
      if (limit > 0.0) {
        dv = std::clamp(dv, -limit, limit);
        v = prev_v + dv;
      }
    }

    // 严格遵循线速度上下限（对后退同样按幅值约束）
    const double sign = (v >= 0.0) ? 1.0 : -1.0;
    const double abs_v = std::abs(v);
    const double bounded_abs_v = std::clamp(abs_v, params_.min_velocity, params_.max_velocity);
    v = sign * bounded_abs_v;

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

// ============================================================================
// 栅格图可视化实现
// ============================================================================

void TransitionController::initializeGridMap()
{
  if (!goal_set_) {
    LOG_WARN("无法初始化栅格图：未设置目标");
    return;
  }

  // 计算栅格图边界（包括当前位置和目标位置的范围）
  // 假设起点为 (0, 0)，实际可以从 TF 获取
  double min_x = std::min(0.0, goal_x_);
  double max_x = std::max(0.0, goal_x_);
  double min_y = std::min(0.0, goal_y_);
  double max_y = std::max(0.0, goal_y_);

  // 动态边距：根据地图范围自适应
  double range_x = max_x - min_x;
  double range_y = max_y - min_y;
  double max_range = std::max(range_x, range_y);

  // 边距为地图范围的10%，但限制在50mm~200mm之间
  double margin = std::clamp(max_range * 0.1, 0.05, 0.2);

  grid_width_ = (max_x - min_x) + 2 * margin;
  grid_height_ = (max_y - min_y) + 2 * margin;
  grid_origin_x_ = min_x - margin;
  grid_origin_y_ = min_y - margin;

  // 智能分辨率调整：根据地图大小自动调整，防止图像过大
  double base_resolution = 0.0003;  // 基础分辨率 0.3mm/px
  double map_diagonal = std::hypot(grid_width_, grid_height_);

  // 根据地图对角线长度调整分辨率
  if (map_diagonal > 5.0) {
    grid_resolution_ = 0.002;  // 超大地图: 2.0mm/px
  } else if (map_diagonal > 3.0) {
    grid_resolution_ = 0.001;  // 大地图: 1.0mm/px
  } else if (map_diagonal > 1.5) {
    grid_resolution_ = 0.0005;  // 中地图: 0.5mm/px
  } else {
    grid_resolution_ = base_resolution;  // 小地图: 0.3mm/px (高精度)
  }

  // 创建栅格图
  int width_pixels = static_cast<int>(grid_width_ / grid_resolution_);
  int height_pixels = static_cast<int>(grid_height_ / grid_resolution_);

  // 安全限制：最大图像尺寸10000x10000像素（约286MB）
  const int MAX_PIXELS = 10000;
  if (width_pixels > MAX_PIXELS || height_pixels > MAX_PIXELS) {
    double scale = std::max(
      static_cast<double>(width_pixels) / MAX_PIXELS,
      static_cast<double>(height_pixels) / MAX_PIXELS
    );
    grid_resolution_ *= scale;
    width_pixels = static_cast<int>(grid_width_ / grid_resolution_);
    height_pixels = static_cast<int>(grid_height_ / grid_resolution_);
    LOG_WARN("栅格图尺寸过大，已自动降低分辨率至 %.2fmm/px", grid_resolution_ * 1000.0);
  }
  grid_map_ = cv::Mat(height_pixels, width_pixels, CV_8UC3, cv::Scalar(255, 255, 255));

  // 绘制栅格线和目标点
  drawGridLines();
  drawGoalOnGrid();

  // 清空轨迹
  trajectory_.clear();

  last_grid_update_time_ = this->now();
  saveGridMap();

  // 计算栅格线间隔（与drawGridLines保持一致）
  double grid_interval = (map_diagonal > 5.0) ? 0.1 :
                         (map_diagonal > 3.0) ? 0.05 :
                         (map_diagonal > 1.5) ? 0.02 : 0.01;

  LOG_INFO("初始化栅格图 大小: %.2f x %.2f m, 尺寸: %d x %d 像素, "
           "分辨率: %.2fmm/px, 栅格间隔: %.0fmm, 边距: %.0fmm, 内存: %.1fMB",
           grid_width_, grid_height_, width_pixels, height_pixels,
           grid_resolution_ * 1000.0, grid_interval * 1000.0, margin * 1000.0,
           width_pixels * height_pixels * 3.0 / (1024.0 * 1024.0));
}

cv::Point TransitionController::worldToGrid(double x, double y)
{
  int grid_x = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
  int grid_y = grid_map_.rows - static_cast<int>((y - grid_origin_y_) / grid_resolution_) - 1;
  return cv::Point(grid_x, grid_y);
}

void TransitionController::drawGoalOnGrid()
{
  if (grid_map_.empty()) {
    return;
  }

  cv::Point goal_pt = worldToGrid(goal_x_, goal_y_);
  if (isPointInGrid(goal_pt)) {
    // 绘制目标点（蓝色圆，适应高分辨率）
    cv::circle(grid_map_, goal_pt, 8, cv::Scalar(255, 0, 0), -1);  // Blue goal

    // 如果设置了目标朝向，绘制朝向箭头
    if (goal_theta_set_) {
      double arrow_length = 0.02;  // 20mm (缩小箭头)
      double end_x = goal_x_ + arrow_length * std::cos(goal_theta_);
      double end_y = goal_y_ + arrow_length * std::sin(goal_theta_);
      cv::Point arrow_end = worldToGrid(end_x, end_y);
      if (isPointInGrid(arrow_end)) {
        cv::arrowedLine(grid_map_, goal_pt, arrow_end, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, 0, 0.3);
      }
    }
  }
}

void TransitionController::drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose)
{
  if (grid_map_.empty()) {
    return;
  }

  cv::Point robot_pt = worldToGrid(pose.pose.position.x, pose.pose.position.y);
  if (!isPointInGrid(robot_pt)) {
    return;
  }

  // ============================================================================
  // 关键修复：只绘制新增的轨迹段，不重复绘制已有轨迹
  // ============================================================================

  // 1. 绘制新增的轨迹线段（从上一个点到当前点）
  if (!trajectory_.empty()) {
    cv::Point last_pt = trajectory_.back();
    if (isPointInGrid(last_pt)) {
      // 只绘制新增的线段（绿色轨迹线，适应高分辨率）
      cv::line(grid_map_, last_pt, robot_pt, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    }
  }

  // 2. 记录当前位置到轨迹
  trajectory_.push_back(robot_pt);

  // 3. 绘制机器人当前位置（橙色圆 + 箭头，适应高分辨率）
  //    注意：这会累积在图上，显示机器人的历史位置
  //    如果只想显示最新位置，需要每次重绘整个栅格图（性能较差）
  cv::circle(grid_map_, robot_pt, 5, cv::Scalar(0, 165, 255), -1);  // Orange robot

  // 4. 绘制机器人朝向箭头
  double theta = tf2::getYaw(pose.pose.orientation);
  double arrow_length = 0.01;  // 10mm (缩小箭头)
  double end_x = pose.pose.position.x + arrow_length * std::cos(theta);
  double end_y = pose.pose.position.y + arrow_length * std::sin(theta);
  cv::Point arrow_end = worldToGrid(end_x, end_y);
  if (isPointInGrid(arrow_end)) {
    cv::arrowedLine(grid_map_, robot_pt, arrow_end, cv::Scalar(0, 100, 255), 1, cv::LINE_AA, 0, 0.2);
  }
}

void TransitionController::drawGridLines()
{
  if (grid_map_.empty()) {
    return;
  }

  cv::Scalar grid_color(220, 220, 220);

  // 根据地图大小智能选择栅格线间隔
  double map_diagonal = std::hypot(grid_width_, grid_height_);
  double grid_interval;

  if (map_diagonal > 5.0) {
    grid_interval = 0.1;  // 超大地图: 100mm间隔
  } else if (map_diagonal > 3.0) {
    grid_interval = 0.05;  // 大地图: 50mm间隔
  } else if (map_diagonal > 1.5) {
    grid_interval = 0.02;  // 中地图: 20mm间隔
  } else {
    grid_interval = 0.01;  // 小地图: 10mm间隔
  }

  // 绘制垂直栅格线
  for (double x = grid_origin_x_; x < grid_origin_x_ + grid_width_; x += grid_interval) {
    cv::Point pt1 = worldToGrid(x, grid_origin_y_);
    cv::Point pt2 = worldToGrid(x, grid_origin_y_ + grid_height_);
    if (isPointInGrid(pt1) && isPointInGrid(pt2)) {
      cv::line(grid_map_, pt1, pt2, grid_color, 1);
    }
  }

  // 绘制水平栅格线
  for (double y = grid_origin_y_; y < grid_origin_y_ + grid_height_; y += grid_interval) {
    cv::Point pt1 = worldToGrid(grid_origin_x_, y);
    cv::Point pt2 = worldToGrid(grid_origin_x_ + grid_width_, y);
    if (isPointInGrid(pt1) && isPointInGrid(pt2)) {
      cv::line(grid_map_, pt1, pt2, grid_color, 1);
    }
  }
}

void TransitionController::saveGridMap()
{
  if (grid_map_.empty() || params_.grid_map_path.empty()) {
    return;
  }

  std::string filename = params_.grid_map_path + "/transition_path_tracking.png";
  cv::imwrite(filename, grid_map_);
}

void TransitionController::updateGridMapIfNeeded(const geometry_msgs::msg::PoseStamped& current_pose)
{
  if (!params_.enable_grid_map) {
    return;
  }

  auto current_time = this->now();
  if ((current_time - last_grid_update_time_).seconds() > 1.0) {  // 每1s保存一次
    drawRobotOnGrid(current_pose);
    saveGridMap();
    last_grid_update_time_ = current_time;
  }
}

bool TransitionController::isPointInGrid(const cv::Point& pt)
{
  return pt.x >= 0 && pt.x < grid_map_.cols && pt.y >= 0 && pt.y < grid_map_.rows;
}

}  // namespace follow_controller
}  // namespace xline
