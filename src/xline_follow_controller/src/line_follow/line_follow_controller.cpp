// 直线路径跟随控制器的具体实现。
// 主要流程：
// 1. 从 YAML 中加载参数到 LineParams 结构体；
// 2. 根据路径长度和工作模式动态调整加速/减速/对齐距离；
// 3. 使用带多层滤波的 PID 控制角速度，Sigmoid 曲线控制线速度；
// 4. 在起始对齐 -> 路径跟随 -> 终点对齐之间切换状态。

#include "xline_follow_controller/line_follow/line_follow_controller.hpp"
#include "xline_follow_controller/common/yaml_parser.hpp"
#include "xline_follow_controller/common/path_utils.hpp"
#include <angles/angles.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <thread>
#include <functional>

namespace xline
{
namespace follow_controller
{

// 构造函数：只做一次性的初始化工作，
// 包括参数加载、滤波器初始化以及 PID/时间戳设置。
LineFollowController::LineFollowController()
  : BaseFollowController("line_follow_controller")
  , current_state_(ControlState::IDLE)
  , received_plan_(false)
  , goal_reached_(false)
  , back_follow_(false)
  , m_work_state_(true)
  , short_path_(false)
  , decel_phase_entered_(false)
  , is_transition_path_(false)
  , current_waypoint_index_(0)
  , path_length_(0.0)
  , robot_yaw_(0.0)
  , current_linear_speed_(0.0)
  , current_angular_speed_(0.0)
  , prev_angular_velocity_(0.0)
  , last_yaw_error_(0.0)
  , angular_vel_hampel_filter_(5, 3.0)
  , wait_duration_(0.2)
  , waiting_(false)
  , angular_smoother_(2.0, 0.7)
  , start_line_aligned_(false)
  , start_line_aligned_count_(0)
  , accel_start_distance_to_start_(0.0)
{
  updateParameters();
  initializeFilters();

  heading_pid_controller_ = std::make_shared<PIDController>(
      followingPID().kp, followingPID().ki, followingPID().kd, 0.2, -0.2);
  heading_pid_controller_->setIntegralLimits(-0.05, 0.05);

  last_time_ = std::chrono::steady_clock::now();
  wait_start_time_ = std::chrono::steady_clock::now();
}

LineFollowController::~LineFollowController()
{
}

void LineFollowController::setTransitionPath(bool is_transition)
{
  is_transition_path_ = is_transition;
  LOG_INFO("设置转场路径: %d", is_transition_path_);
}


void LineFollowController::updateParameters()
{
  // 从 ament_index 中查找安装路径，再拼接出 line.yaml 的完整路径
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("xline_follow_controller");
  std::string config_file_path = package_share_directory + "/config/line.yaml";

  xline::YamlParser::YamlParser parser(config_file_path);

  try
  {
    // ================================
    // 1. 调试路径相关参数
    // ================================
    // 调试参数 (debug.*)
    params_.debug.enabled = parser.getParameter<bool>("debug.enabled");
    params_.debug.raw_path = parser.getParameter<std::string>("debug.raw_path");
    params_.debug.filtered_path = parser.getParameter<std::string>("debug.filtered_path");
    
    resolved_debug_raw_dir_ = xline::path_utils::resolve_path(params_.debug.raw_path);
    resolved_debug_filtered_dir_ = xline::path_utils::resolve_path(params_.debug.filtered_path);

    // ================================
    // 2. 航向 PID 参数
    // ================================
    // 航向PID参数 (heading_pid.*)
    params_.heading_pid.alignment.kp = parser.getParameter<double>("heading_pid.alignment.kp");
    params_.heading_pid.alignment.ki = parser.getParameter<double>("heading_pid.alignment.ki");
    params_.heading_pid.alignment.kd = parser.getParameter<double>("heading_pid.alignment.kd");
    
    params_.heading_pid.following.kp = parser.getParameter<double>("heading_pid.following.kp");
    params_.heading_pid.following.ki = parser.getParameter<double>("heading_pid.following.ki");
    params_.heading_pid.following.kd = parser.getParameter<double>("heading_pid.following.kd");

    // ================================
    // 3. 速度参数
    // ================================
    // 速度参数 (velocity.*)
    params_.velocity.accel_min = parser.getParameter<double>("velocity.accel_min");
    params_.velocity.decel_min = parser.getParameter<double>("velocity.decel_min");
    params_.velocity.walk_max = parser.getParameter<double>("velocity.walk_max");
    params_.velocity.work_max = parser.getParameter<double>("velocity.work_max");
    params_.velocity.alignment = parser.getParameter<double>("velocity.alignment");
    params_.velocity.acce_factor = parser.getParameter<double>("velocity.acce_factor");
    params_.velocity.accel_sigmoid_k = parser.getParameter<double>("velocity.accel_sigmoid_k");
    params_.velocity.decel_sigmoid_k = parser.getParameter<double>("velocity.decel_sigmoid_k");
    params_.velocity.accel_sigmoid_center = parser.getParameter<double>("velocity.accel_sigmoid_center");
    params_.velocity.decel_sigmoid_center = parser.getParameter<double>("velocity.decel_sigmoid_center");

    // ================================
    // 4. 距离参数
    // ================================
    // 距离参数 (distance.*)
    params_.distance.alignment = parser.getParameter<double>("distance.alignment");
    params_.distance.deceleration = parser.getParameter<double>("distance.deceleration");
    params_.distance.acceleration = parser.getParameter<double>("distance.acceleration");
    params_.distance.lookahead = parser.getParameter<double>("distance.lookahead");
    params_.distance.waypoint_tolerance = parser.getParameter<double>("distance.waypoint_tolerance");

    // ================================
    // 5. 原地旋转参数
    // ================================
    // 旋转参数 (rotation.*)
    params_.rotation.max_w = parser.getParameter<double>("rotation.max_w");
    params_.rotation.min_w = parser.getParameter<double>("rotation.min_w");
    params_.rotation.factor = parser.getParameter<double>("rotation.factor");
    params_.rotation.angle_threshold = parser.getParameter<double>("rotation.angle_threshold");
    params_.rotation.smooth_factor = parser.getParameter<double>("rotation.smooth_factor");

    // ================================
    // 6. 滤波器参数
    // ================================
    // 滤波器参数 (filter.*)
    params_.filter.pos_hampel_window = parser.getParameter<int>("filter.pos_hampel_window");
    params_.filter.pos_hampel_k = parser.getParameter<double>("filter.pos_hampel_k");
    params_.filter.pos_savgol_window = parser.getParameter<int>("filter.pos_savgol_window");
    params_.filter.pos_savgol_order = parser.getParameter<int>("filter.pos_savgol_order");
    params_.filter.angular_hampel_window = parser.getParameter<int>("filter.angular_hampel_window");
    params_.filter.angular_hampel_k = parser.getParameter<double>("filter.angular_hampel_k");

    // ================================
    // 7. 阶段参数（对齐/跟随）
    // ================================
    // 阶段参数 (phase.*)
    params_.phase.alignment.max_angular_vel = parser.getParameter<double>("phase.alignment.max_angular_vel");
    params_.phase.alignment.max_angular_accel = parser.getParameter<double>("phase.alignment.max_angular_accel");
    params_.phase.alignment.current_heading_weight = parser.getParameter<double>("phase.alignment.current_heading_weight");
    params_.phase.alignment.target_heading_weight = parser.getParameter<double>("phase.alignment.target_heading_weight");
    params_.phase.alignment.filter_alpha = parser.getParameter<double>("phase.alignment.filter_alpha");
    params_.phase.alignment.smoother_freq = parser.getParameter<double>("phase.alignment.smoother_freq");
    params_.phase.alignment.smoother_damping = parser.getParameter<double>("phase.alignment.smoother_damping");
    params_.phase.alignment.line_cross_track_tolerance = parser.getParameter<double>("phase.alignment.line_cross_track_tolerance");
    params_.phase.alignment.line_heading_tolerance = parser.getParameter<double>("phase.alignment.line_heading_tolerance");
    params_.phase.alignment.line_alignment_stable_count = parser.getParameter<int>("phase.alignment.line_alignment_stable_count");

    params_.phase.following.max_angular_vel = parser.getParameter<double>("phase.following.max_angular_vel");
    params_.phase.following.max_angular_accel = parser.getParameter<double>("phase.following.max_angular_accel");
    params_.phase.following.current_heading_weight = parser.getParameter<double>("phase.following.current_heading_weight");
    params_.phase.following.target_heading_weight = parser.getParameter<double>("phase.following.target_heading_weight");
    params_.phase.following.filter_alpha = parser.getParameter<double>("phase.following.filter_alpha");
    params_.phase.following.smoother_freq = parser.getParameter<double>("phase.following.smoother_freq");
    params_.phase.following.smoother_damping = parser.getParameter<double>("phase.following.smoother_damping");

    // ================================
    // 8. 制导参数（回线/进线）
    // ================================
    if (parser.hasParameter("guidance.mode"))
    {
      params_.guidance.mode = parser.getParameter<std::string>("guidance.mode");
      std::transform(params_.guidance.mode.begin(), params_.guidance.mode.end(), params_.guidance.mode.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    }
    if (parser.hasParameter("guidance.alignment_mode"))
    {
      params_.guidance.alignment_mode = parser.getParameter<std::string>("guidance.alignment_mode");
      std::transform(params_.guidance.alignment_mode.begin(), params_.guidance.alignment_mode.end(),
                     params_.guidance.alignment_mode.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    }
    if (parser.hasParameter("guidance.following_mode"))
    {
      params_.guidance.following_mode = parser.getParameter<std::string>("guidance.following_mode");
      std::transform(params_.guidance.following_mode.begin(), params_.guidance.following_mode.end(),
                     params_.guidance.following_mode.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    }
    if (parser.hasParameter("guidance.stanley_k"))
    {
      params_.guidance.stanley_k = parser.getParameter<double>("guidance.stanley_k");
    }
    if (parser.hasParameter("guidance.stanley_v0"))
    {
      params_.guidance.stanley_v0 = parser.getParameter<double>("guidance.stanley_v0");
    }
    if (parser.hasParameter("guidance.theta_max"))
    {
      params_.guidance.theta_max = parser.getParameter<double>("guidance.theta_max");
    }

    // 同步到运行时变量
    syncRuntimeParams();
  }
  catch (const std::exception& e)
  {
    LOG_ERROR("参数加载失败: %s", e.what());
    throw;
  }
}

void LineFollowController::syncRuntimeParams()
{
  // 将 YAML 里的“静态配置”映射到方便运行时使用的成员变量：
  // - 速度/距离等标量参数；
  // - 两个阶段（对齐/跟随）的运行时配置；
  // - 滤波器和平滑器的参数；
  // - PID 控制器的系数。
  // 速度参数
  runtime_accel_min_vel_ = params_.velocity.accel_min;
  runtime_decel_min_vel_ = params_.velocity.decel_min;
  runtime_walk_max_vel_ = params_.velocity.walk_max;
  runtime_work_max_vel_ = params_.velocity.work_max;
  runtime_alignment_vel_ = params_.velocity.alignment;

  // 距离参数
  runtime_alignment_dist_ = params_.distance.alignment;
  runtime_accel_dist_ = params_.distance.acceleration;
  runtime_decel_dist_ = params_.distance.deceleration;

  // 阶段运行时参数
  following_params_ = LinePhaseRuntimeParams::fromConfig(params_.phase.following);
  alignment_params_ = LinePhaseRuntimeParams::fromConfig(params_.phase.alignment);

  // 滤波参数
  alpha_ = following_params_.filter_alpha;

  // 二阶平滑器
  angular_smoother_.setParameters(alignment_params_.smoother_freq, alignment_params_.smoother_damping);

  // PID控制器
  if (heading_pid_controller_)
  {
    heading_pid_controller_->setGains(alignmentPID().kp, alignmentPID().ki, alignmentPID().kd);
  }
}

void LineFollowController::initializeFilters()
{
  // 根据配置重新初始化所有滤波器。
  // 由于窗口大小、阶数等可能在参数更新后变化，这里统一 reset。
  x_filter_.reset(params_.filter.pos_savgol_window, params_.filter.pos_savgol_order);
  y_filter_.reset(params_.filter.pos_savgol_window, params_.filter.pos_savgol_order);
  h_x_filter_.reset(params_.filter.pos_hampel_window, params_.filter.pos_hampel_k);
  h_y_filter_.reset(params_.filter.pos_hampel_window, params_.filter.pos_hampel_k);
  angular_vel_hampel_filter_.reset(params_.filter.angular_hampel_window, params_.filter.angular_hampel_k);
}

void LineFollowController::resetControllerState()
{
  // 将状态机与速度等运行时状态恢复到“未开始跟随”的初始状态，
  // 不会重新加载 YAML，只是清理缓存。
  current_state_ = ControlState::IDLE;
  goal_reached_ = false;
  back_follow_ = false;
  waiting_ = false;
  current_waypoint_index_ = 0;
  current_linear_speed_ = 0.0;
  current_angular_speed_ = 0.0;
  last_yaw_error_ = 0.0;
  decel_phase_entered_ = false;
  start_line_aligned_ = false;
  start_line_aligned_count_ = 0;
  accel_start_distance_to_start_ = 0.0;

  prev_angular_velocity_ = 0.0;
  second_prev_angular_velocity_ = 0.0;
  prev_smoothed_angular_velocity_ = 0.0;
  angular_vel_history_.clear();

  if (heading_pid_controller_)
  {
    heading_pid_controller_->reset();
  }

  angular_smoother_.reset();
  last_time_ = std::chrono::steady_clock::now();
}

void LineFollowController::updateStartLineAlignment(double robot_x, double robot_y, double robot_yaw, double distance_to_start)
{
  if (start_line_aligned_)
  {
    return;
  }

  if (alignment_params_.line_alignment_stable_count <= 0)
  {
    start_line_aligned_ = true;
    accel_start_distance_to_start_ = distance_to_start;
    return;
  }

  const double cross_track_error = computeCrossTrackError(robot_x, robot_y);

  double path_ideal_yaw = tf2::getYaw(original_start_pose_.pose.orientation);
  if (back_follow_)
  {
    path_ideal_yaw = normalizeAngle(path_ideal_yaw + M_PI);
  }
  const double heading_error = angles::shortest_angular_distance(robot_yaw, path_ideal_yaw);

  const bool cross_track_ok = std::abs(cross_track_error) <= alignment_params_.line_cross_track_tolerance;
  const bool heading_ok = std::abs(heading_error) <= alignment_params_.line_heading_tolerance;

  if (cross_track_ok && heading_ok)
  {
    ++start_line_aligned_count_;
  }
  else
  {
    start_line_aligned_count_ = 0;
  }

  if (start_line_aligned_count_ >= alignment_params_.line_alignment_stable_count)
  {
    start_line_aligned_ = true;
    accel_start_distance_to_start_ = distance_to_start;

    heading_pid_controller_->setGains(followingPID().kp, followingPID().ki, followingPID().kd);
    angular_smoother_.setParameters(following_params_.smoother_freq, following_params_.smoother_damping);

    if (heading_pid_controller_)
    {
      heading_pid_controller_->reset();
    }
    angular_smoother_.reset();
    prev_smoothed_angular_velocity_ = 0.0;
    prev_angular_velocity_ = 0.0;

    LOG_INFO("起步对齐完成: cross_track=%.4f(m), heading_err=%.4f(rad), stable=%d",
             cross_track_error, heading_error, start_line_aligned_count_);
  }
}

// ============================================================================
// 路径设置
// ============================================================================

bool LineFollowController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
  // 主入口：接收一条只包含起点和终点的路径（或简单直线），
  // 完成路径长度统计、短路径判断和一系列运行参数的自适应调整。
  if (orig_global_plan.poses.empty())
  {
    LOG_ERROR("接收到空路径");
    return false;
  }

  double raw_path_length = 0.0;
  for (size_t i = 0; i + 1 < orig_global_plan.poses.size(); ++i)
  {
    const auto& p1 = orig_global_plan.poses[i].pose.position;
    const auto& p2 = orig_global_plan.poses[i + 1].pose.position;

    if (!std::isfinite(p1.x) || !std::isfinite(p1.y) || 
        !std::isfinite(p2.x) || !std::isfinite(p2.y))
    {
      continue;
    }

    raw_path_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
  }
  LOG_INFO("接收到路径: 点数=%zu, 总长度=%.3fm", orig_global_plan.poses.size(), raw_path_length);

  global_plan_ = orig_global_plan;
  original_target_pose_ = global_plan_.poses.back();
  original_start_pose_ = global_plan_.poses.front();
  target_pose_ = global_plan_.poses.back();
  end_pose_ = target_pose_;

  double dx = target_pose_.pose.position.x - original_start_pose_.pose.position.x;
  double dy = target_pose_.pose.position.y - original_start_pose_.pose.position.y;
  original_path_length_ = std::sqrt(dx * dx + dy * dy);

  if (isDebugEnabled())
  {
    original_path_.clear();
    filtered_path_.clear();
    original_path_.push_back({ original_start_pose_.pose.position.x, original_start_pose_.pose.position.y });
    original_path_.push_back({ target_pose_.pose.position.x, target_pose_.pose.position.y });
    filtered_path_.push_back({ original_start_pose_.pose.position.x, original_start_pose_.pose.position.y });
    filtered_path_.push_back({ target_pose_.pose.position.x, target_pose_.pose.position.y });
  }

  extendPath();

  dx = target_pose_.pose.position.x - original_start_pose_.pose.position.x;
  dy = target_pose_.pose.position.y - original_start_pose_.pose.position.y;
  path_length_ = std::sqrt(dx * dx + dy * dy);

  updateParameters();

  double original_alignment_dist = runtime_alignment_dist_;
  double original_decel_dist = runtime_decel_dist_;
  double original_accel_dist = runtime_accel_dist_;

  double total_required_distance = original_alignment_dist + original_decel_dist + original_accel_dist;
  double safety_margin = std::max(0.1, total_required_distance * 0.1);
  short_path_ = (path_length_ < (total_required_distance + safety_margin));

  if (short_path_)
  {
    if (path_length_ < 0.1)
    {
      runtime_alignment_dist_ = path_length_ * 0.6;
      runtime_decel_dist_ = path_length_ * 0.4;
      runtime_accel_dist_ = 0;
    }
    else if (path_length_ < 0.2)
    {
      runtime_alignment_dist_ = path_length_ * 0.5;
      runtime_decel_dist_ = path_length_ * 0.5;
      runtime_accel_dist_ = 0;
    }
    else if (path_length_ < 0.4)
    {
      runtime_alignment_dist_ = path_length_ * 0.3;
      runtime_decel_dist_ = path_length_ * 0.5;
      runtime_accel_dist_ = path_length_ * 0.2;
    }
    else
    {
      runtime_alignment_dist_ = path_length_ * 0.25;
      runtime_decel_dist_ = path_length_ * 0.5;
      runtime_accel_dist_ = path_length_ * 0.25;
    }
  }
  else
  {
    double current_max_speed = m_work_state_ ? runtime_work_max_vel_ : runtime_walk_max_vel_;
    const double speed_threshold = 0.3;

    if (current_max_speed > speed_threshold)
    {
      double total_control_distance = original_alignment_dist + original_decel_dist + original_accel_dist;
      double available_high_speed_distance = path_length_ - total_control_distance;

      if (available_high_speed_distance < path_length_ * 0.3)
      {
        double speed_reduction_factor = std::max(0.5, available_high_speed_distance / (path_length_ * 0.3));
        double optimized_max_speed = speed_threshold + (current_max_speed - speed_threshold) * speed_reduction_factor;

        if (m_work_state_)
        {
          runtime_work_max_vel_ = optimized_max_speed;
        }
        else
        {
          runtime_walk_max_vel_ = optimized_max_speed;
        }

        double speed_ratio = optimized_max_speed / speed_threshold;
        runtime_decel_dist_ = original_decel_dist * std::pow(speed_ratio, 1.5);
        runtime_decel_dist_ = std::min(runtime_decel_dist_, path_length_ * 0.4);

        double remaining_distance = path_length_ - runtime_alignment_dist_ - runtime_decel_dist_;
        runtime_accel_dist_ = std::max(0.0, remaining_distance * 0.6);
      }
      else if (current_max_speed > speed_threshold * 1.5)
      {
        double conservative_factor = std::min(1.0, path_length_ / (total_control_distance * 2.0));
        double optimized_max_speed = speed_threshold * 1.5 * conservative_factor +
                                     speed_threshold * 0.5 * (1.0 - conservative_factor);

        if (m_work_state_)
        {
          runtime_work_max_vel_ = optimized_max_speed;
        }
        else
        {
          runtime_walk_max_vel_ = optimized_max_speed;
        }

        double speed_ratio = optimized_max_speed / speed_threshold;
        runtime_decel_dist_ = original_decel_dist * speed_ratio;
      }
    }
  }

  double current_max_speed = m_work_state_ ? runtime_work_max_vel_ : runtime_walk_max_vel_;
  double min_decel_distance = std::max(0.15, current_max_speed * 0.5);
  runtime_decel_dist_ = std::max(runtime_decel_dist_, min_decel_distance);

  initializeFilters();
  resetControllerState();

  start_print = false;
  stop_print = true;

  received_plan_ = true;
  current_state_ = ControlState::ALIGNING_START;

  return true;
}

bool LineFollowController::setPlan(const std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>>& plan)
{
  // 适配上层传入的一维 Pose 数组，转换成 nav_msgs::Path 再复用主 setPlan 逻辑。
  if (!plan || plan->empty())
  {
    LOG_ERROR("接收到空路径或空指针");
    return false;
  }

  nav_msgs::msg::Path new_global_plan;
  new_global_plan.header.frame_id = "world";

  for (const auto& pose : *plan)
  {
    new_global_plan.poses.push_back(pose);
  }

  end_pose_ = new_global_plan.poses.back();
  return setPlan(new_global_plan);
}

bool LineFollowController::setPlan(double start_x, double start_y, double end_x, double end_y)
{
  // 通过简单的起点/终点坐标生成一条直线路径，
  // 起点朝向对齐到终点方向，便于后续对齐逻辑使用。
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "world";

  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double length = std::hypot(dx, dy);

  const double yaw = std::atan2(dy, dx);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header = path.header;
  start_pose.pose.position.x = start_x;
  start_pose.pose.position.y = start_y;
  start_pose.pose.position.z = 0.0;
  start_pose.pose.orientation = tf2::toMsg(q);
  path.poses.push_back(start_pose);

  constexpr double interval = 0.003;
  if (length > 1e-9)
  {
    const int steps = static_cast<int>(length / interval);
    const double ux = dx / length;
    const double uy = dy / length;

    for (int i = 1; i < steps; ++i)
    {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      const double s = interval * static_cast<double>(i);
      p.pose.position.x = start_x + ux * s;
      p.pose.position.y = start_y + uy * s;
      p.pose.position.z = 0.0;
      p.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(p);
    }
  }

  geometry_msgs::msg::PoseStamped end_pose;
  end_pose.header = path.header;
  end_pose.pose.position.x = end_x;
  end_pose.pose.position.y = end_y;
  end_pose.pose.position.z = 0.0;
  end_pose.pose.orientation = tf2::toMsg(q);
  path.poses.push_back(end_pose);

  return setPlan(path);
}

void LineFollowController::setSpeedLimit(const double& speed_limit)
{
  // 提供一个外部“总速度限制”入口：
  // - 同时压缩工作/非工作最大速度；
  // - 对阶段权重做一点调整，让低速情况下更注重当前航向；
  // - 并对对齐阶段速度做下限保护。
  if (speed_limit > 0.0)
  {
    runtime_work_max_vel_ = std::min(runtime_work_max_vel_, speed_limit);
    runtime_walk_max_vel_ = std::min(runtime_walk_max_vel_, speed_limit);
    runtime_accel_min_vel_ = speed_limit;
    runtime_decel_min_vel_ = speed_limit * 0.5;

    alignment_params_.current_heading_weight = 0.8;
    alignment_params_.target_heading_weight = 0.2;
    following_params_.current_heading_weight = 0.8;
    following_params_.target_heading_weight = 0.2;

    runtime_alignment_vel_ = std::min(runtime_alignment_vel_, std::max(speed_limit * 0.5, 0.08));

    LOG_INFO("速度限制已设置: limit=%.2f, work_max=%.2f, walk_max=%.2f, alignment=%.2f",
             speed_limit, runtime_work_max_vel_, runtime_walk_max_vel_, runtime_alignment_vel_);
  }
}

void LineFollowController::setWorkState(bool state)
{
  m_work_state_ = state;

  if (!m_work_state_)
  {
    bool should_back = shouldGoBackward(
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        tf2::getYaw(current_pose_.pose.orientation),
        end_pose_.pose.position.x, end_pose_.pose.position.y,
        tf2::getYaw(end_pose_.pose.orientation));

    double distance = std::sqrt(
        std::pow(current_pose_.pose.position.x - end_pose_.pose.position.x, 2) +
        std::pow(current_pose_.pose.position.y - end_pose_.pose.position.y, 2));

    if (should_back && distance < 1.5)
    {
      setBackFollow(true);
    }
    else
    {
      setBackFollow(false);
    }
  }
}

void LineFollowController::setBackFollow(bool back)
{
  back_follow_ = back;
  LOG_INFO("设置后退模式: %d", back);
}

void LineFollowController::setPose(const geometry_msgs::msg::PoseStamped& pose)
{
  current_pose_ = pose;
}

bool LineFollowController::isGoalReached()
{
  return goal_reached_;
}

bool LineFollowController::cancel()
{
  resetControllerState();
  return true;
}


double LineFollowController::computeLinearSpeed(double distance_to_target, double distance_to_start)
{
  // 线速度规划核心：
  // - 远离终点时：根据起点距离用 Sigmoid 做渐进加速；
  // - 接近终点时：根据终点距离用另一条 Sigmoid 曲线平滑减速；
  // - 短路径会进一步压缩最大速度；
  // - 还会对每一帧的减速度做上限，避免速度突变。
  static double prev_speed = 0.0;
  double target_speed = 0.0;

  if (distance_to_target < runtime_decel_dist_)
  {
    if (!decel_phase_entered_)
    {
      max_linear_speed_ = prev_speed;
      decel_phase_entered_ = true;
      prev_speed = 0.0;
    }

    const double precise_stop_distance = m_work_state_ ? 0.05 : 0.1;

    if (distance_to_target <= precise_stop_distance)
    {
      target_speed = runtime_decel_min_vel_;
    }
    else
    {
      double normalized_distance =
          (distance_to_target - precise_stop_distance) / (runtime_decel_dist_ - precise_stop_distance);
      normalized_distance = std::max(0.0, std::min(1.0, normalized_distance));
      double sigmoid_value =
          1.0 / (1.0 + std::exp(-decelSigmoidK() * (normalized_distance - decelSigmoidCenter())));
      target_speed = runtime_decel_min_vel_ + (max_linear_speed_ - runtime_decel_min_vel_) * sigmoid_value;
    }
  }
  else
  {
    // 线速度“加速时机”额外约束：
    // - 在 distance_to_start < 0.2m 之前，即使已经满足加速标准（start_line_aligned_==true），也不允许进入加速段；
    // - 一旦 distance_to_start > 0.2m，如果仍未满足加速标准，则维持现有逻辑（对齐段低速），不做额外延后；
    // - 注意：该约束只限制线速度，角速度控制保持原有逻辑。
    constexpr double kMinAccelStartDistance = 0.2;
    const bool allow_linear_accel_by_distance = is_transition_path_ ? true : (distance_to_start >= kMinAccelStartDistance);

    if (current_state_ == ControlState::ALIGNING_START || !start_line_aligned_ || !allow_linear_accel_by_distance)
    {
      target_speed = prev_speed + acceFactor();
      max_linear_speed_ = runtime_alignment_vel_;
    }
    else
    {
      max_linear_speed_ = m_work_state_ ? runtime_work_max_vel_ : runtime_walk_max_vel_;

      if (short_path_)
      {
        if (path_length_ < 0.1)
        {
          max_linear_speed_ = runtime_accel_min_vel_;
        }
        else if (path_length_ < 0.2)
        {
          max_linear_speed_ = runtime_accel_min_vel_ * 1.5;
        }
        else if (path_length_ < 0.4)
        {
          max_linear_speed_ = runtime_alignment_vel_;
        }
        else
        {
          max_linear_speed_ = runtime_alignment_vel_ * 1.5;
        }

        double standard_max_speed = m_work_state_ ? runtime_work_max_vel_ : runtime_walk_max_vel_;
        max_linear_speed_ = std::min(max_linear_speed_, standard_max_speed * 0.5);
      }

      if (runtime_accel_dist_ <= 1e-6)
      {
        target_speed = max_linear_speed_;
      }
      else
      {
        // 如果对齐很早完成但被 0.25m 门槛延后加速，为避免加速曲线在 0.25m 处“跳速”，
        // 将加速曲线的距离零点钉在 max(对齐完成时刻, 0.25m)。
        const double effective_accel_start_distance =
            std::max(accel_start_distance_to_start_, kMinAccelStartDistance);

        double distance_since_accel_start = distance_to_start - effective_accel_start_distance;
        distance_since_accel_start = std::max(0.0, distance_since_accel_start);

        double normalized_distance = distance_since_accel_start / runtime_accel_dist_;
        normalized_distance = std::max(0.0, std::min(1.0, normalized_distance));

        double sigmoid_value =
            1.0 / (1.0 + std::exp(-accelSigmoidK() * (normalized_distance - accelSigmoidCenter())));

        target_speed = runtime_alignment_vel_ + (max_linear_speed_ - runtime_alignment_vel_) * sigmoid_value;
      }
    }

    target_speed = std::min(target_speed, max_linear_speed_);
    target_speed = std::max(target_speed, runtime_accel_min_vel_);
  }

  static std::chrono::steady_clock::time_point prev_time = std::chrono::steady_clock::now();

  auto current_time = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(current_time - prev_time).count();
  prev_time = current_time;

  if (dt > 0.001)
  {
    double max_decel_rate = 0.5;
    double max_speed_change = max_decel_rate * dt;

    if (target_speed < prev_speed - max_speed_change)
    {
      target_speed = prev_speed - max_speed_change;
    }
  }

  if (distance_to_target > runtime_decel_dist_)
  {
    const double smooth_factor = 0.25;
    target_speed = prev_speed + smooth_factor * (target_speed - prev_speed);
  }

  prev_speed = target_speed;
  current_linear_speed_ = target_speed;


  return current_linear_speed_;
}

double LineFollowController::computeAngularVelocity(double yaw_error, double dt, double distance_to_target,
                                                    double distance_to_start, double linear_speed)
{
  (void)distance_to_target;
  (void)distance_to_start;
  (void)linear_speed;

  // 角速度规划核心：
  // - 对齐阶段和跟随阶段使用不同的 PID 参数和角速度/角加速度限制；
  // - 对 PID 输出做 Hampel 滤波 + 一阶低通（alpha）；
  // - 最后再经过一个二阶平滑器，减少急剧转向带来的抖动。
  const bool is_alignment_phase = !start_line_aligned_;
  double current_max_angular_vel =
      is_alignment_phase ? alignment_params_.max_angular_vel : following_params_.max_angular_vel;

  if (heading_pid_controller_)
  {
    heading_pid_controller_->setOutputLimits(-current_max_angular_vel, current_max_angular_vel);
  }

  double raw_angular_velocity = heading_pid_controller_->compute(yaw_error, dt);
  double angular_acceleration = (raw_angular_velocity - prev_angular_velocity_) / dt;

  double current_max_angular_accel = is_alignment_phase ?
      alignment_params_.max_angular_accel : following_params_.max_angular_accel;

  if (std::abs(angular_acceleration) > current_max_angular_accel)
  {
    angular_acceleration = (angular_acceleration > 0) ? current_max_angular_accel : -current_max_angular_accel;
    raw_angular_velocity = prev_angular_velocity_ + angular_acceleration * dt;
  }

  const double alpha = is_alignment_phase ? alignment_params_.filter_alpha : following_params_.filter_alpha;

  double smoothed_angular_vel = angular_vel_hampel_filter_.filter(raw_angular_velocity);

  smoothed_angular_vel = angular_smoother_.filter(smoothed_angular_vel, dt);

  smoothed_angular_vel = alpha * smoothed_angular_vel + (1 - alpha) * prev_smoothed_angular_velocity_;

  prev_smoothed_angular_velocity_ = smoothed_angular_vel;

  prev_angular_velocity_ = smoothed_angular_vel;
  return smoothed_angular_vel;
}

double LineFollowController::calculateRotationVelocity(const double& angle_diff)
{
  double factor = 1.0 / (1.0 + std::exp(-rotationFactor() * std::abs(angle_diff)));

  if (std::abs(angle_diff) < rotationAngleThreshold())
  {
    double cosine_factor = rotationSmoothFactor() *
        (1.0 - std::cos(M_PI * std::abs(angle_diff) / rotationAngleThreshold()));
    factor *= cosine_factor;
  }

  double rot_vel = rotationMaxW() * factor;
  rot_vel = std::max(rot_vel, rotationMinW());

  return (angle_diff > 0.0) ? rot_vel : -rot_vel;
}

bool LineFollowController::shouldGoBackward(double curr_x, double curr_y, double curr_yaw,
                                            double target_x, double target_y, double target_yaw)
{
  // 简单的“是否后退更优”判定：
  // 比较正向和反向两种走法在“起始对齐 + 终点对齐”时需要转的总角度，
  // 谁的总转角更小，就认为谁更优。
  double dx = target_x - curr_x;
  double dy = target_y - curr_y;
  double target_direction = std::atan2(dy, dx);

  double forward_initial_turn = normalizeAngle(target_direction - curr_yaw);
  double forward_final_turn = normalizeAngle(target_yaw - target_direction);
  double forward_total_turn = std::abs(forward_initial_turn) + std::abs(forward_final_turn);

  double reverse_direction = normalizeAngle(target_direction + M_PI);
  double reverse_initial_turn = normalizeAngle(reverse_direction - curr_yaw);
  double reverse_final_turn = normalizeAngle(target_yaw - reverse_direction);
  double reverse_total_turn = std::abs(reverse_initial_turn) + std::abs(reverse_final_turn);

  return (forward_total_turn > reverse_total_turn);
}

bool LineFollowController::isBeyondGoal(double robot_x, double robot_y)
{
  // 判断是否已经“越过”终点：
  // - 沿起点到终点的方向做投影，看投影长度是否超过整条路径；
  // - 或者距离终点已经小于 waypoint 容差，也认为到达/略过了终点。
  double path_dx = original_target_pose_.pose.position.x - original_start_pose_.pose.position.x;
  double path_dy = original_target_pose_.pose.position.y - original_start_pose_.pose.position.y;
  double path_length = std::sqrt(path_dx * path_dx + path_dy * path_dy);

  if (path_length == 0.0)
  {
    double distance_to_goal = std::sqrt(
        std::pow(robot_x - original_target_pose_.pose.position.x, 2) +
        std::pow(robot_y - original_target_pose_.pose.position.y, 2));
    return (distance_to_goal <= waypointTolerance());
  }

  double robot_dx = robot_x - original_start_pose_.pose.position.x;
  double robot_dy = robot_y - original_start_pose_.pose.position.y;

  double dot_product = robot_dx * path_dx + robot_dy * path_dy;
  double proj_length = dot_product / path_length;

  double distance_to_goal = std::sqrt(
      std::pow(robot_x - original_target_pose_.pose.position.x, 2) +
      std::pow(robot_y - original_target_pose_.pose.position.y, 2));

  return (proj_length > path_length) || (distance_to_goal <= waypointTolerance());
}

bool LineFollowController::isAlignedWithTarget(double robot_yaw,
                                               const geometry_msgs::msg::Quaternion& target_orientation,
                                               bool is_backward)
{
  // 判断当前航向是否已经与目标航向对齐：
  // - 前进模式直接比较 yaw 差值；
  // - 后退模式会把目标 yaw 加 π，等价于“倒着面对目标”。
  double target_yaw = tf2::getYaw(target_orientation);
  const double yaw_tolerance = 0.05;

  if (is_backward)
  {
    target_yaw += M_PI;
    target_yaw = std::atan2(std::sin(target_yaw), std::cos(target_yaw));
  }

  double yaw_error = angles::shortest_angular_distance(robot_yaw, target_yaw);

  bool has_crossed = (last_yaw_error_ > 0 && yaw_error < 0) || (last_yaw_error_ < 0 && yaw_error > 0);
  last_yaw_error_ = yaw_error;

  return (std::abs(yaw_error) < yaw_tolerance) || has_crossed;
}


geometry_msgs::msg::PoseStamped LineFollowController::getNextWaypoint(double robot_x, double robot_y)
{
  if (current_waypoint_index_ >= global_plan_.poses.size() - 1)
  {
    return target_pose_;
  }

  for (size_t i = current_waypoint_index_; i < global_plan_.poses.size(); ++i)
  {
    double dx = global_plan_.poses[i].pose.position.x - robot_x;
    double dy = global_plan_.poses[i].pose.position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance >= lookaheadDist())
    {
      current_waypoint_index_ = i;
      return global_plan_.poses[i];
    }
  }

  current_waypoint_index_ = global_plan_.poses.size() - 1;
  return global_plan_.poses.back();
}

size_t LineFollowController::findNearestSegment(double robot_x, double robot_y)
{
  size_t nearest_index = 0;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i)
  {
    const auto& pose1 = global_plan_.poses[i].pose.position;
    const auto& pose2 = global_plan_.poses[i + 1].pose.position;

    double distance = distanceToSegment(robot_x, robot_y, pose1.x, pose1.y, pose2.x, pose2.y);

    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_index = i;
    }
  }

  return nearest_index;
}

double LineFollowController::computeCrossTrackError(double robot_x, double robot_y)
{
  if (global_plan_.poses.size() < 2)
  {
    return 0.0;
  }

  size_t nearest_segment_index = findNearestSegment(robot_x, robot_y);

  const auto& pose1 = global_plan_.poses[nearest_segment_index].pose.position;
  const auto& pose2 = global_plan_.poses[nearest_segment_index + 1].pose.position;

  double dx = pose2.x - pose1.x;
  double dy = pose2.y - pose1.y;
  double length = std::sqrt(dx * dx + dy * dy);

  if (length == 0.0)
  {
    return 0.0;
  }

  double nx = -dy / length;
  double ny = dx / length;

  double rx = robot_x - pose1.x;
  double ry = robot_y - pose1.y;

  return rx * nx + ry * ny;
}

void LineFollowController::extendPath()
{
  if (original_path_length_ < 0.01)
  {
    return;
  }

  double dx = target_pose_.pose.position.x - original_start_pose_.pose.position.x;
  double dy = target_pose_.pose.position.y - original_start_pose_.pose.position.y;

  double norm = std::sqrt(dx * dx + dy * dy);
  double unit_dx = dx / norm;
  double unit_dy = dy / norm;

  int num_points = static_cast<int>(PATH_EXTENSION_LENGTH / PATH_POINT_INTERVAL);

  for (int i = 1; i <= num_points; ++i)
  {
    geometry_msgs::msg::PoseStamped new_pose;
    new_pose.header = target_pose_.header;

    double extension_distance = i * PATH_POINT_INTERVAL;
    new_pose.pose.position.x = target_pose_.pose.position.x + unit_dx * extension_distance;
    new_pose.pose.position.y = target_pose_.pose.position.y + unit_dy * extension_distance;
    new_pose.pose.position.z = target_pose_.pose.position.z;
    new_pose.pose.orientation = target_pose_.pose.orientation;

    global_plan_.poses.push_back(new_pose);
  }

  target_pose_ = global_plan_.poses.back();
  end_pose_ = target_pose_;

  LOG_INFO("路径已延长 %.2fm,添加了 %d 个路径点", PATH_EXTENSION_LENGTH, num_points);
}


bool LineFollowController::handleStateAlignment(double robot_yaw,
                                                const geometry_msgs::msg::Quaternion& target_orientation,
                                                geometry_msgs::msg::TwistStamped& cmd_vel, bool is_backward)
{
  double target_yaw = tf2::getYaw(target_orientation);

  if (is_backward)
  {
    target_yaw += M_PI;
    target_yaw = std::atan2(std::sin(target_yaw), std::cos(target_yaw));
  }

  double yaw_error = angles::shortest_angular_distance(robot_yaw, target_yaw);

  if (isAlignedWithTarget(robot_yaw, target_orientation, is_backward))
  {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    last_yaw_error_ = 0.0;

    waiting_ = true;
    wait_start_time_ = std::chrono::steady_clock::now();

    return true;
  }

  double angular_output = calculateRotationVelocity(yaw_error);

  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = angular_output;

  return false;
}

void LineFollowController::handlePathFollowing(double robot_x, double robot_y,
                                               geometry_msgs::msg::TwistStamped& cmd_vel)
{
  double original_target_dx = original_target_pose_.pose.position.x - robot_x;
  double original_target_dy = original_target_pose_.pose.position.y - robot_y;
  double distance_to_original_target = std::sqrt(original_target_dx * original_target_dx + original_target_dy * original_target_dy);

  double original_start_dx = original_start_pose_.pose.position.x - robot_x;
  double original_start_dy = original_start_pose_.pose.position.y - robot_y;
  double distance_to_original_start = std::sqrt(original_start_dx * original_start_dx + original_start_dy * original_start_dy);

  // double start_dx = start_pose_.pose.position.x - robot_x;
  // double start_dy = start_pose_.pose.position.y - robot_y;
  // double distance_to_start = std::sqrt(start_dx * start_dx + start_dy * start_dy);

  geometry_msgs::msg::PoseStamped target_waypoint = getNextWaypoint(robot_x, robot_y);

  double path_dx, path_dy;
  if (back_follow_)
  {
    path_dx = robot_x - target_waypoint.pose.position.x;
    path_dy = robot_y - target_waypoint.pose.position.y;
  }
  else
  {
    path_dx = target_waypoint.pose.position.x - robot_x;
    path_dy = target_waypoint.pose.position.y - robot_y;
  }

  double path_direction_yaw = std::atan2(path_dy, path_dx);
  double final_target_yaw = path_direction_yaw;

  double path_ideal_yaw = tf2::getYaw(original_start_pose_.pose.orientation);
  if (back_follow_)
  {
    path_ideal_yaw += M_PI;
    path_ideal_yaw = std::atan2(std::sin(path_ideal_yaw), std::cos(path_ideal_yaw));
  }

  updateStartLineAlignment(robot_x, robot_y, robot_yaw_, distance_to_original_start);

  double linear_speed = computeLinearSpeed(distance_to_original_target, distance_to_original_start);

  const size_t nearest_segment_index = findNearestSegment(robot_x, robot_y);
  const auto& seg_p1 = global_plan_.poses[nearest_segment_index].pose.position;
  const auto& seg_p2 = global_plan_.poses[nearest_segment_index + 1].pose.position;

  double seg_dx = seg_p2.x - seg_p1.x;
  double seg_dy = seg_p2.y - seg_p1.y;
  if (back_follow_)
  {
    seg_dx = -seg_dx;
    seg_dy = -seg_dy;
  }
  const double seg_len = std::hypot(seg_dx, seg_dy);
  const double seg_unit_x = (seg_len > 1e-9) ? (seg_dx / seg_len) : 1.0;
  const double seg_unit_y = (seg_len > 1e-9) ? (seg_dy / seg_len) : 0.0;
  const double path_yaw = std::atan2(seg_unit_y, seg_unit_x);

  // 使用“与运动方向一致的法向”计算横向误差（back_follow_ 时符号也会同步翻转）
  const double seg_nx = -seg_unit_y;
  const double seg_ny = seg_unit_x;
  const double rx = robot_x - seg_p1.x;
  const double ry = robot_y - seg_p1.y;
  const double cross_track_error = rx * seg_nx + ry * seg_ny;

  const double theta_max = std::max(0.0, params_.guidance.theta_max);
  const std::string& guidance_mode = params_.guidance.mode;
  std::string effective_mode = guidance_mode;
  if (guidance_mode == "hybrid")
  {
    effective_mode = start_line_aligned_ ? params_.guidance.following_mode : params_.guidance.alignment_mode;
  }

  if (effective_mode == "stanley")
  {
    const double k = std::max(0.0, params_.guidance.stanley_k);
    const double v0 = std::max(0.0, params_.guidance.stanley_v0);
    const double v = std::abs(linear_speed);

    // 注意：这里的 cross_track_error 使用“左法向”（seg_n = (-uy, ux)）定义：
    // - cross_track_error > 0 表示机器人在路径左侧；
    // - 为了朝路径收敛，截获角应与横向误差符号相反（否则会越修越偏）。
    double intercept = std::atan2(-k * cross_track_error, v + v0);
    if (theta_max > 1e-6)
    {
      intercept = std::clamp(intercept, -theta_max, theta_max);
    }
    final_target_yaw = normalizeAngle(path_yaw + intercept);
  }
  else
  {
    // 兼容原行为：path_ideal_yaw 与 path_direction_yaw 加权
    double current_heading_weight = 0.0;
    double target_heading_weight = 0.0;

    if (start_line_aligned_)
    {
      current_heading_weight = following_params_.current_heading_weight;
      target_heading_weight = following_params_.target_heading_weight;
    }
    else
    {
      current_heading_weight = alignment_params_.current_heading_weight;
      target_heading_weight = alignment_params_.target_heading_weight;
    }

    final_target_yaw = path_ideal_yaw * current_heading_weight + path_direction_yaw * target_heading_weight;
  }

  if (robot_x == 0.0 || robot_y == 0.0)
  {
    final_target_yaw = path_ideal_yaw;
  }

  if(distance_to_original_start > 0.14){
    start_print = true;
    stop_print = false;
  }

  if (distance_to_original_target < 0.03)
  {
    start_print = false;
    stop_print = true;
  }

  current_pose_.pose.position.x = robot_x;
  current_pose_.pose.position.y = robot_y;

  double yaw_error = angles::shortest_angular_distance(robot_yaw_, final_target_yaw);
  double dt = getDeltaTime();
  double angular_output =
      computeAngularVelocity(yaw_error, dt, distance_to_original_target, distance_to_original_start, linear_speed);

  current_angular_speed_ = angular_output;

  if (distance_to_original_target < 0.07)
  {
    angular_output = 0.0;
  }

  cmd_vel.twist.linear.x = linear_speed;
  cmd_vel.twist.angular.z = angular_output;

  if (back_follow_)
  {
    cmd_vel.twist.linear.x = -cmd_vel.twist.linear.x;
  }

  // debug 用：与制导一致的横向误差
  // （此处不再调用 computeCrossTrackError，避免 back_follow_ 时符号不一致）

  if (isDebugEnabled())
  {
    LOG_INFO("路径跟随 - 航向误差: %.4f, 横向误差: %.4f, 速度: [%.3f, %.3f], 后退: %d",
             yaw_error, cross_track_error, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, back_follow_);
    LOG_INFO(" ");
  }
}

bool LineFollowController::handleWaitingState(geometry_msgs::msg::TwistStamped& cmd_vel)
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

// ============================================================================
// 主控制循环
// ============================================================================

bool LineFollowController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                   const geometry_msgs::msg::Twist& velocity,
                                                   geometry_msgs::msg::TwistStamped& cmd_vel)
{
  (void)velocity;

  // 主循环入口：
  // - 根据当前状态机分支到起始对齐/路径跟随/终点对齐；
  // - 在路径跟随阶段调用 computeLinearSpeed + computeAngularVelocity 计算 cmd_vel；
  // - 在终点对齐阶段主要做调试数据导出。
  if (!received_plan_)
  {
    LOG_WARN("尚未接收到路径");
    return false;
  }

  if (goal_reached_)
  {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return true;
  }

  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0)
  {
    initializeFilters();
  }

  if (waiting_)
  {
    if (handleWaitingState(cmd_vel))
    {
      return true;
    }
  }

  robot_yaw_ = tf2::getYaw(pose.pose.orientation);
  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;

  double filtered_x_ = h_x_filter_.filter(robot_x);
  double filtered_y_ = h_y_filter_.filter(robot_y);

  double smoothed_robot_x = x_filter_.filter(filtered_x_);
  double smoothed_robot_y = y_filter_.filter(filtered_y_);

  double control_x = smoothed_robot_x;
  double control_y = smoothed_robot_y;

  if (isDebugEnabled())
  {
    original_path_.push_back({ filtered_x_, filtered_y_ });
    filtered_path_.push_back({ smoothed_robot_x, smoothed_robot_y });
  }

  switch (current_state_)
  {
    case ControlState::ALIGNING_START:
    {
      if (handleStateAlignment(robot_yaw_, original_start_pose_.pose.orientation, cmd_vel, back_follow_))
      {
        current_state_ = ControlState::FOLLOWING_PATH;
      }
      break;
    }

    case ControlState::FOLLOWING_PATH:
    {
      // start_print = true;
      // stop_print = false;

      if (isBeyondGoal(control_x, control_y))
      {
        current_state_ = ControlState::ALIGNING_END;
        current_linear_speed_ = 0.0;
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        waiting_ = true;
        wait_start_time_ = std::chrono::steady_clock::now();
        return true;
      }

      handlePathFollowing(control_x, control_y, cmd_vel);
      break;
    }

    case ControlState::ALIGNING_END:
    {
      start_print = false;
      stop_print = true;

      current_state_ = ControlState::GOAL_REACHED;
      goal_reached_ = true;

      std::string timestamp = std::to_string(
          std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count());

      if (!resolved_debug_raw_dir_.empty())
      {
        exportDebugData(resolved_debug_raw_dir_ + "/_" + timestamp + "_line.csv", original_path_);
      }
      if (!resolved_debug_filtered_dir_.empty())
      {
        exportDebugData(resolved_debug_filtered_dir_ + "/_" + timestamp + "_line.csv", filtered_path_);
      }
      break;
    }

    case ControlState::GOAL_REACHED:
    {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      break;
    }

    default:
    {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      break;
    }
  }

  return true;
}


double LineFollowController::getDeltaTime()
{
  auto current_time = std::chrono::steady_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
      current_time - last_time_).count();
  last_time_ = current_time;

  if (dt <= 0.0 || std::isnan(dt) || std::isinf(dt) || dt > 1.0)
  {
    dt = 0.025;
  }

  return dt;
}

double LineFollowController::normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

double LineFollowController::distanceToSegment(double x, double y,
                                               double x1, double y1,
                                               double x2, double y2)
{
  double A = x - x1;
  double B = y - y1;
  double C = x2 - x1;
  double D = y2 - y1;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = (len_sq != 0.0) ? (dot / len_sq) : -1.0;

  double xx, yy;
  if (param < 0.0)
  {
    xx = x1;
    yy = y1;
  }
  else if (param > 1.0)
  {
    xx = x2;
    yy = y2;
  }
  else
  {
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

  double dx = x - xx;
  double dy = y - yy;
  return std::sqrt(dx * dx + dy * dy);
}

void LineFollowController::exportDebugData(const std::string& file_path,
                                           const std::vector<std::vector<double>>& data)
{
  std::lock_guard<std::mutex> lock(file_mutex_);

  std::filesystem::path file_path_obj(file_path);
  std::filesystem::path directory = file_path_obj.parent_path();

  if (!directory.empty() && !std::filesystem::exists(directory))
  {
    try
    {
      std::filesystem::create_directories(directory);
      LOG_INFO("创建目录: %s", directory.string().c_str());
    }
    catch (const std::exception& e)
    {
      LOG_ERROR("无法创建目录 %s: %s", directory.string().c_str(), e.what());
      return;
    }
  }

  std::ofstream file(file_path);
  if (!file.is_open())
  {
    LOG_ERROR("无法打开文件进行数据导出: %s", file_path.c_str());
    return;
  }

  file << std::fixed << std::setprecision(10);
  file << "x,y\n";

  for (const auto& row : data)
  {
    for (size_t i = 0; i < row.size(); ++i)
    {
      file << row[i];
      if (i < row.size() - 1)
        file << ",";
    }
    file << "\n";
  }

  file.close();
}

}  // namespace follow_controller
}  // namespace xline
