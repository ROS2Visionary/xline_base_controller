#include "xline_follow_controller/rpp_follow/rpp_follow_controller.hpp"
#include "xline_follow_controller/common/path_utils.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <cstdlib>

namespace xline
{
namespace follow_controller
{

RPPController::RPPController()
  : BaseFollowController("rpp_follow_controller")
  // 策略模式
  , path_strategy_(nullptr)
  , current_strategy_type_(PathStrategyType::CURVE)
  // 状态标志
  , initialized_(false)
  , goal_reached_(false)
  , waiting_(false)
  , back_follow_(false)
  // 航向预对准
  , need_yaw_prealign_(false)
  , yaw_prealign_done_(false)
  , target_yaw_(0.0)
  // 误差统计
  , max_error_(0.0)
  , avg_error_(0.0)
  , error_sum_(0.0)
  , error_count_(0)
  , current_lateral_error_(0.0)
  , current_curvature_(0.0)
  // 路径信息
  , path_length_(0.0)
  , traversed_distance_(0.0)
  , remaining_distance_(0.0)
  , last_closest_idx(0)
  // 栅格图参数
  , grid_resolution_(0.01)
  , grid_width_(10.0)
  , grid_height_(10.0)
  // 角速度滤波
  , previous_angular_vel_(0.0)
  , predicted_angular_vel_(0.0)
  , angle_to_path_prev_(0.0)
  , lookahead_dist_prev_(0.0)
  // 二阶平滑器
  , second_order_filter_(2.0, 0.7)
{
  RCLCPP_INFO(get_logger(), "RPPController 实例已创建（结构体参数版），正在初始化...");

  // 基于环境变量初始化默认的栅格图路径
  const char* ws_root = std::getenv("XLINE_WS_ROOT");
  if (ws_root && *ws_root)
  {
    params_.debug.grid_map_path = ws_root;
  }

  // 创建默认策略（曲线路径）
  path_strategy_ = createPathStrategy(PathStrategyType::CURVE);
  path_strategy_->setLogger(get_logger());

  // 先加载参数，再根据参数做一次初始化
  updateParameters("/config/rpp_curve.yaml");
  initialize();
}

RPPController::~RPPController()
{
  RCLCPP_INFO(get_logger(), "RPPController 实例已销毁");
}

void RPPController::initialize()
{
  if (initialized_)
  {
    return;
  }

  // 计算时间步长 - 18Hz 更新频率
  d_t_ = 1.0 / 18.0;

  // 创建栅格图保存目录
  if (params_.debug.enable_grid_map)
  {
    std::filesystem::create_directories(params_.debug.grid_map_path);
    RCLCPP_INFO(get_logger(), "栅格图将保存到: %s", params_.debug.grid_map_path.c_str());
  }

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "控制器初始化完成，控制周期: %.4fs", d_t_);
}

// ============================================================================
// 第二部分：策略管理
// ============================================================================

void RPPController::switchStrategy(PathStrategyType type)
{
  if (current_strategy_type_ == type && path_strategy_ != nullptr)
  {
    return;
  }

  path_strategy_ = createPathStrategy(type);
  path_strategy_->setLogger(get_logger());
  current_strategy_type_ = type;

  RCLCPP_INFO(get_logger(), "切换到 %s 路径策略",
              type == PathStrategyType::CIRCLE ? "圆形" : "曲线");
}

std::string RPPController::getCurrentStrategyType() const
{
  if (path_strategy_)
  {
    return path_strategy_->getTypeName();
  }
  return "none";
}

// ============================================================================
// 第三部分：参数管理
// ============================================================================

void RPPController::loadParamsFromYaml(const xline::YamlParser::YamlParser& parser)
{
  // =========================================================================
  // [1] 目标到达判定
  // =========================================================================
  params_.goal.dist_tol = parser.getParameter<double>("goal.dist_tol");
  params_.goal.rotate_tol = parser.getParameter<double>("goal.rotate_tol");

  // =========================================================================
  // [2] Pure Pursuit 前瞻参数
  // =========================================================================
  params_.lookahead.time = parser.getParameter<double>("lookahead.time");
  params_.lookahead.min_dist = parser.getParameter<double>("lookahead.min_dist");
  params_.lookahead.max_dist = parser.getParameter<double>("lookahead.max_dist");

  // =========================================================================
  // [3] 速度控制 - 线速度
  // =========================================================================
  params_.velocity.linear.base = parser.getParameter<double>("velocity.linear.base");
  params_.velocity.linear.min = parser.getParameter<double>("velocity.linear.min");
  params_.velocity.linear.max = parser.getParameter<double>("velocity.linear.max");
  params_.velocity.linear.max_acceleration = parser.getParameter<double>("velocity.linear.max_acceleration");

  // =========================================================================
  // [4] 速度控制 - 角速度
  // =========================================================================
  params_.velocity.angular.min = parser.getParameter<double>("velocity.angular.min");
  params_.velocity.angular.max = parser.getParameter<double>("velocity.angular.max");
  params_.velocity.angular.max_acceleration = parser.getParameter<double>("velocity.angular.max_acceleration");

  // =========================================================================
  // [5] 路径跟踪约束
  // =========================================================================
  params_.tracking.regulated_min_radius = parser.getParameter<double>("tracking.regulated_min_radius");
  params_.tracking.approach.dist = parser.getParameter<double>("tracking.approach.dist");
  params_.tracking.approach.min_v = parser.getParameter<double>("tracking.approach.min_v");

  // =========================================================================
  // [6] 角速度平滑 - 基础参数
  // =========================================================================
  params_.smoothing.type = parser.getParameter<std::string>("smoothing.type");
  params_.smoothing.lowpass.gain = parser.getParameter<double>("smoothing.lowpass.gain");
  params_.smoothing.moving_average.window_size = parser.getParameter<int>("smoothing.moving_average.window_size");

  // =========================================================================
  // [7] 角速度平滑 - 二阶滤波器
  // =========================================================================
  params_.smoothing.second_order.freq = parser.getParameter<double>("smoothing.second_order.freq");
  params_.smoothing.second_order.damping = parser.getParameter<double>("smoothing.second_order.damping");

  // =========================================================================
  // [8] 滤波器配置 - 位置
  // =========================================================================
  params_.filter.position.hampel_window = parser.getParameter<int>("filter.position.hampel_window");
  params_.filter.position.hampel_k = parser.getParameter<double>("filter.position.hampel_k");
  params_.filter.position.savgol_window = parser.getParameter<int>("filter.position.savgol_window");
  params_.filter.position.savgol_order = parser.getParameter<int>("filter.position.savgol_order");

  // =========================================================================
  // [9] 滤波器配置 - 角速度
  // =========================================================================
  params_.filter.angular.lowpass_use_biquad_cascade = parser.getParameter<bool>("filter.angular.lowpass_use_biquad_cascade");
  params_.filter.angular.lowpass_active = parser.getParameter<bool>("filter.angular.lowpass_active");
  params_.filter.angular.use_offset_limit = parser.getParameter<bool>("filter.angular.lowpass_use_offset_limit");
  params_.filter.angular.output_offset = parser.getParameter<double>("filter.angular.lowpass_output_offset");
  params_.filter.angular.cutoff_freq = parser.getParameter<double>("filter.angular.cutoff_freq");
  params_.filter.angular.sample_rate = parser.getParameter<double>("filter.angular.sample_rate");
  params_.filter.angular.output_limit_rate = parser.getParameter<double>("filter.angular.output_limit_rate");
  params_.filter.angular.rate_limit = parser.getParameter<double>("filter.angular.rate_limit");


  // =========================================================================
  // [11] 调试与可视化
  // =========================================================================
  params_.debug.enable_grid_map = parser.getParameter<bool>("debug.enable_grid_map");
  if (parser.hasParameter("debug.grid_map_path"))
  {
    params_.debug.grid_map_path = xline::path_utils::resolve_path(
        parser.getParameter<std::string>("debug.grid_map_path"));
  }

  // =========================================================================
  // [12] 偏差控制
  // =========================================================================
  params_.deviation.start_factor = parser.getParameter<double>("deviation.start_factor");
  params_.deviation.end_factor = parser.getParameter<double>("deviation.end_factor");
  // 兼容旧字段 deviation.rate（历史上用于放宽区间的偏差比率）
  if (parser.hasParameter("deviation.relaxed_ratio"))
  {
    params_.deviation.relaxed_ratio = parser.getParameter<double>("deviation.relaxed_ratio");
  }
  else if (parser.hasParameter("deviation.rate"))
  {
    params_.deviation.relaxed_ratio = parser.getParameter<double>("deviation.rate");
  }

  // 中间阶段严格约束比例（可选，未配置则沿用默认值）
  if (parser.hasParameter("deviation.strict_ratio"))
  {
    params_.deviation.strict_ratio = parser.getParameter<double>("deviation.strict_ratio");
  }
}

void RPPController::updateParameters(std::string file_path)
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("xline_follow_controller");
  std::string config_file_path = package_share_directory + file_path;
  xline::YamlParser::YamlParser parser(config_file_path);

  // 从YAML加载参数到结构体
  loadParamsFromYaml(parser);

  // 更新二阶平滑器参数
  second_order_filter_.setParameters(params_.smoothing.second_order.freq, 
                                      params_.smoothing.second_order.damping);

  // 更新策略参数
  if (path_strategy_)
  {
    path_strategy_->updateParameters(file_path);
  }

  RCLCPP_INFO(get_logger(),
              "参数已更新: 前瞻距离[%.2f-%.2f]m, 速度[%.2f-%.2f]m/s, 角速度[%.2f-%.2f]rad/s",
              params_.lookahead.min_dist, params_.lookahead.max_dist, 
              params_.velocity.linear.min, params_.velocity.linear.max, 
              params_.velocity.angular.min, params_.velocity.angular.max);
}

// ============================================================================
// 第四部分：路径设置
// ============================================================================

void RPPController::setAngleRange(double start_angle, double end_angle)
{
  // 切换到圆形路径策略
  switchStrategy(PathStrategyType::CIRCLE);

  // 设置角度范围
  auto* circle_strategy = dynamic_cast<CirclePathStrategy*>(path_strategy_.get());
  if (circle_strategy)
  {
    circle_strategy->setAngleRange(start_angle, end_angle);
  }
}

bool RPPController::setPlanForCircle(double circle_center_x, double circle_center_y,
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

  // 切换到圆形路径策略
  switchStrategy(PathStrategyType::CIRCLE);
  updateParameters("/config/rpp_circle.yaml");
  back_follow_ = false;  // 圆形路径不使用后退模式

  auto* circle_strategy = dynamic_cast<CirclePathStrategy*>(path_strategy_.get());
  if (!circle_strategy)
  {
    RCLCPP_ERROR(get_logger(), "策略类型错误");
    return false;
  }

  // 设置圆形参数
  double actual_radius = circle_radius;
  circle_strategy->setCircleCenter(circle_center_x, circle_center_y);
  circle_strategy->setCircleRadius(actual_radius);
  circle_strategy->initializeDynamicParams();

  // 重置控制器状态
  resetControllerState();

  try
  {
    // 生成圆形路径
    nav_msgs::msg::Path circle_path = circle_strategy->generateCirclePath(
        circle_center_x, circle_center_y, actual_radius, robot_pose);

    RCLCPP_INFO(get_logger(), "圆形路径已生成 - 圆心: (%.3f, %.3f), 半径: %.3f m, 点数: %zu",
                circle_center_x, circle_center_y, actual_radius, circle_path.poses.size());

    // 设置路径
    return setPlan(circle_path);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "设置圆形路径时发生异常: %s", e.what());
    return false;
  }
}

bool RPPController::setPlanForSpline(const std::vector<std::pair<double, double>>& vertices,
                                      int degree,
                                      double start_x, double start_y,
                                      double end_x, double end_y)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化，请先调用 initialize()");
    return false;
  }

  // 切换到曲线路径策略
  switchStrategy(PathStrategyType::CURVE);
  updateParameters("/config/rpp_curve.yaml");

  auto* curve_strategy = dynamic_cast<CurvePathStrategy*>(path_strategy_.get());
  if (!curve_strategy)
  {
    RCLCPP_ERROR(get_logger(), "策略类型错误，无法转换为 CurvePathStrategy");
    return false;
  }

  // 重置控制器状态
  resetControllerState();

  // 调用 CurvePathStrategy 的 setSplinePath 方法生成路径
  if (!curve_strategy->setSplinePath(vertices, degree, start_x, start_y, end_x, end_y))
  {
    RCLCPP_ERROR(get_logger(), "设置 Spline 路径失败");
    return false;
  }

  // 计算路径信息（从 curve_strategy 复制）
  // 注意：curve_strategy 内部已经生成了 global_plan_，我们需要获取它
  // 由于 global_plan_ 是 private，我们通过其他方式同步
  calculatePathInfo();

  RCLCPP_INFO(get_logger(), "Spline 路径设置完成");
  return true;
}

bool RPPController::setPlanForEllipse(double center_x, double center_y,
                                       double major_axis_x, double major_axis_y,
                                       double ratio, double rotation,
                                       double start_angle, double end_angle)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化，请先调用 initialize()");
    return false;
  }

  // 切换到曲线路径策略
  switchStrategy(PathStrategyType::CURVE);
  updateParameters("/config/rpp_curve.yaml");

  auto* curve_strategy = dynamic_cast<CurvePathStrategy*>(path_strategy_.get());
  if (!curve_strategy)
  {
    RCLCPP_ERROR(get_logger(), "策略类型错误，无法转换为 CurvePathStrategy");
    return false;
  }

  // 重置控制器状态
  resetControllerState();

  // 调用 CurvePathStrategy 的 setEllipsePath 方法生成路径
  if (!curve_strategy->setEllipsePath(center_x, center_y, major_axis_x, major_axis_y,
                                       ratio, rotation, start_angle, end_angle))
  {
    RCLCPP_ERROR(get_logger(), "设置 Ellipse 路径失败");
    return false;
  }

  // 计算路径信息
  calculatePathInfo();

  RCLCPP_INFO(get_logger(), "Ellipse 路径设置完成");
  return true;
}

bool RPPController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化，请先调用 initialize()");
    return false;
  }

  if (orig_global_plan.poses.empty())
  {
    RCLCPP_ERROR(get_logger(), "收到空路径，无法设置计划");
    return false;
  }

  RCLCPP_INFO(get_logger(), "设置新计划，包含 %zu 个点", orig_global_plan.poses.size());

  // 如果当前不是圆形策略，切换到曲线策略
  if (current_strategy_type_ != PathStrategyType::CIRCLE)
  {
    switchStrategy(PathStrategyType::CURVE);
  }

  // 重置控制器状态
  resetControllerState();

  // 处理和优化路径
  if (orig_global_plan.poses.size() > 2)
  {
    global_plan_ = smoothAndSubdividePath(orig_global_plan);
    RCLCPP_INFO(get_logger(), "路径优化完成: 原始点数=%zu, 优化后点数=%zu",
                orig_global_plan.poses.size(), global_plan_.poses.size());
  }
  else
  {
    global_plan_ = orig_global_plan;
  }

  // 验证并清理路径中的无效点
  if (!validateAndCleanPath())
  {
    return false;
  }

  // 计算路径信息
  calculatePathInfo();

  // 设置策略的路径
  if (path_strategy_)
  {
    path_strategy_->setPlan(global_plan_);
  }

  // 初始化栅格图
  if (params_.debug.enable_grid_map)
  {
    initializeGridMap(global_plan_);
  }

  return true;
}

void RPPController::setBackFollow(bool back)
{
  back_follow_ = back;
  need_yaw_prealign_ = true;
  yaw_prealign_done_ = false;

  // 同步到曲线策略
  if (current_strategy_type_ == PathStrategyType::CURVE)
  {
    auto* curve_strategy = dynamic_cast<CurvePathStrategy*>(path_strategy_.get());
    if (curve_strategy)
    {
      curve_strategy->setBackFollow(back);
    }
  }

  RCLCPP_INFO(get_logger(), "设置后退模式: %s", back ? "启用" : "禁用");
}

bool RPPController::isGoalReached()
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化");
    return false;
  }

  if (goal_reached_)
  {
    static bool reported = false;
    if (!reported)
    {
      RCLCPP_INFO(get_logger(), "目标已到达 - 最大误差: %.4fm, 平均误差: %.4fm",
                  max_error_, avg_error_);
      reported = true;
    }
    return true;
  }

  return false;
}
// ============================================================================
// 第五部分：核心速度计算
// ============================================================================

bool RPPController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose,
                                            const geometry_msgs::msg::Twist& current_velocity,
                                            geometry_msgs::msg::TwistStamped& cmd_vel)
{
  if (!initialized_ || global_plan_.poses.empty())
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化或路径为空");
    return false;
  }

  // 初始化输出命令
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";

  // 检查是否已到达目标
  if (goal_reached_)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "目标已达到，保持停止状态");
    return true;
  }

  // 处理等待状态
  static int waiting_count = 0;
  if (waiting_)
  {
    if (++waiting_count > 10)
    {
      waiting_count = 0;
      waiting_ = false;
    }
    return true;
  }

  // 位置滤波处理
  geometry_msgs::msg::PoseStamped current_pose = filterRobotPose(robot_pose);

  static bool filter_reset = false;
  try
  {
    auto start_time = this->now();

    // 位置有效性检查
    if (!isValidPose(current_pose))
    {
      RCLCPP_ERROR(get_logger(), "接收到无效的机器人位置: (%.3f, %.3f)",
                   current_pose.pose.position.x, current_pose.pose.position.y);
      return false;
    }

    // 检查策略是否需要航向预对准
    bool strategy_needs_prealign = path_strategy_ && path_strategy_->needsYawPrealignment();
    if ((strategy_needs_prealign || back_follow_) && need_yaw_prealign_ && !yaw_prealign_done_)
    {
      double prealign_yaw = strategy_needs_prealign ?
          path_strategy_->getTargetYaw() : target_yaw_;

      if (performYawPrealignment(current_pose, prealign_yaw, cmd_vel))
      {
        yaw_prealign_done_ = true;
        need_yaw_prealign_ = false;
        waiting_ = true;

        if (path_strategy_)
        {
          path_strategy_->setYawPrealignmentDone();
        }

        RCLCPP_INFO(get_logger(), "航向预对准完成，开始路径跟随");
      }
      return true;
    }

    // 后退模式下调整航向角
    if (back_follow_)
    {
      current_pose = adjustPoseForBackward(current_pose);
    }

    // 构建策略上下文
    PathStrategyContext ctx;
    ctx.current_pose = current_pose;
    ctx.current_velocity = current_velocity;
    ctx.current_yaw = tf2::getYaw(current_pose.pose.orientation);
    ctx.dt = d_t_;
    ctx.min_v = params_.velocity.linear.min;
    ctx.max_v = params_.velocity.linear.max;

    // 检查是否到达目标（使用策略）
    if (path_strategy_ && path_strategy_->isGoalReached(ctx))
    {
      setGoalReachedState(cmd_vel);

      // 同步打印标志（圆形路径特有）
      if (current_strategy_type_ == PathStrategyType::CIRCLE)
      {
        auto* circle_strategy = dynamic_cast<CirclePathStrategy*>(path_strategy_.get());
        if (circle_strategy)
        {
          start_print = circle_strategy->shouldStartPrint();
          stop_print = circle_strategy->shouldStopPrint();
        }
      }

      return true;
    }

    // 裁剪全局路径
    std::vector<geometry_msgs::msg::PoseStamped> pruned_plan;
    pruneGlobalPlan(current_pose, global_plan_, pruned_plan);

    if (pruned_plan.empty())
    {
      RCLCPP_ERROR(get_logger(), "裁剪后的路径为空");
      return false;
    }

    // 获取前瞻点
    // 这里的 current_velocity 通常来自上一次输出的 cmd_vel（或底盘回传的等效速度）。
    // 启动/切换策略后的第一拍可能是 0，此时如果直接用 0 参与前瞻距离计算，
    // 在某些参数组合下会导致前瞻距离过小甚至为 0（进而让曲率/误差计算退化）。
    // 因此：当线速度为 0（或无效）时，使用最小线速度作为前瞻距离计算输入。
    double speed_for_lookahead = current_velocity.linear.x;
    if (!std::isfinite(speed_for_lookahead))
    {
      speed_for_lookahead = 0.0;
    }
    if (std::abs(speed_for_lookahead) < 1e-6)
    {
      // 保底：若最小速度配置异常(<=0)，用一个很小的正值避免前瞻距离为 0。
      speed_for_lookahead = std::max(params_.velocity.linear.min, 1e-3);
    }

    double lookahead_distance = getLookAheadDistance(speed_for_lookahead);
    geometry_msgs::msg::PoseStamped lookahead_pose = getLookAheadPoint(lookahead_distance, pruned_plan, true);

    if (!isValidPose(lookahead_pose))
    {
      lookahead_pose = getFallbackLookaheadPoint(pruned_plan);
      if (!isValidPose(lookahead_pose))
      {
        return false;
      }
    }

    // 计算控制量
    double angle_to_lookahead = computeAngleToLookahead(lookahead_pose, current_pose);
    current_curvature_ = computeCurvature(angle_to_lookahead, lookahead_distance);
    current_lateral_error_ = computeLateralError(angle_to_lookahead, lookahead_distance);

    // 更新误差统计
    updateErrorStatistics();

    // 计算期望速度
    double desired_velocity = 0.0;
    if (path_strategy_ && path_strategy_->hasDynamicVelocity())
    {
      desired_velocity = path_strategy_->getTargetVelocity();
      if (!std::isfinite(desired_velocity) || desired_velocity <= 0.0)
      {
        desired_velocity = params_.velocity.linear.base;
      }
    }
    else
    {
      desired_velocity = applyCurvatureConstraint(params_.velocity.linear.base, current_curvature_);
    }

    // 更新策略上下文
    if (path_strategy_ && path_strategy_->hasDynamicVelocity())
    {
      ctx.min_v = desired_velocity;
      ctx.max_v = desired_velocity;
    }

    ctx.angle_to_lookahead = angle_to_lookahead;
    ctx.lookahead_distance = lookahead_distance;
    ctx.curvature = current_curvature_;
    ctx.pp_angular_velocity = desired_velocity * current_curvature_;
    ctx.desired_linear_velocity = desired_velocity;
    ctx.remaining_distance = remaining_distance_;
    ctx.traversed_distance = traversed_distance_;

    // 使用策略计算角速度
    PathStrategyResult result;
    if (path_strategy_)
    {
      path_strategy_->computeAngularVelocity(ctx, result);
    }
    else
    {
      // 回退到标准 PP
      result.desired_angular_velocity = ctx.pp_angular_velocity;
      result.desired_linear_velocity = desired_velocity;
      result.filter_reset = false;
    }

    // 应用角速度平滑
    double smoothed_angular_velocity = smoothAngularVelocity(result.desired_angular_velocity,
        lookahead_distance, angle_to_lookahead, d_t_, result.filter_reset);

    // 生成速度命令
    generateVelocityCommand(cmd_vel, current_velocity, result.desired_linear_velocity, smoothed_angular_velocity);

    // 同步打印标志（圆形路径特有）
    if (current_strategy_type_ == PathStrategyType::CIRCLE)
    {
      auto* circle_strategy = dynamic_cast<CirclePathStrategy*>(path_strategy_.get());
      if (circle_strategy)
      {
        start_print = circle_strategy->shouldStartPrint();
        stop_print = circle_strategy->shouldStopPrint();
      }

      RCLCPP_INFO(get_logger(), "基准角速度: %.5f, pp角速度: %.5f, 期望角速度: %.5f, 平滑角速度: %.5f, 前瞻距离: %.3f, 线速度: %.3f \n",
                  circle_strategy ? circle_strategy->getBaselineAngularVelocity()
                                  : std::numeric_limits<double>::quiet_NaN(),
                  ctx.pp_angular_velocity,
                  result.desired_angular_velocity,
                  smoothed_angular_velocity,
                  lookahead_distance,
                  result.desired_linear_velocity);
    }

    // 更新栅格图
    updateGridMapIfNeeded(current_pose, lookahead_pose);



    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "计算速度命令时发生异常: %s", e.what());
    return false;
  }
}

// ============================================================================
// 第六部分：路径处理辅助函数
// ============================================================================

void RPPController::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& current_pose,
                                    const nav_msgs::msg::Path& global_plan,
                                    std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan)
{
  pruned_plan.clear();

  if (global_plan.poses.empty())
  {
    return;
  }

  // 找到最近的路径点
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < global_plan.poses.size(); ++i)
  {
    double dist = std::hypot(global_plan.poses[i].pose.position.x - current_pose.pose.position.x,
                             global_plan.poses[i].pose.position.y - current_pose.pose.position.y);
    if (dist < min_dist)
    {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // 添加最近点及后续所有点
  pruned_plan.insert(pruned_plan.end(), global_plan.poses.begin() + closest_idx, global_plan.poses.end());
}

geometry_msgs::msg::PoseStamped RPPController::getLookAheadPoint(
    const double& lookahead_dist,
    const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
    bool interpolate_after_goal)
{
  geometry_msgs::msg::PoseStamped lookahead_pose;

  if (transformed_plan.empty())
  {
    return lookahead_pose;
  }

  double accumulated_dist = 0.0;

  for (size_t i = 0; i < transformed_plan.size() - 1; ++i)
  {
    double segment_length = std::hypot(
        transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x,
        transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y);

    if (accumulated_dist + segment_length >= lookahead_dist)
    {
      double remaining = lookahead_dist - accumulated_dist;
      double ratio = remaining / segment_length;

      lookahead_pose.header = transformed_plan[i].header;
      lookahead_pose.pose.position.x = transformed_plan[i].pose.position.x +
          ratio * (transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x);
      lookahead_pose.pose.position.y = transformed_plan[i].pose.position.y +
          ratio * (transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y);
      lookahead_pose.pose.position.z = transformed_plan[i].pose.position.z +
          ratio * (transformed_plan[i + 1].pose.position.z - transformed_plan[i].pose.position.z);

      double yaw = std::atan2(
          transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y,
          transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      lookahead_pose.pose.orientation = tf2::toMsg(q);

      return lookahead_pose;
    }

    accumulated_dist += segment_length;
  }

  // 如果没有找到足够远的点，返回最后一个点
  return transformed_plan.back();
}

double RPPController::getLookAheadDistance(double speed)
{
  // 圆形路径：使用预计算的恒定前瞻距离（与实时速度无关）
  if (path_strategy_ && path_strategy_->hasDynamicVelocity())
  {
    const double target_lookahead = path_strategy_->getTargetLookahead();
    if (std::isfinite(target_lookahead) && target_lookahead > 0.0)
    {
      return target_lookahead;
    }
  }

  // 默认逻辑：时间-速度模型（用于曲线路径）
  const double abs_speed = std::abs(speed);
  const double lookahead_dist = abs_speed * params_.lookahead.time;
  return std::clamp(lookahead_dist, params_.lookahead.min_dist, params_.lookahead.max_dist);
}

// ============================================================================
// 第七部分：速度约束
// ============================================================================

double RPPController::applyCurvatureConstraint(const double raw_linear_vel, const double curvature)
{
  if (std::abs(curvature) < 1e-6)
  {
    return raw_linear_vel;
  }

  double radius = 1.0 / std::abs(curvature);
  if (radius < params_.tracking.regulated_min_radius)
  {
    double scale = radius / params_.tracking.regulated_min_radius;
    return std::max(params_.velocity.linear.min, raw_linear_vel * scale);
  }

  return raw_linear_vel;
}

double RPPController::applyApproachConstraint(const double raw_linear_vel,
                                              geometry_msgs::msg::PoseStamped robot_pose_global,
                                              const std::vector<geometry_msgs::msg::PoseStamped>& prune_plan)
{
  if (prune_plan.empty())
  {
    return raw_linear_vel;
  }

  const auto& goal = prune_plan.back();
  double distance_to_end = std::hypot(
      robot_pose_global.pose.position.x - goal.pose.position.x,
      robot_pose_global.pose.position.y - goal.pose.position.y);

  if (distance_to_end < params_.tracking.approach.dist)
  {
    double ratio = distance_to_end / params_.tracking.approach.dist;
    return std::max(params_.tracking.approach.min_v, raw_linear_vel * ratio);
  }

  return raw_linear_vel;
}

// ============================================================================
// 第八部分：辅助计算
// ============================================================================

double RPPController::regularizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double RPPController::dphi(geometry_msgs::msg::PointStamped lookahead_pt,
                           geometry_msgs::msg::PoseStamped robot_pose_global)
{
  double dx = lookahead_pt.point.x - robot_pose_global.pose.position.x;
  double dy = lookahead_pt.point.y - robot_pose_global.pose.position.y;
  double target_angle = std::atan2(dy, dx);
  double robot_angle = tf2::getYaw(robot_pose_global.pose.orientation);
  return regularizeAngle(target_angle - robot_angle);
}

// ============================================================================
// 第九部分：航向预对准
// ============================================================================

bool RPPController::performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose,
                                           double target_yaw,
                                           geometry_msgs::msg::TwistStamped& cmd_vel)
{
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double yaw_error = angles::shortest_angular_distance(current_yaw, target_yaw);
  double yaw_tolerance = 0.05;  // 约3度

  if (std::abs(yaw_error) < yaw_tolerance)
  {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = "base_link";

    RCLCPP_INFO(get_logger(), "航向预对准完成！角度误差: %.3f rad (%.1f°)",
                yaw_error, yaw_error * 180.0 / M_PI);
    return true;
  }

  double angular_vel = calculateRotationVelocity(yaw_error);

  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = angular_vel;
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                       "航向预对准中... 目标: %.3f, 当前: %.3f, 误差: %.3f, 角速度: %.3f",
                       target_yaw, current_yaw, yaw_error, angular_vel);

  return false;
}

double RPPController::calculateRotationVelocity(const double& angle_diff)
{
  double factor = 1.0 / (1.0 + std::exp(-1.0 * std::abs(angle_diff)));

  if (std::abs(angle_diff) < 0.5)
  {
    double cosine_factor = 0.5 * (1.0 - std::cos(M_PI * std::abs(angle_diff) / 0.5));
    factor *= cosine_factor;
  }

  double rot_vel = std::max(1.0 * factor, 0.2);
  return (angle_diff > 0.0) ? rot_vel : -rot_vel;
}

// ============================================================================
// 第十部分：角速度平滑
// ============================================================================

double RPPController::smoothAngularVelocity(double desired_angular_vel,
                                            double lookahead_dist, double angle_to_path,
                                            double dt, bool is_reset)
{
  if (is_reset)
  {
    angular_vel_history_.clear();
    second_order_filter_.reset();
    previous_angular_vel_ = 0.0;
  }

  // 更新历史记录
  angular_vel_history_.push_back(desired_angular_vel);
  if (angular_vel_history_.size() > static_cast<size_t>(params_.smoothing.moving_average.window_size))
  {
    angular_vel_history_.pop_front();
  }

  double smoothed_angular_vel = desired_angular_vel;

  // 应用选定的平滑方法
  if (params_.smoothing.type == "lowpass")
  {
    smoothed_angular_vel = params_.smoothing.lowpass.gain * desired_angular_vel +
                           (1 - params_.smoothing.lowpass.gain) * previous_angular_vel_;
  }
  else if (params_.smoothing.type == "movingAverage")
  {
    smoothed_angular_vel = 0.0;
    for (double vel : angular_vel_history_)
    {
      smoothed_angular_vel += vel;
    }
    smoothed_angular_vel /= angular_vel_history_.size();
  }

  // 二阶平滑器
  smoothed_angular_vel = second_order_filter_.filter(smoothed_angular_vel, dt);

  // 四阶低通滤波
  if (params_.filter.angular.lowpass_active)
  {
    smoothed_angular_vel = angle_vel_lowpass_filter_.filter(smoothed_angular_vel);
  }

  // 更新历史数据
  previous_angular_vel_ = smoothed_angular_vel;
  angle_to_path_prev_ = angle_to_path;
  lookahead_dist_prev_ = lookahead_dist;

  return smoothed_angular_vel;
}

// ============================================================================
// 第十一部分：私有辅助方法
// ============================================================================

void RPPController::resetControllerState()
{
  goal_reached_ = false;
  waiting_ = false;

  // 重置误差统计
  max_error_ = 0.0;
  avg_error_ = 0.0;
  error_sum_ = 0.0;
  error_count_ = 0;
  current_lateral_error_ = 0.0;

  // 重置滤波器
  second_order_filter_.reset();
  previous_angular_vel_ = 0.0;
  angular_vel_history_.clear();

  sg_x_filter_.reset(params_.filter.position.savgol_window, params_.filter.position.savgol_order);
  sg_y_filter_.reset(params_.filter.position.savgol_window, params_.filter.position.savgol_order);
  h_x_filter.reset(params_.filter.position.hampel_window, params_.filter.position.hampel_k);
  h_y_filter.reset(params_.filter.position.hampel_window, params_.filter.position.hampel_k);

  // 获取基准角速度（如果是圆形路径）
  double baseline_angular_vel = 0.0;
  if (current_strategy_type_ == PathStrategyType::CIRCLE)
  {
    auto* circle_strategy = dynamic_cast<CirclePathStrategy*>(path_strategy_.get());
    if (circle_strategy)
    {
      baseline_angular_vel = circle_strategy->getBaselineAngularVelocity();
    }
  }


  double angle_limit = (baseline_angular_vel > 0) ?
      params_.filter.angular.output_limit_rate * baseline_angular_vel : 
      params_.filter.angular.output_limit_rate;
  angle_vel_lowpass_filter_.reset();
  angle_vel_lowpass_filter_.initialize(params_.filter.angular.cutoff_freq, 
                                params_.filter.angular.sample_rate, 
                                angle_limit);
  angle_vel_lowpass_filter_.setLimits(angle_limit, params_.filter.angular.rate_limit, true);
  angle_vel_lowpass_filter_.setOffsetLimit(params_.filter.angular.output_offset, 
                                    params_.filter.angular.use_offset_limit);
  angle_vel_lowpass_filter_.use_biquad_cascade_ = params_.filter.angular.lowpass_use_biquad_cascade;

  last_closest_idx = 0;

  // 重置策略
  if (path_strategy_)
  {
    path_strategy_->reset();
  }
}

nav_msgs::msg::Path RPPController::smoothAndSubdividePath(const nav_msgs::msg::Path& orig_path)
{
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = orig_path.header;
  smoothed_path.poses.push_back(orig_path.poses.front());

  for (size_t i = 0; i < orig_path.poses.size() - 1; ++i)
  {
    const auto& current = orig_path.poses[i];
    const auto& next = orig_path.poses[i + 1];

    double segment_length = std::hypot(next.pose.position.x - current.pose.position.x,
                                       next.pose.position.y - current.pose.position.y);

    if (segment_length > 0.003)
    {
      int num_subdivisions = std::ceil(segment_length / 0.002);

      for (int j = 1; j < num_subdivisions; ++j)
      {
        double ratio = static_cast<double>(j) / num_subdivisions;

        geometry_msgs::msg::PoseStamped intermediate_pose;
        intermediate_pose.header = current.header;
        intermediate_pose.pose.position.x = current.pose.position.x +
            ratio * (next.pose.position.x - current.pose.position.x);
        intermediate_pose.pose.position.y = current.pose.position.y +
            ratio * (next.pose.position.y - current.pose.position.y);
        intermediate_pose.pose.position.z = current.pose.position.z +
            ratio * (next.pose.position.z - current.pose.position.z);

        double yaw = std::atan2(next.pose.position.y - current.pose.position.y,
                                next.pose.position.x - current.pose.position.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        intermediate_pose.pose.orientation = tf2::toMsg(q);

        smoothed_path.poses.push_back(intermediate_pose);
      }
    }

    if (i < orig_path.poses.size() - 2)
    {
      smoothed_path.poses.push_back(next);
    }
  }

  smoothed_path.poses.push_back(orig_path.poses.back());
  return smoothed_path;
}

bool RPPController::validateAndCleanPath()
{
  std::vector<geometry_msgs::msg::PoseStamped> valid_poses;
  valid_poses.reserve(global_plan_.poses.size());

  for (const auto& pose : global_plan_.poses)
  {
    if (std::isfinite(pose.pose.position.x) &&
        std::isfinite(pose.pose.position.y) &&
        std::isfinite(pose.pose.position.z))
    {
      valid_poses.push_back(pose);
    }
  }

  if (valid_poses.empty())
  {
    RCLCPP_ERROR(get_logger(), "清理后路径为空，无法继续");
    return false;
  }

  if (valid_poses.size() != global_plan_.poses.size())
  {
    RCLCPP_WARN(get_logger(), "路径包含无效点，已清理: %zu -> %zu",
                global_plan_.poses.size(), valid_poses.size());
    global_plan_.poses = valid_poses;
  }

  return true;
}

void RPPController::calculatePathInfo()
{
  if (global_plan_.poses.empty())
  {
    return;
  }

  path_length_ = 0.0;
  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i)
  {
    path_length_ += std::hypot(global_plan_.poses[i + 1].pose.position.x - global_plan_.poses[i].pose.position.x,
                               global_plan_.poses[i + 1].pose.position.y - global_plan_.poses[i].pose.position.y);
  }

  remaining_distance_ = path_length_;
  traversed_distance_ = 0.0;

  const auto& goal_pose = global_plan_.poses.back();
  goal_x_ = goal_pose.pose.position.x;
  goal_y_ = goal_pose.pose.position.y;
  goal_theta_ = tf2::getYaw(goal_pose.pose.orientation);
  current_curvature_ = 0.0;

  RCLCPP_INFO(get_logger(), "路径长度: %.3fm, 目标: (%.3f, %.3f, %.3f)",
              path_length_, goal_x_, goal_y_, goal_theta_);
}


geometry_msgs::msg::PoseStamped RPPController::filterRobotPose(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header = robot_pose.header;
  current_pose.pose.orientation = robot_pose.pose.orientation;

  double filtered_x = h_x_filter.filter(robot_pose.pose.position.x);
  double filtered_y = h_y_filter.filter(robot_pose.pose.position.y);

  filtered_x = sg_x_filter_.filter(filtered_x);
  filtered_y = sg_y_filter_.filter(filtered_y);

  current_pose.pose.position.x = filtered_x;
  current_pose.pose.position.y = filtered_y;

  return current_pose;
}

geometry_msgs::msg::PoseStamped RPPController::adjustPoseForBackward(const geometry_msgs::msg::PoseStamped& pose)
{
  geometry_msgs::msg::PoseStamped adjusted_pose = pose;
  double current_yaw = tf2::getYaw(pose.pose.orientation);
  double modified_yaw = std::atan2(std::sin(current_yaw + M_PI), std::cos(current_yaw + M_PI));

  tf2::Quaternion q;
  q.setRPY(0, 0, modified_yaw);
  adjusted_pose.pose.orientation = tf2::toMsg(q);

  return adjusted_pose;
}

bool RPPController::isValidPose(const geometry_msgs::msg::PoseStamped& pose)
{
  return std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y);
}

bool RPPController::checkGoalReached(const geometry_msgs::msg::PoseStamped& current_pose,
                                     geometry_msgs::msg::TwistStamped& cmd_vel)
{
  auto goal_pose = global_plan_.poses.back();
  double distance_to_goal = std::hypot(current_pose.pose.position.x - goal_pose.pose.position.x,
                                       current_pose.pose.position.y - goal_pose.pose.position.y);

  remaining_distance_ = distance_to_goal;
  traversed_distance_ = path_length_ - remaining_distance_;

  // 使用策略进行判定
  PathStrategyContext ctx;
  ctx.current_pose = current_pose;
  ctx.current_yaw = tf2::getYaw(current_pose.pose.orientation);
  ctx.remaining_distance = remaining_distance_;

  if (path_strategy_ && path_strategy_->isGoalReached(ctx))
  {
    setGoalReachedState(cmd_vel);
    return true;
  }

  return false;
}

void RPPController::setGoalReachedState(geometry_msgs::msg::TwistStamped& cmd_vel)
{
  goal_reached_ = true;
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";

  if (error_count_ > 0)
  {
    avg_error_ = error_sum_ / error_count_;
    RCLCPP_INFO(get_logger(), "跟踪性能统计 - 最大误差: %.4f m, 平均误差: %.4f m",
                max_error_, avg_error_);
  }
}

geometry_msgs::msg::PoseStamped RPPController::getFallbackLookaheadPoint(
    const std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan)
{
  if (!pruned_plan.empty())
  {
    return pruned_plan[std::min(size_t(1), pruned_plan.size() - 1)];
  }
  return geometry_msgs::msg::PoseStamped();
}

double RPPController::computeAngleToLookahead(const geometry_msgs::msg::PoseStamped& lookahead_pose,
                                               const geometry_msgs::msg::PoseStamped& current_pose)
{
  geometry_msgs::msg::PointStamped lookahead_pt;
  lookahead_pt.header = lookahead_pose.header;
  lookahead_pt.point = lookahead_pose.pose.position;

  double angle = dphi(lookahead_pt, current_pose);
  return std::isfinite(angle) ? angle : 0.0;
}

double RPPController::computeCurvature(double angle_to_lookahead, double lookahead_distance)
{
  if (std::abs(lookahead_distance) > 1e-6)
  {
    double curvature = 2.0 * std::sin(angle_to_lookahead) / lookahead_distance;
    return std::isfinite(curvature) ? curvature : 0.0;
  }
  return 0.0;
}

double RPPController::computeLateralError(double angle_to_lookahead, double lookahead_distance)
{
  if (std::abs(lookahead_distance) > 1e-6 && std::isfinite(angle_to_lookahead))
  {
    double error = lookahead_distance * std::sin(angle_to_lookahead);
    return std::isfinite(error) ? error : 0.0;
  }
  return 0.0;
}

void RPPController::updateErrorStatistics()
{
  if (std::isfinite(current_lateral_error_))
  {
    error_sum_ += std::fabs(current_lateral_error_);
    error_count_++;
    if (std::fabs(current_lateral_error_) > max_error_)
    {
      max_error_ = std::fabs(current_lateral_error_);
    }
    avg_error_ = error_sum_ / error_count_;
  }
}

void RPPController::generateVelocityCommand(geometry_msgs::msg::TwistStamped& cmd_vel,
                                             const geometry_msgs::msg::Twist& current_velocity,
                                             double desired_velocity, double desired_angular_velocity)
{
  desired_velocity_ = desired_velocity;

  cmd_vel.twist.angular.z = desired_angular_velocity;

  double cmd_linear = desired_velocity;
  if (!std::isfinite(cmd_linear))
  {
    cmd_linear = 0.0;
  }

  // 曲线路径：按控制器速度边界做一次约束；圆形路径速度已在策略内完成约束
  if (current_strategy_type_ != PathStrategyType::CIRCLE)
  {
    cmd_linear = std::clamp(cmd_linear, params_.velocity.linear.min, params_.velocity.linear.max);
  }

  cmd_vel.twist.linear.x = cmd_linear;

  if (back_follow_ && current_strategy_type_ != PathStrategyType::CIRCLE)
  {
    cmd_vel.twist.linear.x = -cmd_vel.twist.linear.x;
  }

  // 安全检查
  if (!std::isfinite(cmd_vel.twist.linear.x))
  {
    cmd_vel.twist.linear.x = params_.velocity.linear.min;
  }
  if (!std::isfinite(cmd_vel.twist.angular.z))
  {
    cmd_vel.twist.angular.z = 0.0;
  }

  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";
}
// ============================================================================
// 第十二部分：栅格图可视化
// ============================================================================

void RPPController::initializeGridMap(const nav_msgs::msg::Path& path)
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

  double margin = 1.0;
  grid_width_ = (max_x - min_x) + 2 * margin;
  grid_height_ = (max_y - min_y) + 2 * margin;
  grid_origin_x_ = min_x - margin;
  grid_origin_y_ = min_y - margin;

  int width_pixels = static_cast<int>(grid_width_ / grid_resolution_);
  int height_pixels = static_cast<int>(grid_height_ / grid_resolution_);
  grid_map_ = cv::Mat(height_pixels, width_pixels, CV_8UC3, cv::Scalar(255, 255, 255));

  drawGridLines();
  drawPathOnGrid(path, cv::Scalar(0, 0, 255), 1);

  last_grid_update_time_ = this->now();
  saveGridMap();

  RCLCPP_INFO(get_logger(), "初始化栅格图 大小: %.2f x %.2f m, 尺寸: %d x %d 像素",
              grid_width_, grid_height_, width_pixels, height_pixels);
}

cv::Point RPPController::worldToGrid(double x, double y)
{
  int grid_x = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
  int grid_y = grid_map_.rows - static_cast<int>((y - grid_origin_y_) / grid_resolution_) - 1;
  return cv::Point(grid_x, grid_y);
}

void RPPController::drawPathOnGrid(const nav_msgs::msg::Path& path, const cv::Scalar& color, int thickness)
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

  if (!path.poses.empty())
  {
    cv::Point start = worldToGrid(path.poses.front().pose.position.x, path.poses.front().pose.position.y);
    cv::Point end = worldToGrid(path.poses.back().pose.position.x, path.poses.back().pose.position.y);

    if (isPointInGrid(start))
    {
      cv::circle(grid_map_, start, 3, cv::Scalar(0, 255, 0), -1);
    }
    if (isPointInGrid(end))
    {
      cv::circle(grid_map_, end, 3, cv::Scalar(255, 0, 0), -1);
    }
  }
}

void RPPController::drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose)
{
  if (grid_map_.empty())
  {
    return;
  }

  cv::Point robot_pt = worldToGrid(pose.pose.position.x, pose.pose.position.y);
  if (isPointInGrid(robot_pt))
  {
    cv::circle(grid_map_, robot_pt, 2, cv::Scalar(0, 165, 255), -1);
  }
}

void RPPController::drawLookaheadPointOnGrid(const geometry_msgs::msg::Point& lookahead_point)
{
  if (grid_map_.empty())
  {
    return;
  }

  cv::Point lookahead_pt = worldToGrid(lookahead_point.x, lookahead_point.y);
  if (isPointInGrid(lookahead_pt))
  {
    cv::circle(grid_map_, lookahead_pt, 3, cv::Scalar(255, 0, 255), -1);
  }
}

void RPPController::drawGridLines()
{
  if (grid_map_.empty())
  {
    return;
  }

  cv::Scalar grid_color(200, 200, 200);

  for (double x = grid_origin_x_; x < grid_origin_x_ + grid_width_; x += 0.1)
  {
    cv::Point pt1 = worldToGrid(x, grid_origin_y_);
    cv::Point pt2 = worldToGrid(x, grid_origin_y_ + grid_height_);
    if (isPointInGrid(pt1) && isPointInGrid(pt2))
    {
      cv::line(grid_map_, pt1, pt2, grid_color, 1);
    }
  }

  for (double y = grid_origin_y_; y < grid_origin_y_ + grid_height_; y += 0.1)
  {
    cv::Point pt1 = worldToGrid(grid_origin_x_, y);
    cv::Point pt2 = worldToGrid(grid_origin_x_ + grid_width_, y);
    if (isPointInGrid(pt1) && isPointInGrid(pt2))
    {
      cv::line(grid_map_, pt1, pt2, grid_color, 1);
    }
  }
}

void RPPController::saveGridMap()
{
  if (grid_map_.empty() || params_.debug.grid_map_path.empty())
  {
    return;
  }

  std::string filename = params_.debug.grid_map_path + "/rpp_path_tracking.png";
  cv::imwrite(filename, grid_map_);
}

void RPPController::updateGridMapIfNeeded(const geometry_msgs::msg::PoseStamped& current_pose,
                                           const geometry_msgs::msg::PoseStamped& lookahead_pose)
{
  if (!params_.debug.enable_grid_map)
  {
    return;
  }

  auto current_time = this->now();
  if ((current_time - last_grid_update_time_).seconds() > 1.0)
  {
    drawRobotOnGrid(current_pose);
    if (isValidPose(lookahead_pose))
    {
      drawLookaheadPointOnGrid(lookahead_pose.pose.position);
    }
    saveGridMap();
    last_grid_update_time_ = current_time;
  }
}

bool RPPController::isPointInGrid(const cv::Point& pt)
{
  return pt.x >= 0 && pt.x < grid_map_.cols && pt.y >= 0 && pt.y < grid_map_.rows;
}



}  // namespace follow_controller
}  // namespace xline
